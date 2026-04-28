"""
DALI2 <-> CoppeliaSim bridge.

Subscribes to the DALI2 LINDA pub/sub channel on Redis and acts as the
virtual `sim` agent: actuation commands sent by DALI2 rescuer agents are
translated into CoppeliaSim ZMQ Remote API calls; sensor / event updates
generated from the simulation are published back as LINDA messages
addressed to the corresponding rescuer agent.

Message protocol (LINDA: "TO:CONTENT:FROM"):

    Inbound (TO=sim, FROM=rescuer_i):
        set_target(rescuer_i, X, Y)
        go_to_depot(rescuer_i)
        go_to_charger(rescuer_i)
        attach(rescuer_i, victim_id)
        release(rescuer_i, victim_id)

    Outbound (TO=rescuer_i, FROM=sim):
        position(X, Y, Theta)
        battery(Level)
        at_target
        obstacle_detected(Distance)
        victim_in_range(Id, X, Y, Weight)
        delivered(Id)

Run:
    python coppelia_bridge.py
        [--redis-host HOST] [--redis-port PORT] [--tick-hz HZ]
        [--config bridge/scenario.yaml]

Requires:
    pip install -r bridge/requirements.txt
and a running Redis + a running CoppeliaSim instance with the scene
created by `scene/build_scene.py` (or equivalent).
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import logging
import math
import os
import queue
import re
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import redis

try:
    from PIL import Image
except ImportError:
    Image = None  # graceful degradation -- screenshots disabled

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError as e:
    raise SystemExit(
        "coppeliasim-zmqremoteapi-client is required. "
        "Install with: pip install coppeliasim-zmqremoteapi-client"
    ) from e


# --------------------------------------------------------------------------
# Default scenario configuration (overridable from a YAML file at runtime).
# --------------------------------------------------------------------------

ROBOT_NAMES = ["rescuer_1", "rescuer_2", "rescuer_3"]

# Each victim: (id, x, y, weight in {"light", "heavy"})
DEFAULT_VICTIMS = [
    ("victim_1",  4.0,  3.0, "light"),
    ("victim_2", -3.5,  2.5, "heavy"),
    ("victim_3",  0.0, -4.0, "light"),
    ("victim_4",  4.0, -3.5, "heavy"),
    ("victim_5", -2.0, -1.5, "light"),
]

DEPOT_XY    = (0.0,  4.7)
CHARGER_XY  = (-4.7, -4.7)

DETECTION_RADIUS = 3.0      # m -- victim sensing radius; robots must
                            #   physically explore to discover victims
ARRIVAL_RADIUS   = 0.40     # m -- consider target reached
ROBOT_Z          = 0.13     # m -- fixed height (kinematic, no gravity)
HEADING_OFFSET   = math.pi  # Pioneer model's visual front is -X; add pi so
                            #   the body visually faces the direction of travel
MAX_LIN_VEL      = 0.45     # m/s
MAX_ANG_VEL      = 1.40     # rad/s
K_RHO            = 1.5
K_ALPHA          = 4.0

BATTERY_DRAIN_MOVING = 0.5  # %/s
BATTERY_DRAIN_IDLE   = 0.05 # %/s
BATTERY_CHARGE_RATE  = 5.0  # %/s while at charger

SCREENSHOT_INTERVAL  = 5.0  # seconds between screenshots per robot
SCREENSHOT_BASE_DIR  = "screenshots"

# Vision LLM settings (JAN local server or any OpenAI-compatible API)
VISION_LLM_ENDPOINT = os.environ.get(
    "VISION_LLM_ENDPOINT", "http://localhost:1337/v1/chat/completions")
VISION_LLM_MODEL = os.environ.get("VISION_LLM_MODEL", "Qwen3_5-9B-IQ4_XS")
VISION_LLM_ENABLED = os.environ.get("VISION_LLM_ENABLED", "true").lower() in ("1", "true", "yes")

LOG_FORMAT = "[%(asctime)s] [%(levelname)s] [bridge] %(message)s"
logging.basicConfig(level=logging.INFO, format=LOG_FORMAT, datefmt="%H:%M:%S")
log = logging.getLogger("bridge")


# --------------------------------------------------------------------------
# Robot state
# --------------------------------------------------------------------------

@dataclass
class RobotState:
    name: str
    body_handle: int
    left_motor: Optional[int] = None
    right_motor: Optional[int] = None
    target_xy: Optional[tuple[float, float]] = None
    arrived_published: bool = True
    carrying: Optional[str] = None
    battery: float = 100.0
    moving: bool = False
    seen_victims: set[str] = field(default_factory=set)
    camera_handle: Optional[int] = None
    last_screenshot: float = 0.0
    screenshot_count: int = 0
    last_pos_publish: float = 0.0
    last_bat_publish: float = 0.0
    waiting_for_vision: bool = False


@dataclass
class VictimState:
    vid: str
    handle: int
    weight: str
    delivered: bool = False
    carried_by: Optional[str] = None


# --------------------------------------------------------------------------
# LINDA helpers
# --------------------------------------------------------------------------

# Match TO:CONTENT:FROM where TO and FROM contain no ':'.
LINDA_RE = re.compile(r"^([^:]+):(.*):([^:]+)$")


def parse_linda(payload: str) -> Optional[tuple[str, str, str]]:
    """Parse a LINDA message ``TO:CONTENT:FROM``."""
    m = LINDA_RE.match(payload)
    if not m:
        return None
    return m.group(1), m.group(2), m.group(3)


def fmt_linda(to: str, content: str, frm: str) -> str:
    return f"{to}:{content}:{frm}"


# --------------------------------------------------------------------------
# Bridge
# --------------------------------------------------------------------------

class CoppeliaBridge:
    SIM_AGENT = "sim"

    def __init__(self, redis_host: str, redis_port: int, tick_hz: float,
                 victims_cfg: list[tuple[str, float, float, str]]):
        self.r = redis.Redis(host=redis_host, port=redis_port, decode_responses=True)
        self.r.ping()
        self.dt = 1.0 / tick_hz

        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")

        self.robots: dict[str, RobotState] = {}
        self.victims: dict[str, VictimState] = {}
        self.depot_xy = DEPOT_XY
        self.charger_xy = CHARGER_XY
        self.victims_cfg = victims_cfg

        self._stop = threading.Event()
        self._sub_thread: Optional[threading.Thread] = None
        self._cmd_queue: queue.Queue = queue.Queue()

    # ----------------------------------------------------------------------
    # Setup
    # ----------------------------------------------------------------------

    def setup(self) -> None:
        """Resolve all object handles in the running CoppeliaSim scene."""
        for name in ROBOT_NAMES:
            try:
                body = self.sim.getObject(f"/{name}")
            except Exception as e:
                raise RuntimeError(
                    f"Object '/{name}' not found in the scene -- "
                    f"did you run scene/build_scene.py?  Error: {e}"
                )
            self.robots[name] = RobotState(name=name, body_handle=body)
            # Resolve vision sensor (camera)
            try:
                cam = self.sim.getObject(f"/{name}_camera")
                self.robots[name].camera_handle = cam
                log.info("Robot %s camera ready (handle=%d)", name, cam)
            except Exception:
                log.warning("No vision sensor found for %s -- screenshots disabled", name)
            # Create screenshot directory
            sdir = os.path.join(SCREENSHOT_BASE_DIR, name)
            os.makedirs(sdir, exist_ok=True)
            log.info("Robot %s ready (handle=%d)", name, body)

        for vid, _, _, weight in self.victims_cfg:
            try:
                h = self.sim.getObject(f"/{vid}")
            except Exception as e:
                raise RuntimeError(
                    f"Victim '/{vid}' not found in the scene "
                    f"(check scene/build_scene.py). Underlying error: {e}"
                )
            self.victims[vid] = VictimState(vid=vid, handle=h, weight=weight)
            log.info("Victim %s (%s) ready (handle=%d)", vid, weight, h)

        log.info("Depot=%s  Charger=%s", self.depot_xy, self.charger_xy)

    # ----------------------------------------------------------------------
    # Redis pub/sub
    # ----------------------------------------------------------------------

    def publish(self, to: str, content: str) -> None:
        msg = fmt_linda(to, content, self.SIM_AGENT)
        self.r.publish("LINDA", msg)
        log.debug("OUT %s", msg)

    def _sub_loop(self) -> None:
        ps = self.r.pubsub(ignore_subscribe_messages=True)
        ps.subscribe("LINDA")
        log.info("Subscribed to LINDA channel")
        for raw in ps.listen():
            if self._stop.is_set():
                break
            if raw["type"] != "message":
                continue
            payload = raw["data"]
            parsed = parse_linda(payload)
            if parsed is None:
                continue
            to, content, frm = parsed
            if to != self.SIM_AGENT or frm == self.SIM_AGENT:
                continue
            self._cmd_queue.put((content, frm))

    # ----------------------------------------------------------------------
    # Command dispatch (DALI2 -> CoppeliaSim)
    # ----------------------------------------------------------------------

    _CMD_RE = re.compile(r"^\s*(\w+)\s*\((.*)\)\s*$|^\s*(\w+)\s*$")

    def _handle_command(self, content: str, sender: str) -> None:
        m = self._CMD_RE.match(content)
        if not m:
            log.warning("Malformed command from %s: %r", sender, content)
            return
        if m.group(3):
            head, args = m.group(3), []
        else:
            head, args_raw = m.group(1), m.group(2)
            args = [a.strip() for a in args_raw.split(",")] if args_raw else []
        log.info("CMD <- %s : %s%s", sender, head, args)

        handler = getattr(self, f"_cmd_{head}", None)
        if handler is None:
            log.warning("Unknown command %s from %s", head, sender)
            return
        handler(args, sender)

    def _cmd_set_target(self, args, sender):
        # set_target(rescuer_i, X, Y)
        robot, x, y = args[0], float(args[1]), float(args[2])
        st = self.robots.get(robot)
        if st is None:
            return
        st.target_xy = (x, y)
        st.arrived_published = False
        log.info("Target for %s -> (%.2f, %.2f)", robot, x, y)

    def _cmd_go_to_depot(self, args, sender):
        robot = args[0]
        st = self.robots.get(robot)
        if st is None:
            return
        st.target_xy = self.depot_xy
        st.arrived_published = False

    def _cmd_go_to_charger(self, args, sender):
        robot = args[0]
        st = self.robots.get(robot)
        if st is None:
            return
        st.target_xy = self.charger_xy
        st.arrived_published = False

    def _cmd_attach(self, args, sender):
        # attach(rescuer_i, victim_id)
        robot, vid = args[0], args[1]
        rs = self.robots.get(robot)
        vs = self.victims.get(vid)
        if rs is None or vs is None or vs.delivered:
            return
        # For heavy victims we still attach to one robot; the other moves
        # in formation.  The DALI2 cooperative-lift constraint guarantees
        # both robots are physically at the victim before this is invoked.
        rp = self.sim.getObjectPosition(rs.body_handle, -1)
        # Lift victim slightly above the robot
        try:
            self.sim.setObjectParent(vs.handle, rs.body_handle, True)
            self.sim.setObjectPosition(vs.handle, rs.body_handle, [0.0, 0.0, 0.30])
        except Exception as e:
            log.warning("Attach %s -> %s failed: %s", vid, robot, e)
        rs.carrying = vid
        vs.carried_by = robot
        log.info("Attached %s to %s @ (%.2f,%.2f,%.2f)", vid, robot, *rp[:3])

    def _cmd_release(self, args, sender):
        # release(rescuer_i, victim_id)
        robot, vid = args[0], args[1]
        rs = self.robots.get(robot)
        vs = self.victims.get(vid)
        if rs is None or vs is None:
            return
        try:
            self.sim.setObjectParent(vs.handle, -1, True)
        except Exception as e:
            log.warning("Detach failed: %s", e)
        rs.carrying = None
        # Mark delivered if dropped near depot.
        rp = self.sim.getObjectPosition(rs.body_handle, -1)
        d = math.hypot(rp[0] - self.depot_xy[0], rp[1] - self.depot_xy[1])
        if d <= 1.2:
            vs.delivered = True
            vs.carried_by = None
            try:
                self.sim.setObjectPosition(vs.handle, -1,
                                           [self.depot_xy[0],
                                            self.depot_xy[1],
                                            0.10])
            except Exception:
                pass
            self.publish(robot, f"delivered({vid})")
            log.info("Delivered %s at depot via %s", vid, robot)
        else:
            log.info("Released %s at non-depot location (d=%.2f)", vid, d)

    # ----------------------------------------------------------------------
    # Control + sensing tick (CoppeliaSim -> DALI2)
    # ----------------------------------------------------------------------

    def control_tick(self) -> None:
        # Process queued commands from subscription thread (thread-safe).
        while not self._cmd_queue.empty():
            try:
                content, frm = self._cmd_queue.get_nowait()
                self._handle_command(content, frm)
            except queue.Empty:
                break
            except Exception as e:
                log.exception("Error handling command: %s", e)
        for st in self.robots.values():
            self._control_robot(st)
            self._sense_robot(st)
            self._capture_screenshot(st)

    def _control_robot(self, st: RobotState) -> None:
        try:
            pos = self.sim.getObjectPosition(st.body_handle, -1)
            ori = self.sim.getObjectOrientation(st.body_handle, -1)
        except Exception:
            return
        x, y = pos[0], pos[1]
        theta = self._wrap_angle(ori[2] - HEADING_OFFSET)

        # Battery dynamics --------------------------------------------------
        if st.target_xy is not None:
            st.battery = max(0.0, st.battery - BATTERY_DRAIN_MOVING * self.dt)
        else:
            st.battery = max(0.0, st.battery - BATTERY_DRAIN_IDLE * self.dt)
        # Charge while idle on charger
        if (st.target_xy is None and
                math.hypot(x - self.charger_xy[0], y - self.charger_xy[1]) < 0.6):
            st.battery = min(100.0, st.battery + BATTERY_CHARGE_RATE * self.dt)

        # Go-to-goal control (kinematic) ------------------------------------
        if st.waiting_for_vision:
            st.moving = False
            return
        if st.target_xy is not None:
            tx, ty = st.target_xy
            dx, dy = tx - x, ty - y
            rho = math.hypot(dx, dy)
            if rho < ARRIVAL_RADIUS:
                if not st.arrived_published:
                    st.arrived_published = True
                    st.target_xy = None
                    self.publish(st.name, "at_target")
                    log.info("%s reached target", st.name)
                return
            target_th = math.atan2(dy, dx)
            alpha = self._wrap_angle(target_th - theta)
            v = min(MAX_LIN_VEL, K_RHO * rho)
            w = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, K_ALPHA * alpha))
            # Slow down when not aligned
            if abs(alpha) > 0.6:
                v *= 0.3
            # Integrate unicycle model
            new_theta = theta + w * self.dt
            new_x = x + v * math.cos(new_theta) * self.dt
            new_y = y + v * math.sin(new_theta) * self.dt
            try:
                self.sim.setObjectPosition(
                    st.body_handle, -1, [new_x, new_y, ROBOT_Z])
                self.sim.setObjectOrientation(
                    st.body_handle, -1,
                    [0.0, 0.0, new_theta + HEADING_OFFSET])
            except Exception as e:
                log.debug("setObjectPosition failed for %s: %s", st.name, e)
            if not getattr(st, '_ctrl_logged', False):
                log.info("%s kinematic: pos=(%.2f,%.2f) th=%.2f "
                         "target=(%.2f,%.2f) alpha=%.2f v=%.2f w=%.2f",
                         st.name, x, y, theta, tx, ty, alpha, v, w)
                st._ctrl_logged = True
        st.moving = (st.target_xy is not None)

    def _sense_robot(self, st: RobotState) -> None:
        try:
            pos = self.sim.getObjectPosition(st.body_handle, -1)
            ori = self.sim.getObjectOrientation(st.body_handle, -1)
        except Exception:
            return
        x, y = pos[0], pos[1]
        theta = self._wrap_angle(ori[2] - HEADING_OFFSET)

        # Throttled position + battery telemetry.
        now = time.time()
        if now - st.last_pos_publish >= 1.0:
            st.last_pos_publish = now
            self.publish(st.name, f"position({x:.3f},{y:.3f},{theta:.3f})")
        if now - st.last_bat_publish >= 5.0:
            st.last_bat_publish = now
            self.publish(st.name, f"battery({st.battery:.0f})")

        # Victim detection: announce a victim once when first seen.
        for vs in self.victims.values():
            if vs.delivered or vs.carried_by is not None:
                continue
            try:
                vp = self.sim.getObjectPosition(vs.handle, -1)
            except Exception:
                continue
            d = math.hypot(x - vp[0], y - vp[1])
            if d <= DETECTION_RADIUS and vs.vid not in st.seen_victims:
                st.seen_victims.add(vs.vid)
                self.publish(st.name,
                             f"victim_in_range({vs.vid},{vp[0]:.3f},"
                             f"{vp[1]:.3f},{vs.weight})")
                log.info("%s detected %s @ (%.2f,%.2f) [%s]",
                         st.name, vs.vid, vp[0], vp[1], vs.weight)

    # ------------------------------------------------------------------
    # Vision: screenshot capture + LLM analysis
    # ------------------------------------------------------------------

    def _capture_screenshot(self, st: RobotState) -> None:
        """Capture an image from the robot's vision sensor and save it."""
        if st.camera_handle is None or Image is None:
            return
        now = time.time()
        if now - st.last_screenshot < SCREENSHOT_INTERVAL:
            return
        st.last_screenshot = now
        try:
            img_raw, res = self.sim.getVisionSensorImg(st.camera_handle)
            if not img_raw or not res or res[0] <= 0:
                return
            w, h = res[0], res[1]
            # CoppeliaSim returns raw RGB bytes (bottom-up)
            if isinstance(img_raw, bytes):
                raw_bytes = img_raw
            else:
                raw_bytes = bytes(img_raw)
            img = Image.frombytes("RGB", (w, h), raw_bytes)
            img = img.transpose(Image.FLIP_TOP_BOTTOM)  # CoppeliaSim is bottom-up
        except Exception as e:
            log.debug("Screenshot failed for %s: %s", st.name, e)
            return

        st.screenshot_count += 1
        fname = f"image{st.screenshot_count}.jpg"
        fpath = os.path.join(SCREENSHOT_BASE_DIR, st.name, fname)
        try:
            img.save(fpath, "JPEG", quality=85)
        except Exception as e:
            log.warning("Failed to save screenshot %s: %s", fpath, e)
            return
        abs_path = os.path.abspath(fpath).replace("\\", "/")
        log.info("Screenshot %s: %s", st.name, fname)
        # Notify the agent via LINDA (use forward slashes -- Prolog chokes on \)
        self.publish(st.name, f"screenshot('{abs_path}')")
        # Analyse via vision LLM in background thread; robot pauses until done.
        if VISION_LLM_ENABLED:
            st.waiting_for_vision = True
            threading.Thread(
                target=self._analyse_screenshot,
                args=(st.name, abs_path, img),
                daemon=True
            ).start()

    def _analyse_screenshot(self, robot_name: str, image_path: str,
                            img: "Image.Image") -> None:
        """Send a screenshot to the vision LLM and publish the result."""
        try:
            import urllib.request
            # Encode image as base64 JPEG
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=85)
            b64 = base64.b64encode(buf.getvalue()).decode("ascii")

            prompt = (
                "You are a rescue drone camera. Describe what you see in this image "
                "from a search-and-rescue robot's point of view. "
                "Focus on: Are there any victims (red cubes of different sizes)? "
                "Any obstacles (brown/grey blocks)? "
                "Is the path clear? Is there a depot (blue cylinder) or "
                "charger (yellow cylinder)? "
                "Reply with a short factual description (max 2 sentences)."
            )

            body = {
                "model": VISION_LLM_MODEL or "gpt-4o-mini",
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{b64}"
                                }
                            }
                        ]
                    }
                ],
                "max_tokens": 150,
                "temperature": 0.3
            }
            data = json.dumps(body).encode("utf-8")
            req = urllib.request.Request(
                VISION_LLM_ENDPOINT,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=30) as resp:
                result = json.loads(resp.read().decode("utf-8"))
            content = result["choices"][0]["message"]["content"]
            # Sanitise for Prolog (escape quotes, limit length)
            content = content.replace("'", "\''").replace('"', '').strip()
            if len(content) > 300:
                content = content[:297] + "..."
            log.info("Vision LLM [%s]: %s", robot_name, content)
            # Publish analysis result to the agent
            safe_content = content.replace(",", " ").replace("(", "").replace(")", "")
            self.publish(robot_name,
                         f"vision_analysis('{safe_content}')")
        except Exception as e:
            log.warning("Vision LLM analysis failed for %s: %s", robot_name, e)
        finally:
            # Un-gate movement regardless of success/failure.
            rs = self.robots.get(robot_name)
            if rs:
                rs.waiting_for_vision = False

    @staticmethod
    def _wrap_angle(a: float) -> float:
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    # ----------------------------------------------------------------------
    # Lifecycle
    # ----------------------------------------------------------------------

    def run(self) -> None:
        self.setup()
        # Make sure the simulation is running -- start it if necessary.
        try:
            state = self.sim.getSimulationState()
            if state == self.sim.simulation_stopped:
                self.sim.startSimulation()
                log.info("Simulation started")
        except Exception:
            pass

        self._sub_thread = threading.Thread(
            target=self._sub_loop, name="linda-sub", daemon=True)
        self._sub_thread.start()
        log.info("Bridge running -- ctrl+C to stop")
        try:
            next_tick = time.time()
            while not self._stop.is_set():
                self.control_tick()
                next_tick += self.dt
                sleep = next_tick - time.time()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_tick = time.time()
        except KeyboardInterrupt:
            pass
        finally:
            self._stop.set()
            for st in self.robots.values():
                self._set_wheels(st, 0.0, 0.0)
            log.info("Bridge stopped")


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def load_config(path: str) -> list[tuple[str, float, float, str]]:
    """Load victim configuration from a YAML file (optional)."""
    try:
        import yaml
    except ImportError:
        log.warning("PyYAML not installed, ignoring --config")
        return DEFAULT_VICTIMS
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    out = []
    for v in data.get("victims", []):
        out.append((v["id"], float(v["x"]), float(v["y"]), v.get("weight", "light")))
    if not out:
        return DEFAULT_VICTIMS
    return out


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--redis-host", default="localhost")
    p.add_argument("--redis-port", type=int, default=6379)
    p.add_argument("--tick-hz", type=float, default=10.0)
    p.add_argument("--config", default=None,
                   help="Optional YAML scenario file (see scene/scenario.yaml)")
    args = p.parse_args()

    victims = load_config(args.config) if args.config else DEFAULT_VICTIMS
    bridge = CoppeliaBridge(args.redis_host, args.redis_port,
                            args.tick_hz, victims)
    bridge.run()


if __name__ == "__main__":
    main()
