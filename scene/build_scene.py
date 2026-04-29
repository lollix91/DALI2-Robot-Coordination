"""
Programmatically build the search-and-rescue CoppeliaSim scene.

The bridge expects the following object aliases at the scene root:

    /rescuer_1, /rescuer_2, /rescuer_3    -- Pioneer P3DX copies
    /rescuer_i/leftMotor, /rescuer_i/rightMotor   (already present in the
                                                   stock Pioneer model)
    /victim_1 ... /victim_N               -- coloured cuboids
    /depot, /charger                      -- floor markers

Run this script while CoppeliaSim is open (with an empty / default scene),
*before* starting the bridge:

    python scene/build_scene.py
        [--config scene/scenario.yaml]
        [--pioneer-model "models/robots/mobile/pioneer p3dx.ttm"]

It will overwrite any existing objects with the same aliases.
"""
from __future__ import annotations

import argparse
import logging
import math
import os
import sys
from typing import Optional

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError as e:
    raise SystemExit(
        "coppeliasim-zmqremoteapi-client is required. "
        "Install with: pip install coppeliasim-zmqremoteapi-client"
    ) from e


LOG_FORMAT = "[%(asctime)s] [%(levelname)s] [scene] %(message)s"
logging.basicConfig(level=logging.INFO, format=LOG_FORMAT, datefmt="%H:%M:%S")
log = logging.getLogger("scene")


# Layout (metres) -----------------------------------------------------------

ROBOT_SPAWNS = [
    ("rescuer_1", -5.0, -5.0,  math.pi / 4),   # NE-facing (45°)
    ("rescuer_2",  5.0, -5.0, -math.pi / 4),   # NW-facing (-45°)
    ("rescuer_3",  0.0,  5.0,  math.pi),       # South-facing (180°)
]

DEFAULT_VICTIMS = [
    ("victim_1",  5.0,  3.0, "light"),
    ("victim_2", -4.0, -3.0, "heavy"),
    ("victim_3", -3.0,  4.0, "light"),
]

DEPOT_XY    = (0.0,  4.7)

# Obstacles (placed before victims to create a more realistic scene)
DEFAULT_OBSTACLES = [
    ("rock_1",  2.0,  1.0, 0.35, [0.45, 0.40, 0.35]),   # brown rock
    ("rock_2", -1.5,  3.0, 0.40, [0.50, 0.45, 0.38]),   # brown rock
    ("rock_3",  3.0, -1.5, 0.30, [0.55, 0.50, 0.42]),   # brown rock
    ("crate_1",-3.0, -3.0, 0.50, [0.60, 0.55, 0.30]),   # wooden crate
    ("crate_2", 1.0,  3.5, 0.45, [0.58, 0.52, 0.28]),   # wooden crate
    ("barrel_1",4.0,  0.0, 0.35, [0.35, 0.35, 0.40]),   # metal barrel
    ("debris_1",-4.0, 1.0, 0.25, [0.40, 0.38, 0.35]),   # debris
    ("debris_2", 0.5, -2.0, 0.30, [0.42, 0.40, 0.37]),  # debris
]

# Vision sensor resolution
VISION_RES_X = 512
VISION_RES_Y = 512


# --------------------------------------------------------------------------

def safe_remove(sim, alias: str) -> None:
    """Remove an object/model identified by alias if present.

    Tries removeModel first, then removeObject. Both calls may raise
    if the alias points to a stale/half-deleted object; in that case we
    just swallow the error -- the next loadModel/createPrimitiveShape
    will create a brand-new instance under the same alias."""
    for attempt in (sim.removeModel, sim.removeObject):
        try:
            h = sim.getObject(f"/{alias}")
            attempt(h)
            return
        except Exception:
            continue


def _remove_model_scripts(sim, model_handle: int, alias: str) -> int:
    """Remove every script from a loaded model.

    CoppeliaSim's ``modelproperty_scripts_inactive`` flag does NOT always
    prevent the stock Lua demo script from running; the Pioneer's default
    obstacle-avoidance wander then competes with the bridge's motor
    commands.  Belt-and-suspenders: walk the tree and rip out every script.
    """
    removed = 0
    try:
        tree = sim.getObjectsInTree(model_handle, sim.handle_all, 0)
    except Exception:
        tree = [model_handle]

    # --- Method 1: sim.getScript with various type constants ----------------
    stypes = {}
    for attr in ('scripttype_childscript', 'scripttype_simulation',
                 'scripttype_customizationscript', 'scripttype_customization'):
        v = getattr(sim, attr, None)
        if v is not None:
            stypes[attr] = v
    for obj_h in tree:
        for attr, stype in stypes.items():
            # Try (stype, objHandle) and (stype, objHandle, '')
            for args in ((stype, obj_h), (stype, obj_h, '')):
                try:
                    sh = sim.getScript(*args)
                    if sh is not None and sh >= 0:
                        sim.removeScript(sh)
                        removed += 1
                        log.info("  Removed script (type=%s) from obj %d", attr, obj_h)
                        break
                except Exception:
                    continue

    # --- Method 2: look for script-type objects in the tree -----------------
    # In CoppeliaSim V4.6+, scripts are scene objects with their own type id.
    script_obj_type = getattr(sim, 'sceneobject_script', None)
    if script_obj_type is not None:
        try:
            scripts = sim.getObjectsInTree(model_handle, script_obj_type, 0)
            for sh in scripts:
                try:
                    sim.removeObject(sh)
                    removed += 1
                    log.info("  Removed script object %d", sh)
                except Exception:
                    pass
        except Exception:
            pass

    if removed:
        log.info("Removed %d script(s) from /%s", removed, alias)
    else:
        log.warning("Could not find/remove scripts from /%s -- "
                    "if the robot wanders, try removing scripts manually "
                    "in the CoppeliaSim GUI", alias)
    return removed


def load_pioneer(sim, model_path: str, alias: str,
                 x: float, y: float, theta: float) -> int:
    safe_remove(sim, alias)
    h = sim.loadModel(model_path)
    if h is None or h == -1:
        raise RuntimeError(
            f"Could not load Pioneer model from '{model_path}'. "
            "Pass --pioneer-model with an absolute path if necessary."
        )
    # Validate the handle: setObjectAlias will raise '354: object does
    # not exist' if loadModel silently failed (this happens when the
    # scene state is inconsistent, e.g. left over from a crashed run).
    try:
        sim.getObjectAlias(h)
    except Exception:
        raise RuntimeError(
            f"loadModel returned a stale handle for '{model_path}'. "
            "Close CoppeliaSim entirely and reopen with a fresh scene "
            "(File -> New scene), then re-run scene\\build_scene.py."
        )
    sim.setObjectAlias(h, alias)
    # Place above the floor so the model settles cleanly.
    sim.setObjectPosition(h, -1, [x, y, 0.13])
    # The Pioneer model's visual front faces local -X.
    # Add pi so the visual front aligns with the logical heading.
    sim.setObjectOrientation(h, -1, [0.0, 0.0, theta + math.pi])
    # Make the model fully non-dynamic (kinematic).  The bridge moves it
    # directly via setObjectPosition / setObjectOrientation every tick,
    # so we do NOT need the physics engine for the robot.  This also
    # prevents it from falling off the floor under gravity.
    try:
        props = sim.getModelProperty(h)
        sim.setModelProperty(
            h,
            props
            | sim.modelproperty_not_dynamic
            | sim.modelproperty_not_respondable
            | sim.modelproperty_scripts_inactive,
        )
    except Exception as e:
        log.warning("Could not set model properties on /%s: %s", alias, e)
    # Belt-and-suspenders: explicitly remove every script in the model.
    _remove_model_scripts(sim, h, alias)
    log.info("Loaded Pioneer P3DX as /%s at (%.2f, %.2f) -- kinematic mode",
             alias, x, y)
    return h


def make_victim(sim, vid: str, x: float, y: float, weight: str) -> int:
    safe_remove(sim, vid)
    # All victims are fluorescent-green cuboids; heavy ones are larger.
    if weight == "heavy":
        size = [0.45, 0.45, 0.45]
        color = [0.05, 0.95, 0.05]  # fluorescent green
        z = 0.225
    else:
        size = [0.30, 0.30, 0.30]
        color = [0.05, 0.95, 0.05]  # fluorescent green
        z = 0.150
    h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
    sim.setObjectAlias(h, vid)
    sim.setObjectPosition(h, -1, [x, y, z])
    try:
        sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, color)
    except Exception:
        pass
    # Make the cuboid non-dynamic so it stays put until carried.
    try:
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
    except Exception:
        pass
    log.info("Placed victim /%s (%s) at (%.2f, %.2f)", vid, weight, x, y)
    return h


def make_obstacle(sim, alias: str, x: float, y: float,
                  size: float, color: list[float]) -> int:
    """Place a static obstacle (cuboid of random-ish proportions) in the scene."""
    safe_remove(sim, alias)
    # Vary shape slightly for visual interest
    sx = size * 1.0
    sy = size * 0.8
    sz = size * 0.7
    h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [sx, sy, sz], 0)
    sim.setObjectAlias(h, alias)
    sim.setObjectPosition(h, -1, [x, y, sz / 2.0])
    try:
        sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, color)
    except Exception:
        pass
    try:
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 1)
    except Exception:
        pass
    log.info("Placed obstacle /%s at (%.2f, %.2f) size=%.2f", alias, x, y, size)
    return h


def add_vision_sensor(sim, robot_handle: int, alias: str) -> int:
    """Add a forward-facing vision sensor to a robot.

    The sensor is mounted on top of the robot, looking along the
    robot's direction of travel (kinematic +X).
    """
    sensor_alias = f"{alias}_camera"
    safe_remove(sim, sensor_alias)
    # Create a vision sensor
    options = 2  # bit1 = perspective projection
    int_params = [
        VISION_RES_X,  # resolution x
        VISION_RES_Y,  # resolution y
        0,             # reserved
        0,             # reserved
    ]
    float_params = [
        0.15,   # near clipping plane (skip robot body)
        10.0,   # far clipping plane
        60.0 * 3.14159 / 180.0,  # view angle (60 deg)
        0.0,    # reserved
        0.0,    # reserved
        0.0,    # reserved
        0.0, 0.0, 0.0, 0.0, 0.0,  # reserved
    ]
    h = sim.createVisionSensor(options, int_params, float_params)
    sim.setObjectAlias(h, sensor_alias)
    # Mount on top of robot, looking along the direction of travel.
    # We use setObjectMatrix (3x4 row-major) to avoid Euler-angle ambiguity.
    #
    # Vision sensors look along their local -Z.  The bridge's kinematic
    # controller drives the robot along +X (with HEADING_OFFSET rotating
    # the model so its visual front aligns with the heading).  We orient
    # the camera so it captures what lies ahead in the travel direction:
    #   camera -Z  ->  robot +X   (direction of travel)
    #   camera +Y  ->  robot +Z   (up / world up)
    #   camera +X  ->  robot -Y   (right-hand rule)
    #
    # Rotation matrix (columns = where camera X,Y,Z land in robot frame):
    #   R = [[ 0,  0, -1],
    #        [-1,  0,  0],
    #        [ 0,  1,  0]]
    # Position relative to robot: +30 cm in X (forward in direction of
    # travel), 25 cm above centre.
    matrix = [
         0,  0, -1,  0.30,   # row 0
        -1,  0,  0,  0.00,   # row 1
         0,  1,  0,  0.25,   # row 2
    ]
    sim.setObjectParent(h, robot_handle, True)
    sim.setObjectMatrix(h, robot_handle, matrix)
    log.info("Added vision sensor /%s to /%s", sensor_alias, alias)
    return h


def make_marker(sim, alias: str, x: float, y: float,
                color: list[float], radius: float = 0.4,
                height: float = 0.35) -> int:
    """Place a 3-D cylinder marker visible from ground-level cameras."""
    safe_remove(sim, alias)
    h = sim.createPrimitiveShape(sim.primitiveshape_cylinder,
                                 [radius * 2, radius * 2, height], 0)
    sim.setObjectAlias(h, alias)
    sim.setObjectPosition(h, -1, [x, y, height / 2.0])
    try:
        sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, color)
    except Exception:
        pass
    try:
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
    except Exception:
        pass
    log.info("Placed marker /%s at (%.2f, %.2f)", alias, x, y)
    return h


def load_victims_cfg(path: Optional[str]):
    if not path:
        return DEFAULT_VICTIMS
    try:
        import yaml
    except ImportError:
        log.warning("PyYAML not installed, ignoring --config")
        return DEFAULT_VICTIMS
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    out = []
    for v in data.get("victims", []):
        out.append((v["id"], float(v["x"]), float(v["y"]),
                    v.get("weight", "light")))
    return out or DEFAULT_VICTIMS


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--pioneer-model",
                   default="models/robots/mobile/pioneer p3dx.ttm",
                   help="Path to the Pioneer P3DX .ttm model "
                        "(relative to the CoppeliaSim install directory, "
                        "or absolute).")
    p.add_argument("--config", default=None,
                   help="Optional YAML scenario file (see scene/scenario.yaml)")
    p.add_argument("--keep-current", action="store_true",
                   help="Do not call sim.stopSimulation() before building.")
    args = p.parse_args()

    client = RemoteAPIClient()
    sim = client.require("sim")

    if not args.keep_current:
        try:
            if sim.getSimulationState() != sim.simulation_stopped:
                sim.stopSimulation()
                log.info("Stopped simulation before scene rebuild")
        except Exception:
            pass

    # 1. Robots + vision sensors -------------------------------------------
    for alias, x, y, theta in ROBOT_SPAWNS:
        rh = load_pioneer(sim, args.pioneer_model, alias, x, y, theta)
        add_vision_sensor(sim, rh, alias)

    # 2. Obstacles (placed before victims for realism) ----------------------
    for alias, x, y, size, color in DEFAULT_OBSTACLES:
        make_obstacle(sim, alias, x, y, size, color)

    # 3. Victims -------------------------------------------------------------
    for vid, x, y, weight in load_victims_cfg(args.config):
        make_victim(sim, vid, x, y, weight)

    # 4. Depot marker -------------------------------------------
    make_marker(sim, "depot",   DEPOT_XY[0],   DEPOT_XY[1],   [0.20, 0.40, 0.95])

    log.info("Scene built. You can now press 'Play' in CoppeliaSim or let the "
             "bridge start the simulation automatically.")


if __name__ == "__main__":
    main()
