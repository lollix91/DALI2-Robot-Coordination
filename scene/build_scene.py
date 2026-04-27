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
    ("rescuer_1", -1.5, -2.5,  0.0),
    ("rescuer_2",  0.0, -2.5,  0.0),
    ("rescuer_3",  1.5, -2.5,  0.0),
]

DEFAULT_VICTIMS = [
    ("victim_1",  4.0,  3.0, "light"),
    ("victim_2", -3.5,  2.5, "heavy"),
    ("victim_3",  0.0, -4.0, "light"),
    ("victim_4",  4.0, -3.5, "heavy"),
    ("victim_5", -2.0, -1.5, "light"),
]

DEPOT_XY    = (0.0,  4.7)
CHARGER_XY  = (-4.7, -4.7)


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
    sim.setObjectOrientation(h, -1, [0.0, 0.0, theta])
    # Disable the stock Pioneer child Lua script -- otherwise its
    # default obstacle-avoidance demo would compete with the bridge's
    # wheel commands and the robot would wander on its own.
    try:
        props = sim.getModelProperty(h)
        sim.setModelProperty(h, props | sim.modelproperty_scripts_inactive)
    except Exception as e:
        log.warning("Could not disable stock scripts on /%s: %s", alias, e)
    log.info("Loaded Pioneer P3DX as /%s at (%.2f, %.2f) -- scripts disabled",
             alias, x, y)
    return h


def make_victim(sim, vid: str, x: float, y: float, weight: str) -> int:
    safe_remove(sim, vid)
    # Light victims are short green cuboids; heavy victims are larger red.
    if weight == "heavy":
        size = [0.45, 0.45, 0.45]
        color = [0.85, 0.10, 0.10]
        z = 0.225
    else:
        size = [0.30, 0.30, 0.30]
        color = [0.10, 0.75, 0.20]
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


def make_marker(sim, alias: str, x: float, y: float,
                color: list[float], radius: float = 0.6) -> int:
    safe_remove(sim, alias)
    h = sim.createPrimitiveShape(sim.primitiveshape_disc,
                                 [radius * 2, radius * 2, 0.005], 0)
    sim.setObjectAlias(h, alias)
    sim.setObjectPosition(h, -1, [x, y, 0.005])
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

    # 1. Robots --------------------------------------------------------------
    for alias, x, y, theta in ROBOT_SPAWNS:
        load_pioneer(sim, args.pioneer_model, alias, x, y, theta)

    # 2. Victims -------------------------------------------------------------
    for vid, x, y, weight in load_victims_cfg(args.config):
        make_victim(sim, vid, x, y, weight)

    # 3. Markers (depot, charger) -------------------------------------------
    make_marker(sim, "depot",   DEPOT_XY[0],   DEPOT_XY[1],   [0.20, 0.40, 0.95])
    make_marker(sim, "charger", CHARGER_XY[0], CHARGER_XY[1], [0.95, 0.85, 0.10])

    log.info("Scene built. You can now press 'Play' in CoppeliaSim or let the "
             "bridge start the simulation automatically.")


if __name__ == "__main__":
    main()
