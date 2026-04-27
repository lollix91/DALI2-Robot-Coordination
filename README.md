# DALI2 Multi-Robot Search-and-Rescue

A digital-twin case study showing how the [DALI2](../DALI2)
multi-agent platform can drive a team of mobile robots inside
**CoppeliaSim**.  The scenario: three Pioneer P3DX rescuers cooperate to
locate victims in an arena and bring them to a safe zone.  Some victims
are **heavy** and require two robots to lift them simultaneously, which
showcases DALI2's cooperative-coordination features (FIPA-style
auctioning, multi-event synchronisation with a delta-t window, told/tell
priority queues, internal-event deliberation, optional LLM oracle).

```
   +------------------+   Redis pub/sub   +-------------------+   ZeroMQ Remote API   +-------------+
   | DALI2 agents     |<----------------->|  Python bridge    |<-------------------->| CoppeliaSim |
   | (SWI-Prolog)     |   LINDA channel   |  (this directory) |  set/get joint vel,  |  digital    |
   |                  |                   |                   |  object pose, ...    |  twin       |
   |  coordinator     |                   |  go-to-goal ctrl  |                      |             |
   |  rescuer_1..3    |                   |  battery sim      |                      |  Pioneer +  |
   |  monitor         |                   |  victim sensor    |                      |  victims    |
   +------------------+                   +-------------------+                      +-------------+
```

## Repository layout

```
DALI2-Robot-Coordination/
|-- agents/
|   `-- rescue.pl              # All four DALI2 agents in one file
|-- bridge/
|   |-- coppelia_bridge.py     # ZMQ <-> Redis bridge
|   `-- requirements.txt
|-- scene/
|   |-- build_scene.py         # Programmatic scene setup (Pioneer + victims)
|   `-- scenario.yaml          # Victim positions / weights
|-- docker-compose.yml         # Optional Redis container
`-- README.md
```

## Prerequisites

| Component                      | Version                     | Purpose                                  |
|--------------------------------|-----------------------------|------------------------------------------|
| **CoppeliaSim Edu**            | >= 4.5                      | The digital twin (ZMQ Remote API ships)  |
| **Python**                     | >= 3.10                     | Bridge + scene builder                   |
| **SWI-Prolog**                 | >= 9.0                      | DALI2 runtime                            |
| **Redis**                      | >= 6.0 (or Docker)          | DALI2 LINDA channel                      |
| **DALI2 source tree**          | this repo's sibling         | The framework itself                     |

CoppeliaSim must be installed locally (downloadable from
[coppeliarobotics.com](https://www.coppeliarobotics.com/downloads)).
The Pioneer P3DX model that the scene builder loads ships with the
default install (`models/robots/mobile/pioneer p3dx.ttm`).

## One-time setup

```powershell
# 1. Install Python dependencies
python -m pip install -r bridge\requirements.txt

# 2. (Optional) start Redis with Docker
docker run -d --name dali2-redis -p 6379:6379 redis:7-alpine

# 3. (Optional) verify SWI-Prolog
swipl --version
```

## Running the demo

Open four terminals (numbers 2/3/4 can be in any order, but Redis must be
up first).

### Terminal 1 -- Redis

If you used the Docker command above it is already running.  Otherwise:

```powershell
redis-server
```

### Terminal 2 -- CoppeliaSim

1. Launch CoppeliaSim.
2. Open a new (empty) scene -- *File -> New Scene*.
3. Press **Play** once so the ZMQ Remote API service is available, then
   press **Stop**.  (Alternatively, you can just leave the scene stopped;
   the bridge will start the simulation for you in step 4.)

### Terminal 3 -- Build the scene

```powershell
python scene\build_scene.py
```

This adds three Pioneer P3DX robots renamed `/rescuer_1..3`, five
coloured cuboid victims (`/victim_1..5`, two of which are heavy/red),
plus depot and charger floor markers.  Edit `scene\scenario.yaml` and
re-run if you want different victim positions.

### Terminal 4 -- DALI2 + bridge

Start the DALI2 server (assumes the DALI2 source tree is at
`..\DALI2`):

```powershell
swipl -l ..\DALI2\src\server.pl -g main -- 8080 agents\rescue.pl
```

In a fifth terminal (or detach the previous one), start the bridge:

```powershell
python bridge\coppelia_bridge.py
```

The bridge auto-starts the CoppeliaSim simulation if it is stopped.
Within ~2 seconds you should see the rescuers begin patrolling and
reporting victim sightings to the coordinator, which runs an auction
for each new sighting.

Open <http://localhost:8080> in a browser to inspect agent beliefs,
past events, and live logs.

## What to watch for

| In CoppeliaSim                                              | In the DALI2 web UI / logs                                              |
|-------------------------------------------------------------|-------------------------------------------------------------------------|
| Each rescuer drives toward different parts of the arena.    | `position_update` messages stream into the coordinator at low priority. |
| A rescuer turns toward a green (light) victim.              | `victim_seen(victim_3, ..., light)` -> auction -> single robot wins.    |
| Two rescuers converge on the same red (heavy) victim.       | `assigned_pair(victim_2, R_a, R_b)` -> two `ready_to_lift` -> `sync_lift` internal event fires. |
| The carrier robot drives the victim back to the blue depot. | `delivered(victim_X)` -> `victim_rescued`.                              |
| A rescuer's wheels stall as battery falls below 25%.        | `low_battery(R, L)` -> coordinator releases the task and re-auctions.   |

## Optional: LLM-driven triage

DALI2 ships with an AI Oracle that can call any LLM exposed by
[OpenRouter](https://openrouter.ai).  When a key is provided, the
coordinator's `victim_seen` handler asks the model for a triage
suggestion:

```powershell
$env:OPENROUTER_API_KEY = "sk-or-..."
swipl -l ..\DALI2\src\server.pl -g main -- 8080 agents\rescue.pl
```

The advice is logged but not used to override the deterministic
auction; this preserves the safety guarantees of the symbolic layer
while letting the LLM contribute soft preferences.

## Troubleshooting

* **`Object '/rescuer_1' not found`** -- run `scene/build_scene.py` while
  CoppeliaSim is open, then start the bridge.
* **`Pioneer model not found`** -- pass an absolute path:
  `python scene/build_scene.py --pioneer-model "C:\Program Files\CoppeliaSim_Edu\models\robots\mobile\pioneer p3dx.ttm"`.
* **No motion** -- press *Play* in CoppeliaSim and confirm the bridge
  is logging `Subscribed to LINDA channel`.
* **Bridge cannot connect to Redis** -- check `docker ps`, or pass
  `--redis-host` / `--redis-port`.

## Reproducibility

The Python bridge is deterministic given a fixed scene; battery decay
and detection radius are pure-Python constants.  CoppeliaSim itself uses
a fixed-step physics solver, so the same `scenario.yaml` produces nearly
identical rescue traces across runs (modulo solver-internal scheduling).
The DALI2 server records every event in past memory and exposes it via
`/api/past?agent=AGENT`, allowing offline replay and inspection.

## Citation

If you use this case study in academic work, please cite the
accompanying CARLA paper.
