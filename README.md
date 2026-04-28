# DALI2 Multi-Robot Search-and-Rescue

Three simulated rescue robots cooperate to find victims in an arena and
bring them to a safe zone. Their **brains** are
[DALI2](https://github.com/AAAI-DISIM-UnivAQ/DALI2) agents (logic
programming, SWI-Prolog). Their **bodies** live inside CoppeliaSim
Edu. A small Python program glues the two together.

> **Quick start:** open CoppeliaSim with a new scene, then run
> `python launch.py` — it starts Redis, builds the scene, launches
> DALI2 and the bridge in one shot.  See section 3 for details.

---

## 1. What you will see when it works

After everything is started:

1. In **CoppeliaSim** three Pioneer P3DX robots start driving around
   an arena with **obstacles** (rocks, crates, debris) and coloured
   cubes representing victims.
2. Each robot has an on-board **vision sensor** (camera). Every few
   seconds the bridge captures a screenshot, saves it to
   `screenshots/<robot_name>/`, and sends the image to a **vision
   LLM** (GPT4All local server or any OpenAI-compatible API) for
   analysis.
3. The LLM describes what it sees; if a victim is detected (green
   cube = light, red cube = heavy), the information is forwarded to
   the **coordinator** agent.
4. The coordinator runs a quick auction: the closest available robot
   wins light victims; the two closest robots cooperate to pick up
   heavy victims.
5. Robots drive their victim to the **blue depot**. When they arrive,
   the box turns into a delivered task.
6. If a robot's battery drops below 25%, it heads to the
   **yellow charger** and is excluded from the next auction.

You can watch the agents' beliefs, past events and live logs in your
browser at <http://localhost:8080>.

---

## 2. Prerequisites (one-time install)

| Software | Where to get it | What we use it for |
|---|---|---|
| **CoppeliaSim Edu** ≥ 4.5 | <https://www.coppeliarobotics.com/downloads> | The simulator (already installed: `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\`) |
| **SWI-Prolog** ≥ 9.0 | <https://www.swi-prolog.org/Download.html> | Runs the DALI2 agents |
| **Python** ≥ 3.10 | <https://www.python.org/downloads/> | Runs the bridge and the scene builder |
| **Redis** ≥ 6 | Easiest: Docker Desktop | Message bus between agents |
| **DALI2 source tree** | This is the sibling folder `..\DALI2\` | The agent runtime itself |

Quick check that everything is on the PATH (open a fresh PowerShell):

```powershell
swipl --version
python --version
docker --version    # only if you'll use Docker for Redis
```

Install the Python dependencies once:

```powershell
cd D:\Repository\DALI2-Robot-Coordination
python -m pip install -r bridge\requirements.txt
```

---

## 3. Run the demo

### Option A — Single command (recommended)

1. Open **CoppeliaSim Edu** → **File → New Scene**.
2. In a single terminal:

```powershell
cd D:\Repository\DALI2-Robot-Coordination
python launch.py
```

This starts Redis, builds the scene (with obstacles + vision sensors),
launches DALI2, and starts the bridge — all in one go.  Press
`Ctrl+C` to stop everything.

Useful flags:

```powershell
python launch.py --skip-redis          # Redis already running
python launch.py --skip-scene          # scene already built
python launch.py --config scene\scenario.yaml
python launch.py --pioneer-model "C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\models\robots\mobile\pioneer p3dx.ttm"
```

### Option B — Step by step (four terminals)

<details>
<summary>Click to expand</summary>

#### Terminal 1 — Redis

```powershell
docker run -d --name dali2-redis -p 6379:6379 redis:7-alpine
```

(If already started once: `docker start dali2-redis`.)

#### Terminal 2 — CoppeliaSim

1. Launch CoppeliaSim Edu → File → New Scene.
2. Don't press Play.

#### Terminal 3 — Build scene + start DALI2

```powershell
cd D:\Repository\DALI2-Robot-Coordination
python scene\build_scene.py
swipl -l ..\DALI2\src\server.pl -g main -- 8080 agents\rescue.pl
```

#### Terminal 4 — Bridge

```powershell
cd D:\Repository\DALI2-Robot-Coordination
python bridge\coppelia_bridge.py
```

</details>

> ✅ **Verify:** open <http://localhost:8080> — five agents listed;
> in CoppeliaSim the robots are driving around obstacles and victims.

---

## 4. Reading what's happening

Three places to look:

| Where | What you see |
|---|---|
| **CoppeliaSim window** | Robots driving, picking up cubes, returning to the blue depot. |
| **Terminal 4 (DALI2)** | Agent log lines: `Spotted victim_3`, `Awarding ...`, `Assigned: ...`, `Lift command received for ...`, `Delivered ...`. |
| **Terminal 5 (bridge)** | `CMD <- rescuer_1 : set_target [...]`, `Attached victim_3 to rescuer_1`, `Delivered victim_3 at depot via rescuer_1`. |

Web UI at <http://localhost:8080>:

- Click an agent to see its **beliefs** (current state),
- the **past events** tab shows everything that ever happened, with
  timestamps,
- the **logs** tab is a live stream.

---

## 5. Stopping everything

If you used `launch.py`: just press `Ctrl+C` — it stops everything.

If you used separate terminals, stop in reverse order:

1. `Ctrl+C` in bridge terminal.
2. `Ctrl+C` in DALI2 terminal.
3. Stop the simulation in CoppeliaSim (⏹), then close the window.
4. `docker stop dali2-redis` (or just close the Redis terminal).

---

## 6. Customising the scenario

Edit `scene\scenario.yaml` to change victim positions/weights:

```yaml
victims:
  - { id: victim_1, x:  4.0, y:  3.0, weight: light }
  - { id: victim_2, x: -3.5, y:  2.5, weight: heavy }
  ...
```

Then re-run the **scene builder and the bridge** with `--config`:

```powershell
python scene\build_scene.py        --config scene\scenario.yaml
python bridge\coppelia_bridge.py   --config scene\scenario.yaml
```

(You don't need to restart Redis or DALI2 for scenario changes.)

---

## 7. Vision system

Each robot has a **CoppeliaSim vision sensor** (camera) mounted on
top. The bridge captures screenshots every 5 seconds and saves them to
`screenshots/<robot>/imageN.jpg`.

The screenshot is sent as an event to the DALI2 agent (`screenshotE`)
and, if a vision LLM is reachable, the image is also analysed
automatically. The LLM responds with a Prolog fact like
`victim_detected(light)` or `obstacle_ahead`, which the agent uses to
inform the coordinator.

### Local vision LLM (GPT4All)

1. Install [GPT4All](https://gpt4all.io/) and enable the local
   server (default: `http://localhost:4891`).
2. Load a vision-capable model (e.g. LLaVA).
3. The bridge and DALI2 will automatically use it.

Environment variables for customisation:

| Variable | Default | Description |
|---|---|---|
| `VISION_LLM_ENDPOINT` | `http://localhost:4891/v1/chat/completions` | OpenAI-compatible API |
| `VISION_LLM_MODEL` | (empty — server picks) | Model name |
| `VISION_LLM_ENABLED` | `true` | Set to `false` to disable |

### Remote LLM (OpenRouter)

DALI2 also ships with an **AI Oracle** for text-based triage via
[OpenRouter](https://openrouter.ai):

```powershell
$env:OPENROUTER_API_KEY = "sk-or-..."
python launch.py
```

Without a key the system runs in pure-symbolic mode.

---

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| `Object '/rescuer_1' not found` (in bridge log) | The scene builder didn't run, or you re-opened CoppeliaSim. Re-run `python scene\build_scene.py`. |
| `Could not load Pioneer model` | Use the absolute path: `python scene\build_scene.py --pioneer-model "C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\models\robots\mobile\pioneer p3dx.ttm"` |
| `Connection refused` from the bridge to CoppeliaSim | CoppeliaSim isn't open, or it's still starting. Make sure the window is up before launching the bridge. |
| `redis.exceptions.ConnectionError` | Redis isn't running. Re-check Terminal 1. |
| `swipl: command not found` | Install SWI-Prolog and re-open the terminal so the new PATH is picked up. |
| Robots loaded but they don't move | The simulation is paused. Press ▶ in CoppeliaSim, or just restart the bridge — it auto-starts the sim. |
| `Loading agents from: agents\rescue.pl` then nothing | Wait ~2 seconds; agents start asynchronously. If still nothing, check Redis is reachable. |
| Web UI shows agents but no logs | Refresh the page; or check that DALI2 isn't blocked by the Windows firewall on port 8080. |

---

## 9. Repository layout

```
DALI2-Robot-Coordination\
+- agents\
|   `- rescue.pl              # All four DALI2 agents in one file
+- bridge\
|   +- coppelia_bridge.py     # Redis <-> CoppeliaSim ZMQ + vision
|   `- requirements.txt
+- scene\
|   +- build_scene.py         # Scene setup (robots, obstacles, victims)
|   `- scenario.yaml          # Victim positions / weights
+- screenshots\               # Auto-created: per-robot camera images
+- launch.py                  # Single-command launcher
+- docker-compose.yml         # Optional: `docker compose up -d`
+- README.md                  # You are here
```

---

## 10. How it works (architecture)

```
   +----------------+   Redis pub/sub   +------------------+   ZMQ Remote API   +-------------+
   | DALI2 agents   | <===============> |  Python bridge   | <================> | CoppeliaSim |
   | (SWI-Prolog)   |   LINDA channel   |  (10 Hz control) |                    | digital     |
   |                |                   |                  |                    | twin        |
   |  coordinator   |                   |  go-to-goal      |                    |             |
   |  rescuer_1..3  |                   |  battery model   |                    |  Pioneer +  |
   |  monitor       |                   |  victim sensor   |                    |  victims +  |
   |                |                   |  vision capture  |                    |  obstacles  |
   +----------------+                   +--   +-------+  -+                    +-------------+
                                             | Vision |
                                             | LLM    | (GPT4All / OpenAI-compatible)
                                             +--------+
```

The bridge registers itself as the (virtual) DALI2 agent named
`sim`. From the agents' perspective, the simulator is just another
peer:

- the rescuer says `send(sim, set_target(Me, X, Y))`,
- the bridge receives it and drives the wheels,
- when the wheels arrive, the bridge publishes `at_target` back on
  the LINDA channel addressed to that rescuer.

That means swapping the simulator for real robots later is a
drop-in: any process that follows the same message protocol becomes
the new `sim` agent.

---

## 11. Linux / macOS

The instructions are identical, except:

- shell paths use `/` instead of `\`,
- you can run Redis natively (`redis-server` in a terminal, or
  `brew services start redis` on macOS),
- CoppeliaSim Edu typically lives in `~/CoppeliaSim/`, so:
  ```bash
  python scene/build_scene.py \
      --pioneer-model "$HOME/CoppeliaSim/models/robots/mobile/pioneer p3dx.ttm"
  ```

---

## 12. Citing

If you use this case study in academic work, please cite the
accompanying CARLA paper.
