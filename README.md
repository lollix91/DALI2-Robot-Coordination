# DALI2 Multi-Robot Search-and-Rescue

Three simulated rescue robots cooperate to find victims in an arena and
bring them to a safe zone. Their **brains** are
[DALI2](https://github.com/AAAI-DISIM-UnivAQ/DALI2) agents (logic
programming, SWI-Prolog). Their **bodies** live inside CoppeliaSim
Edu. A small Python program glues the two together.

> **You will run 4 things in parallel:** Redis, CoppeliaSim, the DALI2
> server, and the Python bridge. Don't worry, every step has its own
> terminal and a copy-paste command below.

---

## 1. What you will see when it works

After everything is started:

1. In **CoppeliaSim** three Pioneer P3DX robots start driving around
   the arena.
2. When one of them gets close enough to a coloured cube (a
   "victim"), it tells the **coordinator** agent.
3. The coordinator runs a quick auction: the closest available robot
   wins green ("light") victims; the two closest robots cooperate to
   pick up red ("heavy") victims.
4. Robots drive their victim to the **blue depot**. When they arrive,
   the box turns into a delivered task.
5. If a robot's battery drops below 25%, it heads to the
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

## 3. Run the demo, step by step

You'll need **four terminals**. Open them all up front, in this order.
The instructions below assume your repos live at `D:\Repository\` (as
they currently do); change the paths if yours differ.

### Terminal 1 — Redis

Run **once**, then leave it alone.

```powershell
docker run -d --name dali2-redis -p 6379:6379 redis:7-alpine
```

(If you already started it once, next time just do
`docker start dali2-redis`.)

No Docker? Download `redis-server.exe` (e.g. from
<https://github.com/microsoftarchive/redis/releases>) and run
`redis-server.exe` in this terminal — it must stay open.

> ✅ **Verify:** `docker ps` shows `dali2-redis` running on port `6379`.

### Terminal 2 — CoppeliaSim

1. Launch **CoppeliaSim Edu** (Start menu, or
   `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\coppeliaSim.exe`).
2. **File → New Scene** so you start from an empty arena.
3. Leave the simulator open. Don't press Play yet — the scene builder
   in the next step will populate the empty scene.

> ✅ **Verify:** the CoppeliaSim window is open with an empty floor and
> the toolbar shows ▶ Stop (i.e. the simulation is **not** running).

### Terminal 3 — Build the scene

This Python script connects to the running CoppeliaSim from outside
and adds three Pioneer P3DX robots, five victim cubes, the depot and
the charger.

```powershell
cd D:\Repository\DALI2-Robot-Coordination
python scene\build_scene.py
```

You should see log lines like:

```
[scene] Loaded Pioneer P3DX as /rescuer_1 at (-1.50, -2.50)
[scene] Loaded Pioneer P3DX as /rescuer_2 at  (0.00, -2.50)
[scene] Loaded Pioneer P3DX as /rescuer_3 at  (1.50, -2.50)
[scene] Placed victim /victim_1 (light) at (4.00, 3.00)
...
[scene] Scene built. ...
```

> ✅ **Verify:** the CoppeliaSim window now shows three robots, five
> coloured cubes, a blue disc (depot) and a yellow disc (charger).

If you get a "Pioneer model not found" error, run:

```powershell
python scene\build_scene.py --pioneer-model "C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\models\robots\mobile\pioneer p3dx.ttm"
```

### Terminal 4 — DALI2 agents

Start the DALI2 server, telling it to load `agents\rescue.pl`:

```powershell
cd D:\Repository\DALI2-Robot-Coordination
swipl -l ..\DALI2\src\server.pl -g main -- 8080 agents\rescue.pl
```

You should see:

```
=== DALI2 Multi-Agent System ===
Node: node-8080 | Port: 8080
Loading agents from: agents\rescue.pl
Agents defined: [coordinator,rescuer_1,rescuer_2,rescuer_3,monitor]
Server started, launching agent processes...
```

> ✅ **Verify:** open <http://localhost:8080> in your browser. You
> should see five agents listed.

### Terminal 5 — The bridge

Open a **fifth** terminal and start the bridge. It will:

- subscribe to Redis for messages addressed to the (virtual) agent `sim`,
- start the CoppeliaSim simulation if it is stopped,
- drive the robots and stream their sensors back to the agents.

```powershell
cd D:\Repository\DALI2-Robot-Coordination
python bridge\coppelia_bridge.py
```

You should see:

```
[bridge] Robot rescuer_1 ready (handle=...)
[bridge] Robot rescuer_2 ready (handle=...)
[bridge] Robot rescuer_3 ready (handle=...)
[bridge] Victim victim_1 (light) ready (handle=...)
...
[bridge] Subscribed to LINDA channel
[bridge] Simulation started
[bridge] Bridge running -- ctrl+C to stop
```

> ✅ **Verify:** in CoppeliaSim the simulation is now playing (▶ icon
> changed to ⏸), and the three robots start driving.

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

In reverse order is cleanest:

1. `Ctrl+C` in Terminal 5 (bridge).
2. `Ctrl+C` in Terminal 4 (DALI2).
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

## 7. Optional: LLM-driven triage

DALI2 ships with an **AI Oracle** that can call any LLM through
[OpenRouter](https://openrouter.ai). When a key is set, the
coordinator's `victim_seenE` rule asks the model for a triage
suggestion (advice only — the deterministic auction still decides who
goes where):

```powershell
$env:OPENROUTER_API_KEY = "sk-or-..."
swipl -l ..\DALI2\src\server.pl -g main -- 8080 agents\rescue.pl
```

Without the key the system runs in pure-symbolic mode, which is the
default.

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
|   +- coppelia_bridge.py     # Redis  <->  CoppeliaSim ZMQ
|   `- requirements.txt
+- scene\
|   +- build_scene.py         # Programmatic scene setup
|   `- scenario.yaml          # Victim positions / weights
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
   |  monitor       |                   |  victim sensor   |                    |  victims    |
   +----------------+                   +------------------+                    +-------------+
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
