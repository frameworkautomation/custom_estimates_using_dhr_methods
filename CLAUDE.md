# Project Context for Claude

## Repo cloning setup

There is a `cloning_stuff/` folder at the project root that manages external repo dependencies.

### Files

- **`cloning_stuff/repos.txt`** — one SSH URL per line (e.g. `git@github.com:user/repo.git`). Lines starting with `#` are ignored. The user fills this in manually.
- **`cloning_stuff/make_clones.sh`** — reads `repos.txt` and clones each repo into `clones/`. Skips repos already cloned. Prints errors and continues if a clone fails (e.g. SSH key not set up yet).
- **`cloning_stuff/update_clones.sh`** — iterates every repo in `clones/`, pulls from `origin main`, and appends a timestamped log to `cloning_stuff/update_clones.log`. Prints errors per repo and continues.
- **`cloning_stuff/update_clones.log`** — tracked by git so the user can see a history of when external assets were updated.

### Directory layout

```
.
├── .gitignore                  # ignores clones/ and steps_from_SolidWorks/
├── CLAUDE.md                   # this file
├── README.md
├── clones/                     # cloned repos land here (git-ignored)
├── steps_from_SolidWorks/      # derived STEP exports (git-ignored, see below)
├── using_rhino_to_convert_solid_works/
│   ├── rhino_convert_to_step.py    # Rhino Python script that does the conversion
│   └── run_rhino_convert.bat       # double-click on Windows to run the conversion
└── cloning_stuff/
    ├── repos.txt
    ├── make_clones.sh
    ├── update_clones.sh
    └── update_clones.log
```

### Notes
- `clones/` is git-ignored — never commit it.
- SSH keys need to be configured on the machine for `make_clones.sh` and `update_clones.sh` to succeed. Until then, both scripts will print the SSH errors and continue rather than exiting immediately.
- Both scripts use `set -uo pipefail` (not `-e`) so a single failure doesn't abort the whole run.

## STEP file conversion

SolidWorks assemblies (`.SLDASM`) and parts (`.SLDPRT`) in `clones/` are converted to STEP using Rhino. The output lives in `steps_from_SolidWorks/`, which mirrors the folder structure of `clones/` exactly:

```
clones/<repo>/<subpath>/file.SLDASM
  ->
steps_from_SolidWorks/<repo>/<subpath>/file.step
```

`steps_from_SolidWorks/` is git-ignored — these are derived files, not source.

### If a STEP file is missing

If you need a STEP file and it doesn't exist in `steps_from_SolidWorks/`, the user needs to run the conversion. Tell them to:

1. Make sure the source `.SLDASM` exists under `clones/` (run `cloning_stuff/make_clones.sh` if needed).
2. On their Windows machine, double-click `using_rhino_to_convert_solid_works\run_rhino_convert.bat`. Rhino must be installed (version 7 or 8).
3. The STEP file will appear at the mirrored path under `steps_from_SolidWorks/`.

Do not attempt to read or use `.SLDASM` or `.SLDPRT` files directly — always use the corresponding `.step` file from `steps_from_SolidWorks/`.

## RoboDK station setup

The saved RoboDK station lives at `robo_dk_saves/TestStationFanuc.rdk` (git-ignored). It contains the Fanuc R-2000iC 125L robot and the factory cell geometry already positioned.

### Direct API access (preferred for live queries)

RoboDK exposes a TCP/IP API on `localhost:20500`. While RoboDK is running, any Python process on the same machine can connect directly via `Robolink()` — no need to copy scripts into `C:\RoboDK\Scripts\`, no need to run them through Tools > Run Script, and no need to round-trip through JSON files in `robo_dk_output/`.

Minimal connect:

```python
import sys
sys.path.append("C:/RoboDK/Python")
from robodk.robolink import Robolink
from robodk.robomath import *

RDK = Robolink()  # connects to running RoboDK on localhost:20500
robot = RDK.Item("Fanuc R2000iC 125L")
print(robot.Name())
```

Run with `python script.py` from a normal terminal. You can read items, move the robot, solve IK, etc. — everything the API supports.

Use this pattern for one-off queries and interactive work. The caller-script + file-I/O pattern below is still used for things that need to persist across RoboDK sessions (since the station resets without a paid license to save) or that need to run inside RoboDK for other reasons.

### Caller script (one-time setup per machine)

`robodk_setup/setup_station_caller.py` is meant to be copied once to `C:\RoboDK\Scripts\` and left there. It connects to a running RoboDK instance and loads `robo_dk_saves/TestStationFanuc.rdk` into the station.

To use it:
1. Copy `robodk_setup/setup_station_caller.py` to `C:\RoboDK\Scripts\`.
2. Open RoboDK.
3. Run it via: Tools > Run Script > setup_station_caller.

The caller script must not be moved into the RoboDK Scripts folder permanently from the repo — it lives in `robodk_setup/` in the repo and is copied manually. This avoids committing anything into RoboDK's install directory.

`robodk_setup/setup_station.py` is the main setup script. It loads the station, records cone positions, and deletes cones. It does NOT save the station (see license note below).

`robodk_setup/modifications.py` contains the functions called by setup_station.py:
- `record_cone_positions(RDK)` — writes cone world poses to `robo_dk_output/cone_positions.json` before deletion
- `delete_cones(RDK)` — deletes all `Machine<N>YarnTray<N>Slot<N>(Base)?` items

`robodk_setup/list_items.py` — standalone diagnostic that writes all station item names/types to `robo_dk_output/station_items.txt`. Run it manually if you need to inspect item names.

### License limitation — saving disabled

`TestStationFanuc.rdk` contains two robots: a Fanuc R-2000iC 125L and a linear rail mechanism. RoboDK's free license allows *loading* multi-robot stations but not *saving* them via the API — `RDK.Save()` silently produces an empty ~1KB stub file instead of the real station.

**Do not attempt to save the station via script until a paid RoboDK license is in place.**

### Progress tracking — steps.json

Because saving is disabled, session progress is tracked in `robo_dk_output/steps.json`:

```json
{
  "station_loaded": true,
  "cone_positions_recorded": true,
  "cones_deleted": true,
  "last_updated": "2026-04-30 15:17:16"
}
```

Each time the caller runs, setup_station.py reads this file and skips steps that are already done. Cone deletion is also verified live by querying the station (since the station resets to its original state each RoboDK session). To force a full re-run, delete `steps.json`.

## IK solver architecture (current state)

### Base cones (`base_cone_grab_*`)

`robodk_code/moving_a_cone.py` and `robodk_code/check_base_cone_reachability.py` both use
**RoboDK OptimAxes (Algorithm 3, damped least squares)** — mirroring DHR's
`OptimizationKinematicsModel`. The custom LM solver has been removed from these files.

Config block (same in both files):
```python
OPT_AXES_STATIC_J7 = {
    "AbsJnt_7": 0,  "AbsOn_7": 1, "AbsW_7": 100,   # j7 strongly constrained
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    "RelOn_1..7": 1, "RelW_1..7": 50,               # allow motion on all joints
}
```

Pattern: `robot.setParam("OptimAxes", props)` → `robot.setJoints(HOME_SEED)` → `robot.MoveJ(pose)` → `robot.Joints()`.

Robot reference frame **must be WorldFrame** before any MoveJ with a Cartesian pose
(PoseAbs() returns world-space coords; if the robot's frame is anything else the
target is misinterpreted). Both scripts call `robot.setPoseFrame(world_frame)` before
the solve loop and restore it in a `finally` block.

IK solutions are saved to `ik_solutions/` (gitignored) as timestamped JSON by
`check_base_cone_reachability.py`. `moving_a_cone.py` loads the most recent
`base_cone_ik_*.json` on startup and skips recomputing if all cones are present.

### Destination cones (`cone_grab_*`, under Cones > Cone_N)

`moving_a_cone.py::compute_dest_ik()` uses **RoboDK's built-in `robot.SolveIK(pose)`**.
This works because destination cones are comfortably within 6-DOF reach. j7 is
not constrained — the analytic solver chooses it freely. 11/14 cones solve; 3
(cone_grab_6, 12, 13) fail and are excluded from the selectable list.

`SolveIK` returns a `Mat`, not a list. Use `Mat.list()` to get a flat joint list
(`len(Mat)` gives rows=1, not joint count — common gotcha).

### Custom LM solver (`test_reach_base_cone.py::custom_ik_pos_and_zaxis`)

Still exists in `robodk_code/test_reach_base_cone.py` and is imported by
`robodk_code/move_to_base_cone_grab_with_setable_accuracy.py` (older script).
`moving_a_cone.py` and `check_base_cone_reachability.py` no longer use it for IK
(they still import `fmt_joints` from that file).

## ── Waypoint web viewer (planned) ────────────────────────────────────────────

Interactive 3D GUI for viewing and editing waypoints and edges.
Design doc and implementation plan: `waypoint_viewer/`

**Stack:** FastAPI (Python) backend + React + react-three-fiber frontend.
**Features:** clickable waypoint spheres, orientation arrows, edge lines coloured
by tested status, click two waypoints → create bidirectional edge written back to
`all_waypoints.yaml`, orbit + pan camera controls, edge/waypoint detail panel,
delete edge, filter by source/move_type.

Run: `uvicorn robodk_code.waypoint_server:app` (WSL) → open browser on Windows.

## ── TODO (low priority) ───────────────────────────────────────────────────────

- **[LOW] Move GhPython scripts** — currently in `ai_generated_grasshopper_python/`.
  Move to a cleaner location once workflow stabilises. Don't move until explicitly requested.

- **[LOW] `save_joint_position.py` — series variant** — after single-point tool is tested,
  add a variant that accepts N waypoint names in sequence, lets user jog to each in RoboDK,
  and automatically creates bidirectional edges between consecutive points in `path_config.yaml`.
  Same `source: human` tag.

## Sub-optimal / known limitations (not blocking, but worth fixing later)

### `check_base_cone_reachability.py` approach-pose coupling

The reachability checker solves IK for both the grab pose and the approach pose
(200mm offset along grab Z-axis) in the same script. Approach pose generation is
coupled to the IK solve, so you can't cheaply compute approach poses from saved
grab-pose solutions without re-running the full solver.

### `move_to_base_cone_grab_with_setable_accuracy.py` moves robot visibly during solve

The custom LM solver calls `robot.setJoints(...)` on every iteration, causing
the RoboDK GUI to redraw on every step. Wrap with `RDK.Render(False)` /
`RDK.Render(True)` to suppress this (future work).

## Known issues / future work

### [BLOCKING] Cones and bins do not move with the robot in RoboDK after GH import

After `export_base_cone_waypoints.py` imports STLs via `RDK.AddFile()`, the objects
land at world level with no parent. In the original station they were parented to a
frame that moves with the rail, so they visually tracked the robot.
Need to determine: which frame should base_cone_N and bin_N be parented to after import?
Check the original station tree structure — likely BaseCones frame under the robot base.
Fix: after `RDK.AddFile()`, call `item.setParent(parent_frame)` with the correct frame.

### [BLOCKING] export_base_cone_waypoints.py GH script not running correctly

Still misbehaving after recent fixes. Needs further investigation when user reports
specific error output from the GH print console.

### ~~[EXTREMELY LOW PRIORITY] Bins imported into RoboDK are blue instead of grey~~ FIXED

### IK visible motion issue

See sub-optimal note above re: `move_to_base_cone_grab_with_setable_accuracy.py`.

## ── COLLISION PLANNING: WITH OR WITHOUT CONE ATTACHED ───────────────────────

**Recommendation: plan with a cone attached at all times (worst-case geometry).**

### How RoboDK MoveJ_Test handles attached objects

`robot.MoveJ_Test(from_joints, to_joints)` includes all objects that are children
of the robot or its tool in collision checking. In `check_collision_free_paths.py`,
`cone_mesh.setParentStatic(tool)` is called before the test loop so the cone is
part of the swept volume for every `MoveJ_Test` call, then detached after. This is
already correctly implemented.

### When is the cone held?

| Phase | Cone held? |
|---|---|
| home → base_approach → base_grab | No |
| base_grab → dest_grab (outbound) | Yes |
| dest_grab → home (return) | No |

### One edge set or two?

**One set (with cone) is sufficient.** An edge that is collision-free with the cone
is guaranteed to be collision-free without it. The only risk is unnecessary detours
on the return path (empty gripper) due to conservative edge exclusions — monitor in
practice but don't pre-optimise.

### Approach moves (entering base tray — cone "attached" during testing)

The cone geometry is present during the tray approach test. Risk of false-positive
collision with adjacent filled slots. If this happens, add `collision_disable` pairs
in `path_config.yaml`:
```yaml
collision_disable:
  - ["base_cone_0", "BaseTray"]
```

### Largest cone = worst case

Set `cone_mesh_template` in `path_config.yaml` to the geometrically largest cone
(widest / tallest). If the template item is missing from the station,
`check_collision_free_paths.py` warns and falls back to no-cone testing — not safe
for production.

### Dynamic attach vs. baking into tool geometry

Keep the current dynamic attach approach (`setParentStatic` before/after test loop).
Baking the cone permanently into the tool geometry pollutes the visual model and
removes flexibility for multiple cone sizes.

## ── FUTURE: Automated obstacle avoidance ─────────────────────────────────────

**Current approach (manual waypoints):** A human places intermediate frames in RoboDK
(e.g. curtain-safe poses per machine zone). `check_collision_free_paths.py` validates
edges between them. If an edge collides, the human adds more waypoints and re-runs.
This is how DHR's knitwear-cell works too.

**Why automated is hard:** A path-planning algorithm (RRT, PRM, OMPL) requires:
- A configuration-space obstacle model (not just mesh collision — RoboDK's `MoveJ_Test`
  only checks a straight interpolation)
- A sampler that understands the 7-DOF joint space including the rail
- RoboDK does not expose an internal planner. ROS MoveIt could do this but requires
  a full ROS integration and a robot driver — significant work.

**What "automated" could look like in this project:**

Option A — **Dense-graph search over pre-sampled configs:**
Pre-sample N random collision-free configurations per machine zone (offline, using
`MoveJ_Test`). Build a full Dijkstra/PRM graph. At execution time, query the graph
for a path. No human waypoint placement needed. Downside: N must be large enough
to guarantee connectivity; pre-sampling is slow.

Option B — **Guided RRT in joint space using RoboDK collision checking:**
Implement an RRT in Python. Steer between random samples, check each extension
with `MoveJ_Test`. Works within the existing RoboDK setup. Slow per-query but
no pre-computation. Could be cached in `path_plan.yaml` the same way edges are now.

Option C — **Optimisation-frame style (DHR approach, extended):**
Define one `OptimizationApproachMachine_N` frame per machine zone. Use
`OptimizationKinematicsModel` (`robot.setParam("OptimAxes", ...)`) to bias j7
to the machine X-position while solving arm IK for the curtain-safe frame. This
doesn't avoid obstacles automatically — it still needs human-placed curtain-safe
frames — but it removes the need to manually specify j7 per waypoint.

**Recommendation when ready:** Option B (guided RRT) is the most self-contained path
forward. It reuses `MoveJ_Test` as the collision oracle and requires no external
libraries. Implement as a standalone `plan_rrt.py` that writes edges to `path_plan.yaml`
in the same format as the manual planner. Start with a fixed step size of ~5 degrees
per joint per step.

**Not doing this now.** The current manual approach is sufficient for the static cell
and 1-2 machine validation scope.

## ── DHR's code generation pipeline (we have it) ──────────────────────────────

All generator code IS in the cloned repo. Full pipeline:

```
robodk.yaml  →  generate_states.sh  →  generated_states.py
```

**Files:**
- `clones/knitwear-cell/src/main/xquery/yaml_to_state_class.xq` — the XQuery generator
- `clones/knitwear-cell/generate_states.sh` — shell runner
- `clones/knitwear-cell/libs/saxon-he-12.8.jar` — Saxon HE XQuery processor (Java)
- `clones/knitwear-cell/libs/xmlresolver-4.5.0.jar` — dependency

**Run it:**
```bash
cd clones/knitwear-cell
sh generate_states.sh                         # uses defaults
sh generate_states.sh src/main/config/robodk.yaml src/main/robot/generated_states.py
```

Requires: Java, Saxon HE 12.8 JAR. YAML→JSON step uses `yq` (preferred) or Python PyYAML.

**Inverse — export RoboDK station back to yaml:**
- `clones/knitwear-cell/src/main/utils/generate_yaml.py`
- `clones/knitwear-cell/generate_yaml.sh`
- `clones/knitwear-cell/src/main/robodk/station_yaml_generator.py`

**How the generator works:**
1. `robodk.yaml` defines every frame with a `states:` block:
   ```yaml
   - name: ApproachBuffer
     type: ITEM_TYPE_FRAME
     pose: {x: 0.0, y: 200.0, z: 870.0, ...}
     states:
       - move: J
         optimization: true
         tool_name: GrabbingGripper
         optimization_frame: RobotPedestal
         optimization_axis: X
   ```
2. XQuery reads each frame's `states:` entries and emits one Python class per entry,
   selecting `OptimizationKinematicsModel` when `optimization: true`, `MoveJModel`
   when `move: J`, `MoveLModel` when `move: L`.
3. Output is `generated_states.py` — imported at runtime, no YAML parsing at all.

**Our setup (2026-05-20):** We use DHR's XQuery as-is. The header now imports from
our modules (`from dhr_robot import *`, `from state import State`). We have ported
`state.py`, `dhr_robot.py`, and `state_machine.py` into `robodk_code/`. See the
`## ── DHR runtime port` section above for how to generate and use state classes.

## ── INVESTIGATE: Runtime collision checking vs offline pre-computation ─────────

**Our approach:** Edges are tested offline in bulk by `check_collision_free_paths.py`,
results baked into `path_plan.yaml`. At runtime `moving_a_cone.py` trusts the yaml
and never calls `MoveJ_Test`.

**DHR approach:** `MoveJTestModel` runs `MoveJ_Test` live at execution time, with Redis
caching to skip re-testing identical transitions in the same session.

**Safety question to investigate:** Is DHR's approach strictly safer?

Arguments FOR DHR being safer:
- If something physically changes in the cell at runtime (fixture moved, person in the
  way), live `MoveJ_Test` would detect a new collision that our pre-computed plan misses.
- Our plan assumes the environment is identical to when `check_collision_free_paths.py`
  ran. Any physical change silently invalidates the plan.

Arguments AGAINST (or for our approach being equivalent):
- DHR's `configuration.yaml` has `collision_checking: false` — their live collision
  checking is **disabled in production** (this is the value in their committed config).
  If that flag gates `MoveJTestModel`, they're not actually checking at runtime either.
  Investigate: how does `collision_checking` control `MoveJTestModel` execution?
- For a fully static, guarded cell with no human access during operation, offline
  pre-computation is sufficient and has zero runtime overhead.
- Live `MoveJ_Test` is slow (must run a simulated trajectory in RoboDK) — DHR speeds
  up simulation to 500x and restores it after. This adds latency to every move.

**What to check before going to production:**
1. Confirm whether DHR's `collision_checking: false` actually disables `MoveJTestModel`
   — read `collision_item_presenter_service.py` and check how the flag is propagated.
2. Decide: for our cell, do we want to add a runtime `MoveJ_Test` guard in
   `moving_a_cone.py` as a safety net, even if the pre-computed plan should be valid?
   Cost: latency. Benefit: catches environment changes and configuration bugs.

## ── DHR runtime port (2026-05-20) ────────────────────────────────────────────

Three new files in `robodk_code/`:

| File | Purpose |
|------|---------|
| `state.py` | Stripped `State` Pydantic base class (no @autowired / Redis / gRPC) |
| `dhr_robot.py` | `Robot` mixin pipeline + all kinematics/movement models; `setup(rdk, robot_item)` wires in live RoboDK refs |
| `state_machine.py` | `StateMachine` — getattr-dispatch on a states container |

XQuery header updated: `clones/knitwear-cell/src/main/xquery/yaml_to_state_class.xq`
now emits `from dhr_robot import *` and `from state import State` (was DHR src.main paths).

**Generate state classes from path_config.yaml:**
```bash
# 1. YAML → JSON
python -c "import yaml,json; open('/tmp/pc.json','w').write(json.dumps(yaml.safe_load(open('robo_dk_output/path_config.yaml'))))"

# 2. Run XQuery (Java + Saxon HE required)
java -cp clones/knitwear-cell/libs/saxon-he-12.8.jar:clones/knitwear-cell/libs/xmlresolver-4.5.0.jar \
     net.sf.saxon.Query \
     -q:clones/knitwear-cell/src/main/xquery/yaml_to_state_class.xq \
     json=/tmp/pc.json \
     -o:robodk_code/generated_states.py
```

Output: `robodk_code/generated_states.py` with a `StatesContainer` Pydantic model.

**Add frames to path_config.yaml** (bottom of that file has a commented example):
```yaml
frames:
  - name: CurtainSafeMachine1
    states:
      - move: J
        optimization: true
        tool_name: pickup_closed
        optimization_axis: X
```

**Use StateMachine at runtime:**
```python
import dhr_robot
from state_machine import StateMachine
from generated_states import StatesContainer

dhr_robot.setup(rdk, robot_item)
sm = StateMachine(StatesContainer())
sm.set_state("curtain_safe_machine_1_1")  # snake_case frame name + _N index
ok = sm.handle()
```

Field name convention (XQuery): PascalCase frame → snake_case + `_N` index.
`CurtainSafeMachine1` with 1 state → attribute `curtain_safe_machine_1_1`.

## ── DHR edge/transition architecture (2026-05-21) ───────────────────────────

**DHR has no explicit edge or graph structure.** Transitions between states are
hardcoded as sequential `yield from execute_state(trigger=...)` calls in
`clones/knitwear-cell/src/main/action/move_task.py`.

Key observations from reading move_task.py:
- Line 29: always goes to `transport` first (arm folds to safe pose)
- Line 33/37: `move_on_rail_optimization_approach_{type}_{id}` — explicit rail-slide
  state generated from the `OptimizationApproachMachine{N}` frames in YAML.
  These frames ARE used — not as TCP targets but as dedicated j7-positioning moves
  before the arm reaches into the machine zone.
- Lines 184–203: `approach_machine_{id}_curtain_safe_1` — enter machine zone (MoveJ)
- Lines 211–218: `approach_machine_{id}_curtain_safe_2` — exit machine zone (MoveL)
  Note: _1 = MoveJ (approach), _2 = MoveL (precise approach/retract). Same frame,
  different move type — two separate State classes generated from the same YAML states array.
- Lines 248–255: buffer entry/exit is 3 hardcoded waypoints in sequence
- Lines 79–81 / 150–152: if grab/place fails, `_reverse_path()` retraces all moves
  in reverse order to recover

**Edges are monodirectional in DHR's implicit graph.** Enter and exit use different
state indices (_1 vs _2 = MoveJ vs MoveL) and different arm configs. A→B is not
the same move as B→A. Our `path_plan.yaml` should reflect this:
- TODO: edges in path_plan.yaml should be directional (A→B tested separately from B→A)
- Currently our Dijkstra graph treats edges as bidirectional which may be incorrect

**move_task.py full path:** `clones/knitwear-cell/src/main/action/move_task.py`

## ── WAYPOINT GRAPH BUILD PLAN (2026-05-21) ───────────────────────────────────

Full pipeline from Grasshopper-generated cone points to tested, visualised
collision graph. Do these phases in order.

### Phase 1 — Waypoint YAMLs from existing Grasshopper data

**A. Base cone waypoints YAML**
- Input: existing Grasshopper script that already generates base cone grab points
  relative to the robot at j7=0 (rail home)
- Task: rewrite that GhPython script so it also exports a YAML to
  `robo_dk_output/base_cone_waypoints.yaml`
- Each entry: name, pose (x,y,z,rx,ry,rz), move_type, j7_weight, note
- Include both grab pose AND approach pose (200mm offset along local Z) as
  separate named entries with an auto-generated edge between them

**B. Machine cone waypoints YAML**
- Input: machine cone targets already in RoboDK station (cone_grab_* targets)
- Task: script (or GhPython) that exports these to
  `robo_dk_output/machine_cone_waypoints.yaml` with same schema
- Same: grab + approach per cone, edge between them

**Waypoint YAML schema:**
```yaml
waypoints:
  - name: base_cone_grab_0
    x: 0.0  y: 0.0  z: 500.0  rx: 0.0  ry: 0.0  rz: 0.0
    frame: robot_local          # robot_local | world
    move_type: MoveL            # MoveJ | MoveL | MoveJ_j7_free | MoveJ_optimization
    j7: 0.0                     # explicit j7 when move_type=MoveJ/MoveL
    optimization_frame: null    # frame name when move_type=MoveJ_optimization

edges:
  - from: base_cone_grab_0_approach
    to:   base_cone_grab_0
    tested: null                # null=untested, true=clear, false=collision
  - from: base_cone_grab_0
    to:   base_cone_grab_0_approach
    tested: null
```

### Phase 2 — Grasshopper visualization of waypoints

**GhPython component: `read_waypoints_as_objects.py`**
- Input: yaml_path, filter (optional regex)
- Outputs: planes, names, move_types, j7_values — parallel lists
- Colour hint output: move_type string so GH can colour-code points by type

### Phase 3 — Manual waypoint addition from Grasshopper

**GhPython component: `add_waypoint.py`**
- Inputs: plane (Rhino Plane), name (str), move_type (str), j7 (float),
  optimization_frame (str), yaml_path
- On trigger: appends entry to the YAML, adds bidirectional edges to nearest
  neighbours if requested
- User places planes in Rhino, wires them in, hits button

**Connections are always stored bidirectionally** — two separate edge entries
A→B and B→A, each with their own `tested` field, because enter and exit
paths may collide differently.

### Phase 4 — Visualize full graph in Grasshopper

**GhPython component: `visualize_graph.py`**
- Reads waypoints YAML
- Outputs: planes (waypoints), lines (edges), colours by status:
  - untested: grey
  - tested clear: green
  - tested collision: red
  - move_type colouring for waypoints

### Phase 5 — Edge testing via RoboDK

**Script: `robodk_code/test_edges.py`**
- Reads waypoints YAML
- For each edge with tested=null: runs MoveJ_Test in RoboDK
- Writes tested=true/false back into the YAML
- Both directions tested independently
- Can be re-run; skips already-tested edges unless --retest flag
- Visualize results by re-running Phase 4 component

### Phase 6 — Integration with moving_a_cone.py

- Replace path_config.yaml + path_plan.yaml with unified waypoints YAML
- moving_a_cone.py reads waypoints YAML directly, uses tested edges for
  Dijkstra routing
- OR: keep existing path_plan system but feed it from the waypoints YAML
- Add bin movement: moving bins from one side of the cell to the other
  (same pick-and-place pattern as cones but for bins)

### Phase 7 — Full cell test with ceiling STL

- Import the ceiling STL (from Robert via Slack) into the RoboDK station as a
  collision object
- Re-run `test_edges.py` with ceiling present to find any edges that now collide
- Verify the full end-to-end cone move still works with the ceiling in place

### Current status
- Phase 1A: DONE — `ai_generated_grasshopper_python/export_base_cone_waypoints.py`
  - Inputs: grab_points, approach_points, string_grab_points, string_approach_points (Planes from GH),
    base_origin (Point), cones/bins (geometry for STL), cone_color/string_color/bin_color, trigger
  - Writes `robo_dk_output/base_cone_waypoints.yaml` + STLs to C:/temp/base_cones/ + imports STLs into RoboDK
  - **TODO: review output in Grasshopper and RoboDK — check plane orientations, YAML values, colors**

- Phase 1B: DONE — `robodk_code/export_machine_cone_waypoints.py` (CLI, reads RoboDK live)
  - **TODO: run with RoboDK open, review machine_cone_waypoints.yaml output**
  - **NOTE: may need to be rewritten as GhPython if machine cone positions come from Grasshopper**

- Phase 2: DONE — `ai_generated_grasshopper_python/visualize_waypoints.py`
  - Input: base_origin (optional, defaults to world origin)
  - Outputs: planes, names, move_types, edge_lines, edge_names, edge_statuses
  - Reads base_cone_waypoints.yaml directly (path hardcoded)
  - **TODO: paste into GH component, verify planes appear at correct positions,
    verify edge lines connect approach↔grab correctly, colour by move_type and edge_statuses**

- Phase 3 (RoboDK import): DONE — `robodk_code/import_waypoints_to_robodk.py`
  - Run: `python robodk_code/import_waypoints_to_robodk.py`
  - Creates one RoboDK Target per waypoint under "WaypointTargets" parent frame
  - Blue = MoveJ (approach), Green = MoveL (grab/place)
  - **TODO: run with RoboDK open, verify targets appear at correct world positions,
    check that robot_local frame offset (base_origin) is applied correctly**

- Phases 4–6: NOT STARTED

## ── RULE: joints must always be present ──────────────────────────────────────

**Every waypoint in `path_config.yaml` must have an explicit `joints:` list.**
Never use Cartesian-only (`target:`, `x/y/z`) waypoints for routing — joint values
must be validated by a human and committed. This is enforced by `tests/test_path_config.py`
which will fail CI if any waypoint is missing `joints:`.

Use `save_joint_position.py` to capture joints from RoboDK:
```bash
python robodk_code/save_joint_position.py --robodk-ip 172.23.208.1
# prompts for a name, appends to path_config.yaml with source: human
```

**path_config.yaml is committed** (gitignore exception). Commit it every time you
add or modify a waypoint.

## ── `source` field in waypoint YAMLs ─────────────────────────────────────────

Every waypoint entry in `all_waypoints.yaml` / `base_cone_waypoints.yaml` /
`machine_cone_waypoints.yaml` has a `source:` field:
- `source: grasshopper` — exported by a GhPython component automatically
- `source: human` — captured via `save_joint_position.py`

This makes it auditable which waypoints were validated by a human.

## ── RoboDK IK API ────────────────────────────────────────────────────────────

**`robot.SolveIK(pose, joints_approx=seed)`**
Returns the single IK solution closest to `seed` in joint space. Use this when
you want to seed toward a known good arm configuration for a zone. `joints_approx`
bakes in the j7 position too.

**`robot.SolveIK_All(pose)`**
Returns all discrete IK solutions as a 2D matrix `[N_joints × N_solutions]`.
For a 6-DOF arm there are up to 8 discrete solutions (shoulder/elbow/wrist
combinations). **Important:** for a 7-DOF robot (6+rail), j7 is a continuous
redundancy — `SolveIK_All` returns the 8 arm solutions at the CURRENT j7 position.
It does NOT explore the j7 manifold. To cover the full solution space you would
need to sample j7 positions and call `SolveIK_All` at each — effectively 8 × N_j7.
This is why seeded IK (`SolveIK` with `joints_approx` fixing j7) is preferred.

**`robot.JointsConfig(joints)`**
For any joint solution, returns configuration flags `[REAR, LOWERARM, FLIP]`:
- `REAR`: 1=rear, 0=front (shoulder side)
- `LOWERARM`: 1=lower, 0=upper (elbow direction)
- `FLIP`: 1=flip, 0=non-flip (wrist)

Use to filter solutions for a specific arm configuration:
```python
all_sols = robot.SolveIK_All(pose)
for j in all_sols:
    cfg = robot.JointsConfig(j).list()
    if cfg[0] == 0 and cfg[1] == 0:  # front, elbow up
        joints = j; break
```

## ── Cartesian waypoints → joint poses ────────────────────────────────────────

GH exports Cartesian poses (world coords) in `all_waypoints.yaml`. For path
planning, `path_config.yaml` needs joint values. Three options:

**Option A — `target:` key in path_config.yaml (recommended for cone grabs)**
`check_collision_free_paths.py` already supports `target: "waypoint_name"` — it
resolves the named RoboDK target to joints via SolveIK at plan-creation time.
Since `import_waypoints_to_robodk.py` already creates RoboDK Targets from the
YAML, reference them directly. No extra step needed.
```yaml
waypoints:
  base_cone_grab_0:
    target: "base_cone_grab_0"   # resolved to joints by check_collision_free_paths.py
```

**Option B — Batch IK solve script (not yet built)**
Script reads `all_waypoints.yaml`, solves IK for every world-coord waypoint using
OptimAxes (j7 constrained to zone X-position), writes joint values back. Good for
automating many targets. Risk: wrong IK solution for some poses; needs seed joints
per zone. Build this when Option A proves insufficient.

**Option C — Jog-and-save in RoboDK (recommended for routing candidates)**
1. `import_waypoints_to_robodk.py` to see Cartesian targets in station
2. "Move to Target" in RoboDK GUI to drive robot there
3. `python robodk_code/save_joint_position.py --robodk-ip 172.23.208.1` — prompts
   for a name, appends joint values to `path_config.yaml`
4. Add name to `routing_candidates:` in path_config.yaml

Use Option C for transport/curtain-safe routing candidates (few in number, arm
configuration needs human sign-off). Use Option A for the many cone grab targets.

**Euler angle convention — IMPORTANT**
GH scripts extract ZYX Euler angles: R = Rz * Ry * Rx (rotate X first, Z last).
RoboDK's `TxyzRxyz_2_Pose` uses the opposite order (Rx * Ry * Rz). Always build
RoboDK Mat directly from the Rz*Ry*Rx formula — see `build_pose()` in
`import_waypoints_to_robodk.py` for the reference implementation.

## ── PRIORITY #1 (as of 2026-05-21) ──────────────────────────────────────────

**Goal:** Validate our Dijkstra path planning cycle for our end effector.

**Our cycle (not DHR's):**
1. Define waypoints in `path_config.yaml` (routing candidates + destination groups)
2. Run `check_collision_free_paths.py` — tests edges offline with RoboDK collision checking
3. Result baked into `path_plan.yaml`
4. `moving_a_cone.py` trusts the plan and executes without live collision checking

**What we are doing right now:**
Extract DHR's proven intermediate waypoints from `robodk.yaml` (the CurtainSafe frames,
transport/buffer poses, OptimizationApproach frames) and feed them into our
`path_config.yaml` as routing candidates. Then run our collision checker with OUR
robot + OUR end effector (cone/pickup tool) to see if those DHR-proven paths are
also valid for our setup. This tests whether our Dijkstra approach is viable.

**We are NOT adopting DHR's movement logic** (their move_task.py sequencing, their
State class pipeline, their Redis caching, their live MoveJTestModel). We are using
their YAML data (frame positions) as input to our own tools.

**Steps:**
1. Pull CurtainSafe frames + transport/buffer poses from robodk.yaml into path_config.yaml
2. Run `check_collision_free_paths.py` with RoboDK open
3. Inspect `path_plan.yaml` — which edges are clear for our end effector?
4. If viable: proceed to full cone-move test end-to-end
5. If not viable: identify which edges fail and add intermediate waypoints

## ── RESUME POINT (left off 2026-05-21) ──────────────────────────────────────

**Branch:** `collision_free_path_planning` (branched from `determining_how_position_gripper`)

**Status:** All code complete and pushed (32/32 tests passing). Pending: populating
`path_config.yaml` with real waypoints and running the pipeline.

**What works:**
- `determining_how_position_gripper`: cone placement fix — `cone_mesh.setPose(invH(world_frame.PoseAbs()) * tgt_grab_pose)` with diagnostic logging
- `robot_controller.py`: `PathEvaluationModel` + `MoveJModel` mixin pipeline, edge cache
- `check_collision_free_paths.py`: config loading, hash computation, Dijkstra pathfinding, gateway discovery, node construction, edge testing, plan writer, `main()`
- `path_plan_utils.py`: pure-Python plan loading, cone filtering, sequence building, edge validation — no RoboDK dependency, fully unit-tested
- `moving_a_cone.py`: plan-driven motion sequence using `path_plan_utils`
- `robo_dk_output/path_config.yaml`: human-editable template committed (gitignore exception added)
- `robodk_code/extract_waypoint_frames.py`: dumps named frames from RoboDK as 6D poses (JSON + CSV) for Rhino import
- Conda environments (WSL, miniconda at `/home/samst/miniconda3`):
  - `cone_planner` — pytest + pyyaml; use for `pytest tests/` (no robodk)
  - `robodk_v1` — has robodk installed; use for running any `robodk_code/` scripts
  - To activate from bash: `source /home/samst/miniconda3/etc/profile.d/conda.sh && conda activate <env>`
  - TODO: install robodk into `cone_planner` so one env covers both tests and scripts
  - NOTE: `sys.path.append("C:/RoboDK/Python")` in scripts is a Windows path; silently
    ignored in WSL. Works because robodk is pip-installed in `robodk_v1`.
  - WARNING: the `robodk_v1` conda env on the machine may not match what the repo
    expects (robodk version, other deps). If scripts behave unexpectedly, compare
    `pip list` in `robodk_v1` against any requirements files in the repo and align them.
    TODO: add a `requirements_robodk.txt` or extend `environment.yml` to lock the
    robodk version so the env is reproducible.

**j7 testing shortcut (agreed 2026-05-20):**
For the initial sim validation use a small number of j7 routing candidates (3-4) to
keep the edge count manageable. When moving to production, add the full set of j7
positions and re-run `check_collision_free_paths.py` to test all combinations. The
hash system invalidates and retests everything when routing_candidates changes.

**Next steps:**
1. Populate `robo_dk_output/path_config.yaml` with actual machine zones, gateway waypoints, and multi-j7 routing candidates (see design note below)
2. Run `python robodk_code/check_collision_free_paths.py` to generate `path_plan.yaml`
3. Run `python robodk_code/load_path_plan_to_robodk.py` to visualise all planned positions in RoboDK
4. Test end-to-end: `python robodk_code/moving_a_cone.py --mode ai --base 0 --dest 0`
5. Merge `determining_how_position_gripper` into `collision_free_path_planning`

## ── DESIGN: Multi-j7 routing candidates ──────────────────────────────────────

**Decision (2026-05-19):** The 7th axis is a linear rail. Base cone targets are defined
relative to the robot base frame, so the robot picks them up at j7=0 (rail home). Destination
cones are in world space and may require various rail positions.

**Problem:** A single `transport` waypoint fixes j7 at one value, forcing the rail to make
unnecessary round-trips for destinations that would be cheaper to reach at a different j7.

**Solution:** Define routing candidates at multiple j7 positions:

```yaml
waypoints:
  home:
    joints: [0, 0, 0, 0, 0, 0, 0]
  transport_j7_0:
    joints: [0, -55, 30, 0, -30, -90, 0]
  transport_j7_500:
    joints: [0, -55, 30, 0, -30, -90, 500]

routing_candidates:
  - home
  - transport_j7_0
  - transport_j7_500
```

Dijkstra picks the cheapest collision-free route through whichever rail position works
for each base→destination pair. No code changes needed — the graph handles it.

**Initial scope:** Start with 3-4 j7 positions and 1-2 machines to validate the approach
before scaling. ~28 machines over 20m → zone-based waypoints make more sense than
uniform 20cm spacing (only 2x node reduction but much simpler config).

**Adding new waypoints:** always requires re-running `check_collision_free_paths.py` to
test the new edges. The hash system treats any change to `routing_candidates` as a full
invalidation — there is no partial re-run.

### grab_family — functionally equivalent base grabs at different j7

Base cones are robot-relative, so the same cone slot is reachable at any j7 position
(arm pose is identical, only rail position differs). In `path_plan.yaml`, separate entries
exist per j7 position but share a `grab_family` field:

```yaml
base_cones:
  base_cone_grab_0_j7_0:
    grab_family: base_slot_0      # same physical cone, rail at 0
    tested: true
    approach_joints: [0, -55, 30, 0, -30, -90, 0]
    grab_joints: [...]
    gateways: [transport_j7_0]
  base_cone_grab_0_j7_500:
    grab_family: base_slot_0      # same cone slot, rail at 500
    tested: true
    approach_joints: [0, -55, 30, 0, -30, -90, 500]
    grab_joints: [...]
    gateways: [transport_j7_500]
```

At execution time `moving_a_cone.py` selects whichever family member has a tested
collision-free path from the robot's current rail position — avoids unnecessary rail
travel back to j7=0 if already near another family member.

**Not yet implemented** — document the field in path_plan.yaml when building this out.

### pose_family — future work

Two routing candidates qualify for the same `pose_family` if:
1. They have identical arm joints (j1-j6)
2. There is a tested, collision-free **chain of adjacent j7 steps** connecting them at
   that arm pose — i.e. the rail can slide between them without hitting anything

Both conditions must hold. Identical arm joints alone is not sufficient — an obstacle
may block the rail at an intermediate j7. The chain must be explicitly tested.

Once a chain is confirmed, the checker can treat the family as a 1D rail corridor and
skip re-testing arm collisions between members (arm doesn't move, environment was
already checked step by step). Leave for later — not needed at 3-4 j7 nodes.
