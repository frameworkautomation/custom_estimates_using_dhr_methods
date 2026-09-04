# Project Context for Claude

## ── BRANCH: positioning_robert_end_effector_etc ──────────────────────────────

**Goal:** Evaluate end-effector mounting and reachability for Robert's tools in simulation.

**Tasks:**
1. Mount Robert's end effector in the RoboDK simulation
2. Test whether the end effector can reach into the box (cone holder) from its
   mounted position
3. Test whether the end effector can reach the cones on the creel
4. Test whether the end effector can reach the base cones for the knotter and
   knot picker-upper
5. Check the cutting operation in simulation — verify the tool can perform the
   cut motion without collisions or reachability issues

**DONE: Base cone vacuum pickup movement sequence**

Phases 1–6 of `setup_base_movements.py` are working. IK solving, offset
generation, and program population all complete. 24/24 targets solved on the
6-DOF extracted station.

**TODO (top priority): Collision-aware movement sequences**

Three tasks, in order:

1. **Attach cone to end effector at grab point.** After the robot reaches the
   grab target, parent the cone mesh to the tool so it moves with the robot for
   the rest of the sequence. This is needed so collision checking includes the
   held cone geometry.

2. **Collision avoidance for the movement sequence.** Currently the programs use
   direct MoveJ/MoveL between targets with no intermediate waypoints. Some of
   these moves may collide with the cone holder, adjacent cones, or the bin.
   Options to investigate:
   - Add more offset waypoints (coded in config or auto-generated) to route
     around obstacles
   - Use `MoveJ_Test` / `MoveL_Test` to detect collisions and insert
     intermediate waypoints where needed
   - Manual intermediate frames placed in RoboDK

3. **Debug MoveJ failures to certain targets.** Some MoveJ instructions in the
   populated programs fail at runtime (robot can't reach or path is invalid).
   Investigate why — could be IK config mismatch between the solve-time pose and
   the program target, or the joint seed differing between Phase 4 solving and
   program execution.

**TODO (script 1): Station extraction — `extract_station.py`**

Single-file script that reads a source RoboDK station and builds a new station
containing only the items listed in a config file. Gives repeatability — run it
every time you need a clean starting point for movement-sequence testing.

**CLI:**
```
python robert_checker_stuff/extract_station.py \
  --source for_robert_n1.rdk \
  --dest for_robert_relative_to_base.rdk \
  --config robert_checker_stuff/station_extract_config.json
```
All three arguments have defaults:
- `--source`: `for_robert_n1.rdk`
- `--dest`: `for_robert_relative_to_base.rdk`
- `--config`: `robert_checker_stuff/station_extract_config.json`

**Config file (`station_extract_config.json`):**
Lists what to copy — objects, frames, end-effector elements, poses, targets, etc.
One config serves all extraction needs. Schema TBD but something like:
```json
{
  "items": [
    {"name": "Fanuc R-2000iC/125L", "type": "robot"},
    {"name": "pickup", "type": "tool"},
    {"name": "Base_Right_0", "type": "object"},
    ...
  ]
}
```

**The main challenge: extracting the robot arm without the track.**

In the source station the robot is a child of the linear rail mechanism (j7).
We need to copy the 6-DOF arm into the new station positioned where it sits
when j7 = 0 — but without the rail. This means:
- Compute the robot base pose at j7 = 0 in world space
- Place the robot in the new station at that world pose, parented to a plain
  frame (not a mechanism)
- Copy only j1–j6 joint values; j7 does not exist in the output station
- Assert that the robot resolves correctly in the new station (FK check: same
  TCP world pose for the same j1–j6 as in the source at j7 = 0)

Other items (end effectors, cones, frames) are simpler — read their world pose
from the source, create them in the destination at the same pose.

**Assertion policy:** The script asserts on every item listed in the config. If
any item cannot be found in the source station, the script raises an
`AssertionError` with the missing item name — never silently skips.

**TODO (script 2): Movement sequence builder**

Separate script (to be written after script 1 is working). Reads the extracted
station and the checker results, then builds RoboDK paths/points/programs for
the vacuum pickup movement sequence. Will use heavy assertions initially to
verify it has everything it needs from the extracted station before proceeding.

`SolveIK_All` corrupts the robot-rail connection in this station. Do NOT use it.
Instead, implement z_axis_free as a rotation sweep using RoboDK frame manipulation:

**Class: `ZAxisFreeSolver`**
- Instantiated once at the start of the checker run
- On init: creates a parent frame `DiscoveredWaypoints` in the RoboDK station
- On init: creates a child frame `temp` under `DiscoveredWaypoints`
- On each run of `solve()`: deletes and recreates `DiscoveredWaypoints` (clean slate)

**Method: `solve(robot, RDK, target_pose, j7_target, N=72)`**
1. For `i` in `range(N)`:
   a. Compute rotation angle = `360 * i / N` degrees
   b. Create a rotated copy of the target pose: rotate around the target's own Z axis
      by that angle
   c. Set `temp` frame's pose to the rotated pose
   d. Use the existing `_solve_ik_locked_j7(robot, RDK, temp_pose, j7_target)` to
      attempt IK (OptimAxes + MoveJ — the proven working approach)
   e. If it succeeds AND passes FK verification (err < 50mm):
      - Create a frame `discovered_waypoint_<name>` under `DiscoveredWaypoints`
        with the solved pose, for visual inspection
      - Return the joints
   f. If it fails, continue to next angle
2. If no angle works after N attempts, return failure

**Key constraints:**
- Do NOT use `SolveIK_All` — it corrupts the robot-rail connection in this station
- Do NOT use `move_to_base_cone_grab.py` — it uses `SolveIK_All` internally
- Use the existing `_solve_ik_locked_j7` (OptimAxes + MoveJ) which is proven to work
- The `temp` frame is reused each iteration (set pose, not recreated)
- `DiscoveredWaypoints` is deleted and recreated at the start of each `solve()` call
- Only supported with `Locked_at_j7_0` — error on other track conditions

**Where it goes:**
- New class in `robert_checker_stuff/robert_end_checker.py`
- Called from `solve_point()` when `z_axis_free=True`
- The checker instantiates the class once after connecting to RoboDK

**TODO (script 2, Phase 5): targets_to_use.json + program population**

Two steps: (A) build a merged target lookup file, (B) populate each RoboDK program.

**A. `targets_to_use.json`** — written by `setup_base_movements.py`

All solved targets (whether they needed Z-rotation or not) end up in
`targets_rotated_for_solution/`. Phase 4 creates the main target in `extracted/`
and its before/after offsets in `auto_generated_offsets/before|after` — always as
a group based on the solved pose. So everything the programs need lives in one
place.

`targets_to_use.json` lists only cones where **both** grab and string_grab solved.
Cones with any failure are excluded entirely.

```json
{
  "Base_Right_0": {
    "grab": {
      "target": "Base_Right_0_grab",
      "before": "offset_before_for_Base_Right_0_grab",
      "after": "offset_after_for_Base_Right_0_grab"
    },
    "string_grab": {
      "target": "Base_Right_0_string_grab",
      "before": "offset_before_for_Base_Right_0_string_grab",
      "after": "offset_after_for_Base_Right_0_string_grab"
    }
  }
}
```

All names refer to items inside `targets_rotated_for_solution/` subfolders.

**B. Program population** — also in `setup_base_movements.py` Phase 5

For each cone program (e.g. `Base_Right_0`), add instructions:

1. MoveJ to home (joint target at all-zeros)
2. **String grab sequence** (set knotting tool):
   - MoveL to `offset_before_for_<cone>_string_grab`
   - MoveL to `<cone>_string_grab`
   - MoveL to `offset_after_for_<cone>_string_grab`
3. **Pickup grab sequence** (set pickup tool):
   - MoveL to `offset_before_for_<cone>_grab`
   - MoveL to `<cone>_grab`
   - MoveL to `offset_after_for_<cone>_grab`

All moves are MoveL. All targets come from `targets_rotated_for_solution/` subfolders.

Programs that reference failed targets are skipped (not populated).

**TODO (cutting):**
- Implement linear moves for cutting sequence (approach→top→bottom→pull_away should
  use MoveL between top/bottom/pull_away, not MoveJ — deferred for now)
- Implement cutting paths (`type: "path"`) — these are not just single IK checks
  but need to verify the tool can follow a cut trajectory

## ── robert_checker_stuff testing protocol ─────────────────────────────────────

**Test script:** `robert_checker_stuff/test_ik_mini.py`
**Test config:** `robert_checker_stuff/test_config.json`

Before committing any IK solver changes to `robert_end_checker.py`, run the mini
test script. It uses a minimal config with just 2 points:
- `Base_Right_0_grab` (pickup tool, Locked_at_j7_0) — base cone, should be reachable
- `Front_0_grab` (pickup tool, Optimized_for_j7_at 3600) — machine cone, should be reachable

**Both must return `reachable: true`** with non-zero joint values (not all zeros).
The test reads the output JSON and verifies this automatically.

Run: `python robert_checker_stuff/test_ik_mini.py --robodk-ip 172.23.208.1`

If the test fails, the IK solver code is broken. Do not commit.

## ── robert_checker_stuff architecture ────────────────────────────────────────

**Folder:** `robert_checker_stuff/`

**Files:**
- `robert_end_checker.py` — CLI script, checks reachability for each end effector
- `robert_end_shower.py` — CLI script, visualises results
- `robert_end_checker_config.json` — shared config (populated by Grasshopper scripts)

**Rules:**
- Both scripts are CLI Python, interact with the config JSON and RoboDK only — no
  other data sources.

### Config JSON schema

```json
{
  "end_effectors": [
    {
      "end_effector_name": "SomeTool",
      "paths_and_points_to_check": [
        {
          "name": "cone_holder_slot_0",
          "type": "point",
          "name_path": "Station/Robot/ConeHolder/Slot0",
          "special_track_conditions": {
            "type": "None"
          }
        },
        {
          "name": "base_cone_grab_0",
          "type": "point",
          "name_path": "Station/Robot/BaseCones/Cone0",
          "special_track_conditions": {
            "type": "Locked_at_j7_0"
          }
        },
        {
          "name": "creel_cone_5",
          "type": "point",
          "name_path": "Station/Creel/Cone5",
          "special_track_conditions": {
            "type": "Locked_at_j7_pt",
            "j7_value": 1500.0
          }
        }
      ]
    }
  ]
}
```

### paths_and_points_to_check element fields

- **name** — identifier for this check
- **type** — `"point"` or `"path"` (path not implemented yet, point only for now)
- **name_path** — location in the RoboDK station tree (how to find the item)
- **z_axis_free** — `true`/`false` (optional, default false). When true, uses Z-rotation
  sweep from `move_to_base_cone_grab.py` to find IK with free rotation around target Z axis.
  Currently only supported with `Locked_at_j7_0` — errors on other track conditions.
- **special_track_conditions** — how to handle j7 during IK solve:
  - `type: "None"` — free j7, let solver pick whatever
  - `type: "Locked_at_j7_0"` — lock j7 to 0 when solving
  - `type: "Locked_at_j7_pt"` — lock j7 to a specific value; requires additional
    `j7_value` field specifying the locked position
  - `type: "Optimized_for_j7_at"` — optimizer tries to keep j7 close to the given
    `j7_value` but does not error if it ends up far away; soft preference, not a
    hard constraint

### Notes
- Config JSON is populated by modified Grasshopper scripts (not hand-written)
- `type: "path"` is future work — only `"point"` for now

### robert_end_checker.py behaviour
1. Read `end_effectors` list from config JSON
2. For each end effector, iterate through its `paths_and_points_to_check`
3. Find each item in RoboDK via `name_path`
4. Solve IK with appropriate j7 constraints per `special_track_conditions`
5. Report reachability results
6. Write/cache IK solutions to a file for `robert_end_shower.py` to consume

### robert_end_shower.py behaviour
- Reads cached IK solutions produced by `robert_end_checker.py` — does NOT
  solve IK itself or query RoboDK for solutions
- Visualises the results in RoboDK (details TBD)

### Grasshopper → config JSON pipeline
- A GhPython component populates `robert_end_checker_config.json` with points
- GH writes name, type, name_path, and special_track_conditions for each point
- This replaces hand-editing the config JSON

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
├── .gitignore                  # ignores clones/
├── CLAUDE.md                   # this file
├── README.md
├── clones/                     # cloned repos land here (git-ignored)
├── robert_checker_stuff/       # main working folder — IK checker, station scripts, movement builder
├── robodk_code/                # RoboDK utility scripts (IK solvers, DHR port, etc.)
├── robodk_setup/               # one-time station setup scripts
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

## ── TODO: Fix update_clones.sh timeout ──────────────────────────────────────

`update_clones.sh` uses `git remote show origin` to detect each repo's default branch.
This makes a live network call per repo and times out (~8 min for 13 repos).

**Fix:** Replace with a local lookup:
```bash
git -C "$repo_dir" rev-parse --abbrev-ref origin/HEAD 2>/dev/null | sed 's|origin/||'
```
This reads the locally-cached HEAD ref without a network call. Requires that
`git remote set-head origin --auto` was run at clone time (git clone does this by default).
Fallback: if empty, try `git -C "$repo_dir" symbolic-ref refs/remotes/origin/HEAD`.

## ── TODO: Audit DHR ported code for redundancies and integrity ───────────────

We have ported several DHR modules into `robodk_code/` (`state.py`, `dhr_robot.py`,
`state_machine.py`) and modified the XQuery header in the cloned repo. Two checks
are needed before relying on this code in production:

1. **Redundancy check.** Verify that our ported files (`robodk_code/state.py`,
   `dhr_robot.py`, `state_machine.py`) do not duplicate logic that still exists
   in the cloned `knitwear-cell` repo. If DHR updates their upstream code, we
   need to know which parts are ours vs. theirs to avoid drift. Document which
   functions/classes were stripped vs. kept.

2. **Integrity hash for tool geometry.** When we eventually build collision-aware
   movement sequences, we need a way to detect if the tool mesh or end-effector
   contact points have changed since the last validation run. Plan: hash the tool
   STL dimensions + contact-point poses and store alongside any movement validation
   results, so stale results are automatically invalidated on geometry changes.

Not blocking current work.

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
`## ── DHR runtime port` section below for how to generate and use state classes.

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

**move_task.py full path:** `clones/knitwear-cell/src/main/action/move_task.py`

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

## ── Cone placement module: `place_cones.py` ─────────────────────────────────

**File:** `robert_checker_stuff/place_cones.py` — importable module + CLI script.

**Purpose:** Place cone STEP meshes into existing RoboDK frames with optional
coloring. Imports once, then Copy/Pastes for efficiency.

### `place_cones(RDK, frames, step_file, colors=None, seed=None)`
- Takes a list of existing RoboDK frame Items
- Imports the STEP file once via `AddFile`, then `Copy`/`Paste` for the rest
- Colors: if `colors` is a list, cycles through it. If `seed` is given, generates
  random colors with that seed. If neither, leaves default STEP color.
- `Recolor` applied per-instance

**CLI usage:**
```
python robert_checker_stuff/place_cones.py \
  --robodk-ip 172.23.208.1 \
  --frames "Base_Right_0,Base_Right_1,Base_Right_2" \
  --step-file my_assets/sams_simple_cone.stp \
  --colors "#FF0000,#00FF00,#0000FF"
```

**DHR integration note:** For now this is our informal placement tool. Longer term,
cone positions and frames need to be reflected in DHR's station and their YAML
config (which feeds their state machine).
