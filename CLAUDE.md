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

### IK visible motion issue

See sub-optimal note above re: `move_to_base_cone_grab_with_setable_accuracy.py`.

## ── RESUME POINT (left off 2026-05-17) ──────────────────────────────────────

**Branch:** `determining_how_position_gripper`

**Status:** `moving_a_cone.py` full pick-and-place motion works end-to-end (0mm
pos_err on all waypoints). Reverted to commit `1a2d15c`.

### Pick-and-place animation — partially working

**What works:**
- Base cone mesh found via `RDK.Item(f"base_cone_{N}")` (under `BaseCones → BaseCone_N`)
- Mesh moved to TCP with `setPose(invH(parent_abs) * tcp_pose)` then attached with
  `setParentStatic(tool)` — cone follows the gripper during transit
- Moving part (gripper) set to `closed_angle` at startup

**What doesn't work:**
- **Cone persists but in the wrong position.** The cone does stay permanently after
  the script finishes (it does NOT revert to its original base position), but it ends
  up in the wrong place — not at the destination grab pose. The reparent likely works;
  the issue is the position calculation being wrong.

**Attempts tried:**
1. `setParentStatic(world_frame)` + `setPose(tgt_grab_pose)` — cone persists but
   wrong position
2. `setParent(world_frame)` + `setPose(tgt_grab_pose)` — broke transit movement AND
   wrong final position
3. `setParent(tool)` + `setPose(eye(4))` for attach; `setParent(ActiveStation)` +
   `setPose(tgt_grab_pose)` for release — broke transit movement AND wrong position

**Root cause hypothesis:** `tgt_grab_pose` comes from `tgt_target.PoseAbs()` which
is the world pose of the destination grab TARGET. But `world_frame` (or station root)
may not be at world origin — its own `PoseAbs()` may have a non-identity transform.
If so, `setPose(tgt_grab_pose)` on a child of `world_frame` gives wrong world position.

**Next step:** After release, log `world_frame.PoseAbs()` (or `ActiveStation().PoseAbs()`)
to check if it's truly identity. If not, the local pose should be
`invH(world_frame.PoseAbs()) * tgt_grab_pose`. Also log `cone_mesh.PoseAbs()` after
`setPose` to see where it actually ends up vs where it should be.
