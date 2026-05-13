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
