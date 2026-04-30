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

### Caller script (one-time setup per machine)

`robodk_setup/setup_station_caller.py` is meant to be copied once to `C:\RoboDK\Scripts\` and left there. It connects to a running RoboDK instance and loads `robo_dk_saves/TestStationFanuc.rdk` into the station.

To use it:
1. Copy `robodk_setup/setup_station_caller.py` to `C:\RoboDK\Scripts\`.
2. Open RoboDK.
3. Run it via: Tools > Run Script > setup_station_caller.

The caller script must not be moved into the RoboDK Scripts folder permanently from the repo — it lives in `robodk_setup/` in the repo and is copied manually. This avoids committing anything into RoboDK's install directory.

`robodk_setup/setup_station.py` is an older script that builds the station from scratch (loads robot, cell layout STEP, and Shima Seiki STLs individually). Use the caller/saved-station approach instead unless rebuilding from scratch is needed.
