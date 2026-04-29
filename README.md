# custom_estimates_using_dhr_methods

## Converting SolidWorks files to STEP

The contractor delivers SolidWorks assemblies (`.SLDASM`) in `clones/`. To use them in RoboDK or other tools, convert them to STEP using Rhino.

### Prerequisites

- Rhino 7 or 8 installed on Windows
- SolidWorks files cloned into `clones/` (run `cloning_stuff/make_clones.sh` first)

### Run the conversion

On your **Windows machine** (not WSL), double-click:

```
run_rhino_convert.bat
```

Or run it from a Windows command prompt:

```bat
cd C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods
run_rhino_convert.bat
```

Rhino will open, convert all `.SLDASM` files found under `clones/`, and exit. Progress prints to the console.

### Output

STEP files are written to `steps_from_SolidWorks/`, mirroring the folder structure of `clones/`:

```
clones/atomic-knitting-machine-tending-cell/V1-01-27-2026/Layout-V1-1-27-2026.SLDASM
  ->
steps_from_SolidWorks/atomic-knitting-machine-tending-cell/V1-01-27-2026/Layout-V1-1-27-2026.step
```

Files that already have a corresponding `.step` output are skipped on re-runs.

`steps_from_SolidWorks/` is git-ignored (derived files).

### Notes

- Assemblies that reference parts from other repos may open partially if those parts are missing. The geometry that does load will still export.
- If Rhino is not found at the default path, edit the `RHINO` variable at the top of `run_rhino_convert.bat`.
