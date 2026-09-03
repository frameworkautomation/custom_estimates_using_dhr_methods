# Running DHR's Knitwear-Cell Simulation

## Prerequisites

- **RoboDK** installed at `C:\RoboDK\`
- **Python 3.12** on Windows (`py --version` should work)
- **Podman** (or Docker) for Redis

### One-time dependency install (Windows PowerShell)

```powershell
py -3.12 -m pip install robodk==5.9.4 "pydantic==2.9" injector loguru numpy PyYAML redis grpcio protobuf typing-extensions asyncua opcua
```

## Startup Sequence (3 terminals, all Windows PowerShell)

### Terminal 1 - Start RoboDK

```powershell
& "C:\RoboDK\bin\RoboDK.exe" -NEWINSTANCE -PORT=20502
```

### Terminal 2 - Start Redis + DHR Server

```powershell
# Start Podman VM (only needed once per boot)
podman machine start

# Start Redis (only needed once per boot)
podman run -d -p 6379:6379 --name redis redis
# If "name already in use": podman start redis

# Start the DHR server
cd C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\clones\knitwear-cell
$env:ENV_MODE="local"
py -3.12 main.py
```

Wait for the server to say it's ready on port 50053.

### Terminal 3 - Run the CLI

```powershell
cd C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods
py -3.12 dhr_cli.py
```

## Building the Station (only needed first time or after YAML changes)

Run BEFORE starting `main.py`. Builds all frames, tools, and geometry from
`robodk.yaml` into a live RoboDK station.

```powershell
cd C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\clones\knitwear-cell
$env:ENV_MODE="local"
py -3.12 ..\..\robert_checker_stuff\build_dhr_station.py --name dhr_default
```

Options:
- `--name <name>` saves to `robo_dk_saves/<name>.rdk` (default: `generated_from_dhr_clone.rdk`)
- `--no-save` builds into live RoboDK without saving a file

The saved station (~48MB) can be loaded directly in RoboDK next time instead of
rebuilding: File > Open > `robo_dk_saves/dhr_default.rdk`

## WSL Notes

Running from WSL does NOT work reliably because:
- RoboDK binds to `127.0.0.1:20502` which is unreachable from WSL
- The `-SERVERIP=0.0.0.0` flag does not change the bind address
- Firewall rules alone don't fix it

**Always run from Windows PowerShell.**

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `py` not found | Restart PowerShell after installing Python |
| Redis connection refused | `podman machine start` then `podman run -d -p 6379:6379 --name redis redis` |
| Redis "name already in use" | `podman start redis` (container exists, just start it) |
| RoboDK connection refused | Make sure RoboDK is running on port 20502 |
| gRPC connection refused | Make sure `main.py` is running and says server is ready |
| `LActivate` error | build_dhr_station.py skips this; main.py may need a paid license — investigate |
| Station is empty | Need to build first: run `build_dhr_station.py` |

## What the CLI Can Do

- Oil any machine (1-26) - full 32-point oiling sequence
- Move bins between machine/buffer/rack/cart
- Change tools (GrabbingGripper, MaintenanceGripper)
- Open/close machine doors
- Manual oil pump control
- Get current robot state (joints + TCP pose)
- Acquire/release safety zones
