"""
Build the DHR knitwear-cell RoboDK station from robodk.yaml and save as .rdk.

Replicates what main.py does: creates the injector, builds the Station from
robodk.yaml, calls station.build() + station.configure(), then saves.

Prerequisites:
  1. Start RoboDK on port 20502:
     "C:\\RoboDK\\bin\\RoboDK.exe" -NEWINSTANCE -PORT=20502

  2. Install deps in a Python 3.12+ env:
     pip install robodk==5.9.4 pydantic==2.9 injector loguru numpy PyYAML redis grpcio protobuf typing-extensions

  3. Run from WSL:
     source /home/samst/miniconda3/etc/profile.d/conda.sh && conda activate dhr_build
     cd clones/knitwear-cell
     ENV_MODE=local python ../../robert_checker_stuff/build_dhr_station.py

Output: robo_dk_saves/generated_from_dhr_clone.rdk
"""

import os
import sys
import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

KNITWEAR_CELL_ROOT = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "clones", "knitwear-cell")
)

SAVE_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "robo_dk_saves"))
SAVE_PATH = os.path.join(SAVE_DIR, "generated_from_dhr_clone.rdk")


def to_robodk_path(path):
    abs_path = os.path.abspath(path)
    try:
        if abs_path.startswith("/mnt/"):
            parts = abs_path.split("/")
            drive = parts[2].upper()
            rest = "/".join(parts[3:])
            return f"{drive}:/{rest}"
    except (IndexError, AttributeError):
        pass
    return abs_path


def main():
    original_cwd = os.getcwd()

    if not os.path.isdir(KNITWEAR_CELL_ROOT):
        print(f"[ERROR] knitwear-cell not found at: {KNITWEAR_CELL_ROOT}")
        sys.exit(1)

    os.chdir(KNITWEAR_CELL_ROOT)
    sys.path.insert(0, KNITWEAR_CELL_ROOT)

    os.environ["ENV_MODE"] = "local"

    print(f"[INFO] Working directory: {os.getcwd()}")
    print(f"[INFO] ENV_MODE: {os.environ['ENV_MODE']}")

    # Load configuration directly (bypass injector for Robolink)
    from src.main.config.configurator import Configurator
    config = Configurator().configure()

    print(f"[INFO] Connecting to RoboDK at {config.robodk_ip}:{config.robodk_port}")

    # Connect to RoboDK directly — skip the LActivate command that breaks
    from robodk.robolink import Robolink
    Robolink.NODELAY = config.robolink_nodelay
    rdk = Robolink(config.robodk_ip, config.robodk_port)
    print(f"[INFO] Connected to RoboDK")

    # Load station YAML
    with open("src/main/config/robodk.yaml", "r") as f:
        station_config = yaml.safe_load(f)

    from src.main.robodk.item_presenter import Station
    station = Station.model_validate(station_config)
    station.set_index({})

    # Wire up the injector with our pre-connected Robolink
    from injector import Injector, Module, singleton, provider
    import src.main.di.autowired as autowired_mod
    from src.main.di.app_context import AppContext

    index = {}
    station.set_index(index)

    class PatchedContext(AppContext):
        @singleton
        @provider
        def provide_robolink(self) -> Robolink:
            return rdk

        @singleton
        @provider
        def provide_station(self) -> Station:
            self.station = station
            return self.station

        @singleton
        @provider
        def provide_index(self) -> dict:
            return index

    injector = Injector([PatchedContext()])
    setattr(autowired_mod, "injector", injector)

    print("[INFO] Building station...")
    station.build()
    print("[INFO] station.build() complete")

    # Reconnect in case the build dropped the connection
    try:
        rdk.Item("")
    except Exception:
        print("[INFO] Reconnecting to RoboDK...")
        rdk = Robolink(config.robodk_ip, config.robodk_port)

    try:
        station.configure()
        print("[INFO] station.configure() complete")
    except Exception as e:
        print(f"[WARN] station.configure() failed: {e}")
        print("[INFO] Saving station without configure (build-only)")
        # Reconnect again if configure broke the pipe
        try:
            rdk.Item("")
        except Exception:
            print("[INFO] Reconnecting to RoboDK...")
            rdk = Robolink(config.robodk_ip, config.robodk_port)

    # Save
    save_path = to_robodk_path(SAVE_PATH)
    print(f"[SAVE] Saving station to: {save_path}")
    rdk.Save(save_path)

    os.chdir(original_cwd)
    if os.path.exists(SAVE_PATH):
        size = os.path.getsize(SAVE_PATH)
        print(f"[DONE] Saved: {SAVE_PATH} ({size:,} bytes)")
        if size < 5000:
            print(f"[WARN] File is suspiciously small — RoboDK free license may have truncated.")
    else:
        print(f"[WARN] Save file not found at {SAVE_PATH}")
        print(f"       The station is still open in RoboDK on port 20502.")


if __name__ == "__main__":
    main()
