"""
Build the DHR knitwear-cell RoboDK station from robodk.yaml and save as .rdk.

Replicates what main.py does: creates the injector, builds the Station from
robodk.yaml, calls station.build() + station.configure(), then saves.

Prerequisites:
  1. Start RoboDK on port 20502:
     "C:\\RoboDK\\bin\\RoboDK.exe" -NEWINSTANCE -PORT=20502

  2. Install deps in a Python 3.12+ env:
     pip install robodk==5.9.4 pydantic==2.9 injector loguru numpy PyYAML redis grpcio protobuf typing-extensions

  3. Run this script from the knitwear-cell project root:
     cd clones/knitwear-cell
     set ENV_MODE=local
     python ../../robert_checker_stuff/build_dhr_station.py

Output: robo_dk_saves/generated_from_dhr_clone.rdk
"""

import os
import sys
import json

# Must run from the knitwear-cell project root
KNITWEAR_CELL_ROOT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "clones", "knitwear-cell"
)
KNITWEAR_CELL_ROOT = os.path.normpath(KNITWEAR_CELL_ROOT)

SAVE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "robo_dk_saves"
)
SAVE_DIR = os.path.normpath(SAVE_DIR)
SAVE_PATH = os.path.join(SAVE_DIR, "generated_from_dhr_clone.rdk")


def to_robodk_path(path):
    """Convert WSL /mnt/c/... path to C:/... for RoboDK."""
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
    # Ensure we're running from the knitwear-cell root
    original_cwd = os.getcwd()

    if not os.path.isdir(KNITWEAR_CELL_ROOT):
        print(f"[ERROR] knitwear-cell not found at: {KNITWEAR_CELL_ROOT}")
        print("        Run: cloning_stuff/make_clones.sh first")
        sys.exit(1)

    os.chdir(KNITWEAR_CELL_ROOT)
    sys.path.insert(0, KNITWEAR_CELL_ROOT)

    # Force local config
    os.environ["ENV_MODE"] = "local"

    print(f"[INFO] Working directory: {os.getcwd()}")
    print(f"[INFO] ENV_MODE: {os.environ['ENV_MODE']}")

    # Import DHR modules (must be after chdir + sys.path)
    from src.main.di.app_context import AppContext
    import src.main.di.autowired as autowired_mod
    from src.main.di.autowired import autowired
    from injector import Injector

    # Build injector exactly as main.py does
    injector = Injector([AppContext()])
    setattr(autowired_mod, "injector", injector)

    print("[INFO] Injector created, building station...")

    # Get the station and build it
    from src.main.robodk.item_presenter import Station
    station: Station = injector.get(Station)

    station.build()
    print("[INFO] station.build() complete")

    station.configure()
    print("[INFO] station.configure() complete")

    # Save the station
    robolink = station._robolink
    save_path = to_robodk_path(SAVE_PATH)
    print(f"[SAVE] Saving station to: {save_path}")
    robolink.Save(save_path)

    # Check the file
    os.chdir(original_cwd)
    if os.path.exists(SAVE_PATH):
        size = os.path.getsize(SAVE_PATH)
        print(f"[DONE] Saved: {SAVE_PATH} ({size:,} bytes)")
        if size < 5000:
            print(f"[WARN] File is suspiciously small — RoboDK free license may have truncated the save.")
    else:
        print(f"[WARN] Save file not found at {SAVE_PATH}")
        print(f"       RoboDK may not support saving multi-robot stations without a paid license.")
        print(f"       The station is still open in RoboDK on port 20502.")


if __name__ == "__main__":
    main()
