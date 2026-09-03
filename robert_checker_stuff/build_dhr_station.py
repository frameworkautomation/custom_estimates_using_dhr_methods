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

Output: robo_dk_saves/generated_from_dhr_clone.rdk (or --name <name>)
"""

import argparse
import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

KNITWEAR_CELL_ROOT = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "clones", "knitwear-cell")
)

SAVE_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "robo_dk_saves"))
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


def get_windows_host_ip():
    """Get the Windows host IP from WSL's resolv.conf."""
    try:
        with open("/etc/resolv.conf") as f:
            for line in f:
                if line.strip().startswith("nameserver"):
                    return line.strip().split()[1]
    except FileNotFoundError:
        pass
    return None


def main():
    parser = argparse.ArgumentParser(description="Build DHR station in RoboDK")
    parser.add_argument("--no-save", action="store_true",
                        help="Build into live RoboDK without saving to .rdk file")
    parser.add_argument("--name", type=str, default=None,
                        help="Save filename (e.g. --name my_station). Saves to robo_dk_saves/<name>.rdk")
    args = parser.parse_args()

    original_cwd = os.getcwd()

    if not os.path.isdir(KNITWEAR_CELL_ROOT):
        print(f"[ERROR] knitwear-cell not found at: {KNITWEAR_CELL_ROOT}")
        sys.exit(1)

    os.chdir(KNITWEAR_CELL_ROOT)
    sys.path.insert(0, KNITWEAR_CELL_ROOT)

    os.environ["ENV_MODE"] = "local"

    print(f"[INFO] Working directory: {os.getcwd()}")
    print(f"[INFO] ENV_MODE: {os.environ['ENV_MODE']}")

    # If running from WSL, patch the config to use the Windows host IP
    if sys.platform == "linux":
        windows_ip = get_windows_host_ip()
        if windows_ip:
            print(f"[INFO] WSL detected — patching robodk_ip to {windows_ip}")
            # Monkey-patch the configurator to override robodk_ip
            from src.main.config.configurator import Configurator
            _orig_configure = Configurator.configure
            def _patched_configure(self):
                config = _orig_configure(self)
                if config.robodk_ip in ("localhost", "127.0.0.1"):
                    config.robodk_ip = windows_ip
                return config
            Configurator.configure = _patched_configure

    # Patch AppContext to skip LActivate (fails without paid license)
    from src.main.di.app_context import AppContext
    import src.main.di.autowired as autowired_mod
    from injector import Injector, singleton, provider
    from robodk.robolink import Robolink

    class PatchedAppContext(AppContext):
        @singleton
        @provider
        def provide_robolink(self) -> Robolink:
            config = self.provide_configuration()
            Robolink.NODELAY = config.robolink_nodelay
            rdk = Robolink(config.robodk_ip, config.robodk_port)
            # Skip rdk.Command("LActivate", 1) — breaks without license
            return rdk

    injector = Injector([PatchedAppContext()])
    setattr(autowired_mod, "injector", injector)

    print("[INFO] Injector created, building station...")

    from src.main.robodk.item_presenter import Station
    from src.main.config.configuration import Configuration
    station: Station = injector.get(Station)

    station.build()
    print("[INFO] station.build() complete")

    # build() is @autowired — the injector gave it a Robolink that may now be
    # stale (TCP socket timed out while Python parsed the huge YAML).
    # Replace the cached Robolink in the station object with a fresh connection.
    import time
    time.sleep(1)
    config = injector.get(Configuration)
    station._robolink = Robolink(config.robodk_ip, config.robodk_port)
    # Also replace in the injector's singleton cache so any @autowired calls
    # during configure() get the fresh connection.
    injector.binder.bind(Robolink, to=station._robolink)
    print("[INFO] Reconnected to RoboDK for configure()")

    station.configure()
    print("[INFO] station.configure() complete")

    # Save
    os.chdir(original_cwd)
    robolink = station._robolink

    if args.no_save:
        print("[INFO] --no-save: station built in live RoboDK, file not overwritten.")
    else:
        if args.name:
            name = args.name if args.name.endswith(".rdk") else args.name + ".rdk"
            actual_save_path = os.path.join(SAVE_DIR, name)
        else:
            actual_save_path = SAVE_PATH
        save_path = to_robodk_path(actual_save_path)
        print(f"[SAVE] Saving station to: {save_path}")
        robolink.Save(save_path)
        if os.path.exists(actual_save_path):
            size = os.path.getsize(actual_save_path)
            print(f"[DONE] Saved: {actual_save_path} ({size:,} bytes)")
            if size < 5000:
                print(f"[WARN] File is suspiciously small — RoboDK free license may have truncated.")
        else:
            print(f"[WARN] Save file not found at {actual_save_path}")
            print(f"       The station is still open in RoboDK on port 20502.")


if __name__ == "__main__":
    main()
