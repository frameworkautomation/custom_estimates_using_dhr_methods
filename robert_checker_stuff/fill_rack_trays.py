"""
Fill all Rack1GarmentTray slots with GarmentTray mesh.

Imports DHR's GarmentTray.sld once, then Copy/Pastes into each of the 14
rack slots (7 trays x 2 slots). Skips slots that already have a tray object.

Usage:
    python robert_checker_stuff/fill_rack_trays.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/fill_rack_trays.py --robodk-ip 172.23.208.1 --remove

AI-generated code (Claude Opus 4.6) — human-reviewed before use.
"""

import sys
import os
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT
from robodk.robomath import eye

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)

DEFAULT_TRAY_SLD = os.path.join(
    REPO_ROOT, "clones", "knitwear-cell", "resources", "sld", "GarmentTray.sld"
)

NUM_TRAYS = 7
NUM_SLOTS = 2


def connect(ip=None):
    if ip:
        return Robolink(robodk_ip=ip)
    try:
        rdk = Robolink()
        rdk.Item("")
        print("[INFO] Connected to RoboDK on localhost")
        return rdk
    except Exception:
        print("[INFO] localhost failed, trying 172.23.208.1 ...")
        return Robolink(robodk_ip="172.23.208.1")


def wsl_to_win(path):
    """Convert /mnt/c/... to C:\\... for RoboDK."""
    if path.startswith("/mnt/"):
        drive = path[5]
        return f"{drive.upper()}:{path[6:]}".replace("/", "\\")
    return path


def slot_name(tray_num, slot_num):
    return f"Rack1GarmentTray{tray_num}Slot{slot_num}Base"


def tray_obj_name(tray_num, slot_num):
    return f"GarmentTray_Rack1_T{tray_num}S{slot_num}"


def has_tray(slot_frame):
    """Check if a tray object already exists in this slot."""
    for child in slot_frame.Childs():
        if child.Type() == ITEM_TYPE_OBJECT and "GarmentTray" in child.Name():
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description="Fill Rack1 garment tray slots")
    parser.add_argument("--robodk-ip", default=None)
    parser.add_argument("--step-file", default=DEFAULT_TRAY_SLD,
                        help=f"Path to tray mesh (default: DHR's GarmentTray.sld)")
    parser.add_argument("--remove", action="store_true",
                        help="Remove all placed trays instead of adding")
    args = parser.parse_args()

    RDK = connect(args.robodk_ip)

    # Gather all slot frames
    slots = []
    for tray in range(1, NUM_TRAYS + 1):
        for slot in range(1, NUM_SLOTS + 1):
            name = slot_name(tray, slot)
            frame = RDK.Item(name, ITEM_TYPE_FRAME)
            if frame.Valid():
                slots.append((tray, slot, frame))
                print(f"[FOUND] {name}")
            else:
                print(f"[MISSING] {name}")

    assert len(slots) > 0, "No rack slots found in station"

    if args.remove:
        removed = 0
        for tray, slot, frame in slots:
            for child in frame.Childs():
                if child.Type() == ITEM_TYPE_OBJECT and "GarmentTray" in child.Name():
                    print(f"[REMOVE] {child.Name()} from {slot_name(tray, slot)}")
                    child.Delete()
                    removed += 1
        print(f"[DONE] Removed {removed} tray(s)")
        return

    # Import first tray
    abs_step = os.path.abspath(args.step_file)
    assert os.path.exists(abs_step), f"Mesh file not found: {abs_step}"
    win_path = wsl_to_win(abs_step)

    first_tray_obj = None
    placed = 0
    skipped = 0

    for tray, slot, frame in slots:
        if has_tray(frame):
            print(f"[SKIP] {slot_name(tray, slot)} already has a tray")
            skipped += 1
            continue

        if first_tray_obj is None:
            # Import from file for the first one
            print(f"[IMPORT] Loading {win_path}...")
            first_tray_obj = RDK.AddFile(win_path, frame)
            assert first_tray_obj.Valid(), "Failed to import tray mesh"
            first_tray_obj.setPose(eye(4))
            first_tray_obj.setName(tray_obj_name(tray, slot))
            placed += 1
            print(f"[PLACE] {tray_obj_name(tray, slot)} in {slot_name(tray, slot)}")
        else:
            # Copy/Paste for the rest
            first_tray_obj.Copy()
            pasted = frame.Paste()
            assert pasted.Valid(), f"Paste failed for {slot_name(tray, slot)}"
            pasted.setPose(eye(4))
            pasted.setName(tray_obj_name(tray, slot))
            placed += 1
            print(f"[PLACE] {tray_obj_name(tray, slot)} in {slot_name(tray, slot)}")

    print(f"\n[DONE] Placed {placed}, skipped {skipped} (already had trays)")


if __name__ == "__main__":
    main()
