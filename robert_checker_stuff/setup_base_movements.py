"""
Set up the extracted station for base cone movement sequence testing.

Phase 1 — organize items into visible GUI folders:
  - extracted_targets  — all Target items
  - cones              — Base cone objects (Base_Right_*, Base_Left_*, alt_Base_*)
  - bins               — Bin objects (bin_*)

More phases will be added (movement sequences, IK checks, etc.).

Caching: folders and item placements are reused if they already exist.

Usage:
    python robert_checker_stuff/setup_base_movements.py
    python robert_checker_stuff/setup_base_movements.py --robodk-ip 172.23.208.1
"""

import sys
import re
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_TARGET, ITEM_TYPE_OBJECT,
)

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]

CONE_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+$")
BIN_PATTERN = re.compile(r"^bin_\d+$")

FOLDER_DEFS = {
    "extracted_targets": {
        "item_type": ITEM_TYPE_TARGET,
        "filter": None,  # all targets
    },
    "cones": {
        "item_type": ITEM_TYPE_OBJECT,
        "filter": CONE_PATTERN,
    },
    "bins": {
        "item_type": ITEM_TYPE_OBJECT,
        "filter": BIN_PATTERN,
    },
}


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


def get_or_create_folder(RDK, name):
    """Return existing frame named `name`, or create one under the station root."""
    item = RDK.Item(name, ITEM_TYPE_FRAME)
    if item.Valid():
        print(f"[CACHE] Folder '{name}' already exists — reusing")
        return item
    station = RDK.ActiveStation()
    folder = RDK.AddFrame(name, station)
    print(f"[CREATE] Created folder '{name}'")
    return folder


def items_matching(RDK, item_type, pattern):
    """Return all station items of `item_type` whose name matches `pattern` (or all if None)."""
    all_items = RDK.ItemList(item_type)
    if pattern is None:
        return all_items
    return [it for it in all_items if pattern.match(it.Name())]


def move_item_to_folder(item, folder):
    """Reparent `item` under `folder`, preserving its world pose. Returns True if moved."""
    parent = item.Parent()
    if parent.Valid() and parent.Name() == folder.Name():
        return False  # already in the right folder

    world_pose = item.PoseAbs()
    item.setParent(folder)
    item.setPoseAbs(world_pose)
    return True


def main():
    ap = argparse.ArgumentParser(description="Set up extracted station for base cone movement testing")
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    args = ap.parse_args()

    RDK = connect(args.robodk_ip)

    for folder_name, spec in FOLDER_DEFS.items():
        folder = get_or_create_folder(RDK, folder_name)
        matched = items_matching(RDK, spec["item_type"], spec["filter"])

        moved = 0
        skipped = 0
        for item in matched:
            if move_item_to_folder(item, folder):
                moved += 1
            else:
                skipped += 1

        total = moved + skipped
        print(f"[{folder_name}] {total} item(s): {moved} moved, {skipped} already in place")

    # Make all folders visible (expanded in the tree)
    for folder_name in FOLDER_DEFS:
        folder = RDK.Item(folder_name, ITEM_TYPE_FRAME)
        if folder.Valid():
            folder.setVisible(True)

    print("\n[DONE] Station organized.")


if __name__ == "__main__":
    main()
