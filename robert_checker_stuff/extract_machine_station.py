"""
Extract a minimal station for machine reachability testing.

Copies from the source station:
  - RailMechanismBase (with all children — robot, rail, buffer, tools, etc.)
  - Fence (top-level object)
  - Machine{1,3,5}Base (with children, excluding items containing 'oil')
  - Machine{1,3,5}GarmentTrayBase (with children)
  - Machine{1,3,5}BackBase (with children)

Usage:
    python robert_checker_stuff/extract_machine_station.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/extract_machine_station.py --robodk-ip 172.23.208.1 --dest rdk_machine_reach_testing.rdk

AI-generated code (Claude Opus 4.6) — human-reviewed before use.
"""

import sys
import os
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT, ITEM_TYPE_STATION,
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DEFAULT_DEST = os.path.join(REPO_ROOT, "robo_dk_saves", "rdk_machine_reach_testing.rdk")

MACHINES = [1, 3, 5]


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
    if path.startswith("/mnt/"):
        drive = path[5]
        return f"{drive.upper()}:{path[6:]}".replace("/", "\\")
    return path


def remove_oil_children(item):
    """Recursively remove children whose names contain 'oil'."""
    for child in item.Childs():
        if "oil" in child.Name().lower():
            print(f"  [REMOVE] {child.Name()} (contains 'oil')")
            child.Delete()
        else:
            remove_oil_children(child)


def build_copy_list():
    """Return list of (item_name, skip_oil) tuples to copy."""
    items = [
        ("RailMechanismBase", False),
        ("Fence", False),
    ]
    for m in MACHINES:
        items.append((f"Machine{m}Base", True))
        items.append((f"Machine{m}GarmentTrayBase", False))
        items.append((f"Machine{m}BackBase", False))
    return items


def main():
    parser = argparse.ArgumentParser(
        description="Extract minimal station for machine reachability testing."
    )
    parser.add_argument("--robodk-ip", default=None)
    parser.add_argument("--dest", default=DEFAULT_DEST,
                        help=f"Destination .rdk file (default: {DEFAULT_DEST})")
    args = parser.parse_args()

    RDK = connect(args.robodk_ip)

    source_station = RDK.ActiveStation()
    assert source_station.Valid(), "No active station"
    print(f"[INFO] Source station: {source_station.Name()}")

    # Build list of names to copy
    copy_list = build_copy_list()

    # Verify all required items exist before creating dest station
    for name, _ in copy_list:
        item = RDK.Item(name)
        if not item.Valid():
            print(f"[WARN] {name} not found — will skip")
        else:
            print(f"[FOUND] {name}")

    # Create new station — this invalidates item handles from source
    dest_station = RDK.AddStation("MachineReachability")
    print(f"[INFO] Created destination station")

    # Copy each item: switch to source, find, copy, switch to dest, paste
    for name, skip_oil in copy_list:
        RDK.setActiveStation(source_station)
        item = RDK.Item(name)
        if not item.Valid():
            print(f"[SKIP] {name}")
            continue

        item.Copy()
        RDK.setActiveStation(dest_station)
        pasted = dest_station.Paste()
        if not pasted.Valid():
            print(f"[FAIL] Could not paste {name}")
            continue
        print(f"[COPY] {name}")

        if skip_oil:
            remove_oil_children(pasted)

    # Save destination station
    RDK.setActiveStation(dest_station)
    win_dest = wsl_to_win(os.path.abspath(args.dest))
    # CHECK_LATER: RDK.Save may fail without paid license on multi-robot stations.
    # If it produces a stub file, save manually from RoboDK GUI instead.
    RDK.Save(win_dest)
    print(f"\n[DONE] Saved to {args.dest}")
    print(f"[INFO] Switch to the new station in RoboDK to verify")


if __name__ == "__main__":
    main()
