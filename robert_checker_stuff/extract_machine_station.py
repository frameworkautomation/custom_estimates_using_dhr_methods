"""
Extract a minimal station for machine reachability testing.

Copies from the source station:
  - RailMechanismBase (with all children — robot, rail, buffer, tools, etc.)
  - Fence (top-level object)
  - Machine{1,2,3}Base (with children, excluding items containing 'oil')
  - Machine{1,2,3}GarmentTrayBase (with children)
  - Machine{1,2,3}BackBase (with children)

Usage:
    python robert_checker_stuff/extract_machine_station.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/extract_machine_station.py --robodk-ip 172.23.208.1 --dest machine_reachability.rdk

AI-generated code (Claude Opus 4.6) — human-reviewed before use.
"""

import sys
import os
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT, ITEM_TYPE_STATION,
    ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL,
)
from robodk.robomath import eye

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DEFAULT_DEST = os.path.join(REPO_ROOT, "robo_dk_saves", "machine_reachability.rdk")

MACHINES = [1, 2, 3]


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


def contains_oil(item):
    """Check if item name contains 'oil' (case-insensitive)."""
    return "oil" in item.Name().lower()


def copy_item_to_station(item, dest_station, skip_oil=False):
    """Copy an item (with children) to the destination station.

    If skip_oil is True, recursively removes children containing 'oil' after pasting.
    """
    item.Copy()
    pasted = dest_station.Paste()
    assert pasted.Valid(), f"Failed to paste {item.Name()}"
    print(f"[COPY] {item.Name()}")

    if skip_oil:
        remove_oil_children(pasted)

    return pasted


def remove_oil_children(item):
    """Recursively remove children whose names contain 'oil'."""
    for child in item.Childs():
        if contains_oil(child):
            print(f"[REMOVE] {child.Name()} (contains 'oil')")
            child.Delete()
        else:
            remove_oil_children(child)


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

    # Create new station
    dest_station = RDK.AddStation("MachineReachability")
    RDK.setActiveStation(dest_station)
    print(f"[INFO] Created destination station: MachineReachability")

    # Switch back to source to copy items
    RDK.setActiveStation(source_station)

    # 1. Copy RailMechanismBase (robot, rail, buffer, tools — everything)
    rail = RDK.Item("RailMechanismBase")
    assert rail.Valid(), "RailMechanismBase not found"
    RDK.setActiveStation(dest_station)
    copy_item_to_station(rail, dest_station)
    RDK.setActiveStation(source_station)

    # 2. Copy Fence
    fence = RDK.Item("Fence")
    assert fence.Valid(), "Fence not found"
    RDK.setActiveStation(dest_station)
    copy_item_to_station(fence, dest_station)
    RDK.setActiveStation(source_station)

    # 3. Copy Machine 1, 2, 3 items (Base, GarmentTrayBase, BackBase)
    for m in MACHINES:
        for suffix in ["Base", "GarmentTrayBase", "BackBase"]:
            name = f"Machine{m}{suffix}"
            item = RDK.Item(name)
            if not item.Valid():
                print(f"[WARN] {name} not found, skipping")
                continue
            RDK.setActiveStation(dest_station)
            skip_oil = (suffix == "Base")  # Only filter oil from MachineNBase
            copy_item_to_station(item, dest_station, skip_oil=skip_oil)
            RDK.setActiveStation(source_station)

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
