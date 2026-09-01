"""
Port the EndEffector frame and all its children from one RoboDK station to another.

Loads both stations in the same RoboDK instance, copies the EndEffector subtree
from the source station, and pastes it into the destination station under the robot.

Usage:
    python robert_checker_stuff/port_end_effector.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/port_end_effector.py --robodk-ip 172.23.208.1 \\
        --source for_robert_n1.rdk --dest generated_from_dhr_clone.rdk
"""

import sys
import os
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_ROBOT, ITEM_TYPE_STATION,
    ITEM_TYPE_TOOL,
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
SAVES_DIR = os.path.join(PROJECT_DIR, "robo_dk_saves")

DEFAULT_SOURCE = os.path.join(SAVES_DIR, "for_robert_n1.rdk")
DEFAULT_DEST = os.path.join(SAVES_DIR, "generated_from_dhr_clone.rdk")

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]


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


def connect(ip=None):
    if ip:
        return Robolink(robodk_ip=ip)
    try:
        rdk = Robolink()
        rdk.Item("")
        return rdk
    except Exception:
        return Robolink(robodk_ip="172.23.208.1")


def find_robot(RDK):
    for name in ROBOT_NAMES:
        r = RDK.Item(name, ITEM_TYPE_ROBOT)
        if r.Valid():
            return r
    return None


def main():
    ap = argparse.ArgumentParser(description="Port EndEffector from one station to another")
    ap.add_argument("--robodk-ip", default=None)
    ap.add_argument("--source", default=DEFAULT_SOURCE,
                    help=f"Source .rdk (default: {os.path.basename(DEFAULT_SOURCE)})")
    ap.add_argument("--dest", default=DEFAULT_DEST,
                    help=f"Destination .rdk (default: {os.path.basename(DEFAULT_DEST)})")
    args = ap.parse_args()

    RDK = connect(args.robodk_ip)

    # Remember what's currently open
    current_station = RDK.ActiveStation()
    print(f"[INFO] Current station: {current_station.Name()}")

    # Load source station
    source_path = to_robodk_path(args.source)
    print(f"[LOAD] Loading source: {source_path}")
    src_station = RDK.AddFile(source_path)
    assert src_station.Valid(), f"Failed to load source: {args.source}"
    RDK.setActiveStation(src_station)
    print(f"[INFO] Source station: {src_station.Name()}")

    # Find EndEffector in source
    ee_frame = RDK.Item("EndEffector", ITEM_TYPE_FRAME)
    assert ee_frame.Valid(), "EndEffector frame not found in source station"

    # List what we're copying
    print(f"[INFO] EndEffector children:")
    for child in ee_frame.Childs():
        print(f"  {child.Name()} (type={child.Type()})")
        for gc in child.Childs():
            print(f"    {gc.Name()} (type={gc.Type()})")

    # Copy the EndEffector frame (includes all children)
    ee_frame.Copy()
    print("[COPY] EndEffector copied to clipboard")

    # Load destination station
    dest_path = to_robodk_path(args.dest)
    print(f"[LOAD] Loading destination: {dest_path}")
    dest_station = RDK.AddFile(dest_path)
    assert dest_station.Valid(), f"Failed to load destination: {args.dest}"
    RDK.setActiveStation(dest_station)
    print(f"[INFO] Destination station: {dest_station.Name()}")

    # Check if EndEffector already exists in dest
    existing_ee = RDK.Item("EndEffector", ITEM_TYPE_FRAME)
    if existing_ee.Valid():
        print("[WARN] EndEffector already exists in destination — deleting it first")
        existing_ee.Delete()

    # Find robot in destination to parent the EndEffector under
    robot = find_robot(RDK)
    if robot is not None:
        print(f"[INFO] Found robot: {robot.Name()}")
        # Paste under robot
        pasted = robot.Paste()
    else:
        print("[INFO] No robot found — pasting at station root")
        pasted = RDK.Paste()

    assert pasted.Valid(), "Paste failed"
    print(f"[PASTE] Pasted: {pasted.Name()} (type={pasted.Type()})")

    # Also copy any tools that were under EndEffector's children
    # (tools may be separate items linked to the robot)
    RDK.setActiveStation(src_station)
    tools_to_copy = []
    for tool_name in ["pickup", "knotting", "cutting"]:
        tool = RDK.Item(tool_name, ITEM_TYPE_TOOL)
        if tool.Valid():
            tools_to_copy.append(tool_name)
            tool.Copy()
            RDK.setActiveStation(dest_station)
            pasted_tool = RDK.Paste()
            if pasted_tool.Valid():
                if robot is not None:
                    pasted_tool.setParentStatic(robot)
                print(f"[PASTE] Tool: {pasted_tool.Name()}")
            RDK.setActiveStation(src_station)

    # Switch to destination and save
    RDK.setActiveStation(dest_station)

    print(f"\n[SAVE] Saving destination: {dest_path}")
    RDK.Save(dest_path)

    if os.path.exists(args.dest):
        size = os.path.getsize(args.dest)
        print(f"[DONE] Saved: {args.dest} ({size:,} bytes)")
    else:
        print("[WARN] Save may have failed — check RoboDK")

    # Clean up source station
    RDK.setActiveStation(src_station)
    src_station.Delete()
    RDK.setActiveStation(dest_station)
    print("[CLEANUP] Source station removed from RoboDK")


if __name__ == "__main__":
    main()
