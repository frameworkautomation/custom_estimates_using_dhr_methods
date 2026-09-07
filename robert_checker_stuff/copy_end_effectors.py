"""
Copy end effectors (pickup, knotting, cutting) from the source station to the
destination station. The tools live under the robot's EndEffector frame.

Switches to source station, finds each tool under EndEffector, copies it,
switches to dest station, pastes it under the robot's EndEffector frame there.

Usage:
    python robert_checker_stuff/copy_end_effectors.py --robodk-ip 172.23.208.1 \
        --source "TestStationFanuc" --dest "MachineReachability"
"""

import sys
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_STATION, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL,
    ITEM_TYPE_FRAME,
)

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]
END_EFFECTOR_FRAME = "EndEffector"
TOOLS_TO_COPY = ["pickup", "knotting", "cutting"]


def connect(ip=None):
    if ip:
        return Robolink(robodk_ip=ip)
    try:
        rdk = Robolink()
        rdk.Item("")
        return rdk
    except Exception:
        return Robolink(robodk_ip="172.23.208.1")


def find_station(RDK, name):
    for s in RDK.ItemList(ITEM_TYPE_STATION):
        if s.Name() == name:
            return s
    raise RuntimeError(f"Station '{name}' not found. Available: {[s.Name() for s in RDK.ItemList(ITEM_TYPE_STATION)]}")


def find_robot(RDK):
    for name in ROBOT_NAMES:
        r = RDK.Item(name, ITEM_TYPE_ROBOT)
        if r.Valid():
            return r
    raise RuntimeError(f"Robot not found. Tried: {ROBOT_NAMES}")


def find_ee_frame(RDK):
    """Find the EndEffector frame under the robot."""
    frame = RDK.Item(END_EFFECTOR_FRAME, ITEM_TYPE_FRAME)
    if frame.Valid():
        return frame
    raise RuntimeError(f"'{END_EFFECTOR_FRAME}' frame not found in active station")


def main():
    ap = argparse.ArgumentParser(description="Copy end effectors between stations")
    ap.add_argument("--robodk-ip", default=None)
    ap.add_argument("--source", required=True, help="Source station name")
    ap.add_argument("--dest", required=True, help="Destination station name")
    ap.add_argument("--tools", nargs="*", default=TOOLS_TO_COPY,
                    help=f"Tool names to copy (default: {TOOLS_TO_COPY})")
    args = ap.parse_args()

    RDK = connect(args.robodk_ip)

    source = find_station(RDK, args.source)
    dest = find_station(RDK, args.dest)
    print(f"[INFO] Source: {source.Name()}")
    print(f"[INFO] Dest:   {dest.Name()}")

    # Find tools in source station
    RDK.setActiveStation(source)
    ee_frame_src = find_ee_frame(RDK)
    print(f"[INFO] Found '{END_EFFECTOR_FRAME}' in source")

    copied = 0
    for tool_name in args.tools:
        # Find tool in source
        RDK.setActiveStation(source)
        tool = RDK.Item(tool_name, ITEM_TYPE_TOOL)
        if not tool.Valid():
            print(f"  [WARN] Tool '{tool_name}' not found in source — skip")
            continue

        # Check if already exists in dest
        RDK.setActiveStation(dest)
        existing = RDK.Item(tool_name, ITEM_TYPE_TOOL)
        if existing.Valid():
            print(f"  [SKIP] '{tool_name}' already exists in dest")
            continue

        # Copy from source, paste into dest
        RDK.setActiveStation(source)
        tool.Copy()
        RDK.setActiveStation(dest)

        # Find or create EndEffector frame in dest
        ee_frame_dst = RDK.Item(END_EFFECTOR_FRAME, ITEM_TYPE_FRAME)
        if not ee_frame_dst.Valid():
            # Create it under the robot
            robot = find_robot(RDK)
            ee_frame_dst = RDK.AddFrame(END_EFFECTOR_FRAME, robot)
            print(f"  [CREATE] Created '{END_EFFECTOR_FRAME}' frame in dest under robot")

        pasted = ee_frame_dst.Paste()
        if pasted.Valid():
            print(f"  [COPY] '{tool_name}' -> dest/{END_EFFECTOR_FRAME}/")
            copied += 1
        else:
            print(f"  [FAIL] Could not paste '{tool_name}'")

    # Reconnect tools to robot in dest
    if copied > 0:
        RDK.setActiveStation(dest)
        robot = find_robot(RDK)
        for tool_name in args.tools:
            tool = RDK.Item(tool_name, ITEM_TYPE_TOOL)
            if tool.Valid():
                robot.setTool(tool)
        print(f"\n[DONE] {copied} tool(s) copied")
    else:
        print("\n[DONE] No tools copied")


if __name__ == "__main__":
    main()
