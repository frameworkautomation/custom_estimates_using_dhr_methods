"""
Copy end effectors from one station to another.

Copies:
1. The EndEffector frame (with geometry children) — placed under the robot
2. The tools (pickup, knotting, cutting) — placed as direct children of the robot

Each item copied one at a time to keep clipboard alive across station loads.

Usage:
    python robert_checker_stuff/copy_end_effectors.py --robodk-ip 172.23.208.1 \
        --source robo_dk_saves/MachineReachability.rdk \
        --dest robo_dk_saves/generated_from_dhr_clone.rdk
"""

import sys
import os
import argparse
import time

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


def wsl_to_win(path):
    abs_path = os.path.abspath(path)
    if abs_path.startswith("/mnt/"):
        drive = abs_path[5]
        return f"{drive.upper()}:{abs_path[6:]}".replace("/", "\\")
    return abs_path


def find_robot(RDK):
    for name in ROBOT_NAMES:
        r = RDK.Item(name, ITEM_TYPE_ROBOT)
        if r.Valid():
            return r
    raise RuntimeError(f"Robot not found. Tried: {ROBOT_NAMES}")


def close_all(RDK):
    for s in RDK.ItemList(ITEM_TYPE_STATION):
        s.Delete()


def copy_item_between_stations(RDK, source_win, dest_win, item_name, item_type,
                               paste_parent_fn):
    """Load source, copy item, load dest alongside, paste under parent.

    paste_parent_fn(RDK) -> Item to paste under.
    Returns True if successful.
    """
    close_all(RDK)
    RDK.AddFile(source_win)
    time.sleep(0.5)

    item = RDK.Item(item_name, item_type)
    if not item.Valid():
        print(f"  [WARN] '{item_name}' not found in source — skip")
        return False

    item.Copy()
    print(f"  [COPY] '{item_name}' from source")

    # Load dest alongside (clipboard stays alive)
    RDK.AddFile(dest_win)
    time.sleep(0.5)

    # Switch to dest station
    stations = RDK.ItemList(ITEM_TYPE_STATION)
    if len(stations) > 1:
        # Dest is the second one loaded
        RDK.setActiveStation(stations[-1])

    parent = paste_parent_fn(RDK)
    pasted = parent.Paste()
    if pasted.Valid():
        print(f"  [PASTE] '{pasted.Name()}' under '{parent.Name()}'")
        RDK.Save(dest_win)
        return True
    else:
        print(f"  [FAIL] Could not paste '{item_name}'")
        return False


def main():
    ap = argparse.ArgumentParser(description="Copy end effectors between station files")
    ap.add_argument("--robodk-ip", default=None)
    ap.add_argument("--source", required=True, help="Source .rdk file path")
    ap.add_argument("--dest", required=True, help="Destination .rdk file path")
    ap.add_argument("--tools", nargs="*", default=TOOLS_TO_COPY,
                    help=f"Tool names to copy (default: {TOOLS_TO_COPY})")
    args = ap.parse_args()

    RDK = connect(args.robodk_ip)
    source_win = wsl_to_win(args.source)
    dest_win = wsl_to_win(args.dest)

    # Step 0: Clean dest — remove old tools and EndEffector frame
    print(f"[INFO] Cleaning dest: {args.dest}")
    close_all(RDK)
    RDK.AddFile(dest_win)
    time.sleep(1)

    for tool_name in args.tools:
        existing = RDK.Item(tool_name, ITEM_TYPE_TOOL)
        if existing.Valid():
            print(f"  [DELETE] tool '{tool_name}'")
            existing.Delete()

    existing_ee = RDK.Item(END_EFFECTOR_FRAME, ITEM_TYPE_FRAME)
    if existing_ee.Valid():
        print(f"  [DELETE] frame '{END_EFFECTOR_FRAME}'")
        existing_ee.Delete()

    RDK.Save(dest_win)
    print(f"  [SAVE] Cleaned dest")

    # Step 1: Copy EndEffector frame (with geometry children) under robot
    print(f"\n── Copying '{END_EFFECTOR_FRAME}' frame ──")
    copy_item_between_stations(
        RDK, source_win, dest_win,
        END_EFFECTOR_FRAME, ITEM_TYPE_FRAME,
        paste_parent_fn=find_robot,
    )

    # Step 2: Copy each tool as direct child of robot
    for tool_name in args.tools:
        print(f"\n── Copying tool '{tool_name}' ──")
        copy_item_between_stations(
            RDK, source_win, dest_win,
            tool_name, ITEM_TYPE_TOOL,
            paste_parent_fn=find_robot,
        )

    # Final verification
    print(f"\n── Verification ──")
    close_all(RDK)
    RDK.AddFile(dest_win)
    time.sleep(0.5)

    ee = RDK.Item(END_EFFECTOR_FRAME, ITEM_TYPE_FRAME)
    print(f"  [{'OK' if ee.Valid() else 'MISSING'}] {END_EFFECTOR_FRAME} frame")
    if ee.Valid():
        for c in ee.Childs():
            print(f"    child: {c.Name()} (type={c.Type()})")

    for tool_name in args.tools:
        tool = RDK.Item(tool_name, ITEM_TYPE_TOOL)
        parent = tool.Parent().Name() if tool.Valid() else "?"
        status = "OK" if tool.Valid() else "MISSING"
        print(f"  [{status}] {tool_name} (parent={parent})")

    print("\n[DONE]")


if __name__ == "__main__":
    main()
