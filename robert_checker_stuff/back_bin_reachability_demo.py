"""
Back bin reachability demo — prove the robot (6-DOF, no rail, j7=0) can reach
the cone bin, pick up cone_bin_buffer, and return home.

Uses direct Python API calls (robot.MoveJ/MoveL) against the extracted station
(for_robert_relative_to_base.rdk), following the same pattern as machine
reachability work.

Usage:
    python robert_checker_stuff/back_bin_reachability_demo.py --robodk-ip 172.23.208.1 --use-current
"""

import sys
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME,
    ITEM_TYPE_OBJECT,
)
from robodk.robomath import Pose_2_TxyzRxyz

# ── CONFIG ──────────────────────────────────────────────────────────────────

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]

# Frames in the station that define the approach/grab sequence
APPROACH_FRAMES = [
    "ApproachConeBinBuffer",       # coarse approach (MoveJ)
    "ApproachConeBinBufferBelow",  # descend into bin area (MoveL)
    "Cone_Bin_Frame",              # at bin (MoveL)
]
RETRACT_FRAMES = [
    "ApproachConeBinBufferUp",     # lift out (MoveL)
    "ApproachConeBinBuffer",       # clear bin area (MoveL)
]

GRAB_OBJECT = "cone_bin_buffer"
GRIPPER_NAME = "GrabbingGripper"

# DHR's transport pose (6-DOF)
TRANSPORT_JOINTS = [0, -50, 15, 0, -15, -90]

SPEED_LINEAR = 200    # mm/s
SPEED_JOINTS = 60     # deg/s


# ── CONNECT ─────────────────────────────────────────────────────────────────

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


def find_robot(RDK):
    for name in ROBOT_NAMES:
        r = RDK.Item(name, ITEM_TYPE_ROBOT)
        if r.Valid():
            return r
    return None


# ── POSE HELPERS ────────────────────────────────────────────────────────────

def frame_pose_for_robot(frame, robot):
    """Get frame pose relative to robot base (the proven pattern)."""
    return frame.PoseWrt(robot.Parent())


def describe_pose(pose):
    t = Pose_2_TxyzRxyz(pose)
    return f"x={t[0]:.1f} y={t[1]:.1f} z={t[2]:.1f}"


# ── MAIN ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Back bin reachability demo")
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    ap.add_argument("--use-current", action="store_true",
                    help="Use the currently open station")
    ap.add_argument("--dry-run", action="store_true",
                    help="Only check items exist, don't move the robot")
    args = ap.parse_args()

    RDK = connect(args.robodk_ip)

    # ── Find items ────────────────────────────────────────────────────
    print("\n[FIND] Looking for items in station...")

    robot = find_robot(RDK)
    assert robot is not None, f"Robot not found. Tried: {ROBOT_NAMES}"
    print(f"  Robot: {robot.Name()}")

    gripper = RDK.Item(GRIPPER_NAME, ITEM_TYPE_TOOL)
    assert gripper.Valid(), f"Tool '{GRIPPER_NAME}' not found"
    print(f"  Tool:  {gripper.Name()}")

    # Find all approach/retract frames
    all_frame_names = set(APPROACH_FRAMES + RETRACT_FRAMES)
    frames = {}
    for fname in all_frame_names:
        f = RDK.Item(fname, ITEM_TYPE_FRAME)
        assert f.Valid(), f"Frame '{fname}' not found in station"
        pose = frame_pose_for_robot(f, robot)
        print(f"  Frame: {fname} -> {describe_pose(pose)}")
        frames[fname] = f

    grab_obj = RDK.Item(GRAB_OBJECT, ITEM_TYPE_OBJECT)
    assert grab_obj.Valid(), f"Object '{GRAB_OBJECT}' not found"
    print(f"  Object: {grab_obj.Name()}")

    print("\n[OK] All items found.")

    if args.dry_run:
        print("[DRY-RUN] Stopping before movement.")
        return

    # ── Setup ─────────────────────────────────────────────────────────
    robot.setPoseFrame(robot.Parent())
    robot.setTool(gripper)
    robot.setSpeed(SPEED_LINEAR, SPEED_JOINTS)

    results = []

    def do_move(move_type, frame_name, label=None):
        """Execute a move and record result."""
        label = label or frame_name
        pose = frame_pose_for_robot(frames[frame_name], robot)
        try:
            if move_type == "J":
                robot.MoveJ(pose)
            else:
                robot.MoveL(pose)
            print(f"  [OK]   Move{move_type} -> {label}")
            results.append((label, True, None))
        except Exception as e:
            err = str(e)
            print(f"  [FAIL] Move{move_type} -> {label}: {err}")
            results.append((label, False, err))

    # ── Move home ─────────────────────────────────────────────────────
    print("\n[MOVE] Going to transport pose...")
    try:
        robot.MoveJ(TRANSPORT_JOINTS)
        print("  [OK]   MoveJ -> transport")
        results.append(("transport_start", True, None))
    except Exception as e:
        print(f"  [FAIL] MoveJ -> transport: {e}")
        results.append(("transport_start", False, str(e)))

    # ── Approach sequence ─────────────────────────────────────────────
    print("\n[MOVE] Approach sequence...")
    do_move("J", APPROACH_FRAMES[0], "approach_coarse")
    do_move("L", APPROACH_FRAMES[1], "approach_below")
    do_move("L", APPROACH_FRAMES[2], "at_bin")

    # ── Simulate grab ─────────────────────────────────────────────────
    print("\n[GRAB] Attaching cone_bin_buffer to gripper...")
    try:
        grab_obj.setParent(gripper)
        print("  [OK]   cone_bin_buffer attached to GrabbingGripper")
        results.append(("grab", True, None))
    except Exception as e:
        print(f"  [FAIL] setParent: {e}")
        results.append(("grab", False, str(e)))

    # ── Retract sequence ──────────────────────────────────────────────
    print("\n[MOVE] Retract sequence...")
    do_move("L", RETRACT_FRAMES[0], "retract_up")
    do_move("L", RETRACT_FRAMES[1], "retract_clear")

    # ── Move home ─────────────────────────────────────────────────────
    print("\n[MOVE] Returning to transport pose...")
    try:
        robot.MoveJ(TRANSPORT_JOINTS)
        print("  [OK]   MoveJ -> transport")
        results.append(("transport_end", True, None))
    except Exception as e:
        print(f"  [FAIL] MoveJ -> transport: {e}")
        results.append(("transport_end", False, str(e)))

    # ── Report ────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    all_ok = True
    for label, ok, err in results:
        status = "PASS" if ok else "FAIL"
        suffix = "" if ok else f" — {err}"
        print(f"  [{status}] {label}{suffix}")
        if not ok:
            all_ok = False

    print("=" * 60)
    if all_ok:
        print("ALL MOVES PASSED — bin is reachable from j7=0 position.")
    else:
        failed = [r[0] for r in results if not r[1]]
        print(f"FAILURES: {', '.join(failed)}")
    print("=" * 60)


if __name__ == "__main__":
    main()
