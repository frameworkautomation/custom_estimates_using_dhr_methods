"""
Quick check: can the robot reach the bin grip frames at the current bin position?

Tests reachability of the bin approach/grab/retract frames with the
GrabbingGripper tool. Uses the same IK solver as coupled_pivot_demo.

Usage:
    python robert_checker_stuff/check_bin_grip.py --robodk-ip 172.23.208.1

AI-generated code (Claude Opus 4.6) — human-reviewed before use.
"""

import sys
import math

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME
from robodk.robomath import Pose_2_TxyzRxyz, eye

import argparse

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]

# Frames to check — same as back_bin_reachability_demo.py
BIN_FRAMES = [
    "ApproachConeBinBuffer",
    "ApproachConeBinBufferBelow",
    "Cone_Bin_Frame",
    "ApproachConeBinBufferUp",
]

GRIPPER_TOOL_NAME = "GrabbingGripper"

# Import IK helpers from coupled_pivot_demo
from coupled_pivot_demo import (
    _solve_ik_locked_j7, try_ik_z_sweep, HOME_SEED,
    get_config_flags, config_key,
)


def main():
    ap = argparse.ArgumentParser(description="Check bin grip reachability")
    ap.add_argument("--robodk-ip", default=None)
    args = ap.parse_args()

    if args.robodk_ip:
        RDK = Robolink(robodk_ip=args.robodk_ip)
    else:
        RDK = Robolink()
    RDK._setTimeout(120)

    # Find robot
    robot = None
    for name in ROBOT_NAMES:
        r = RDK.Item(name, ITEM_TYPE_ROBOT)
        if r.Valid():
            robot = r
            break
    assert robot is not None, f"Robot not found. Tried: {ROBOT_NAMES}"
    print(f"Robot: {robot.Name()}")

    # Set world frame
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    if not world_frame.Valid():
        station = RDK.ActiveStation()
        world_frame = RDK.AddFrame("WorldFrame", station)
        world_frame.setPose(eye(4))
    robot.setPoseFrame(world_frame)

    # Find gripper tool
    tool = RDK.Item(GRIPPER_TOOL_NAME, ITEM_TYPE_TOOL)
    if not tool.Valid():
        print(f"[WARN] Tool '{GRIPPER_TOOL_NAME}' not found, trying pickup")
        tool = RDK.Item("pickup", ITEM_TYPE_TOOL)
    assert tool.Valid(), "No suitable tool found"
    robot.setPoseTool(tool)
    print(f"Tool: {tool.Name()}")

    robot_base_pose = Pose_2_TxyzRxyz(robot.PoseAbs())
    print(f"Robot base: [{robot_base_pose[0]:.0f}, {robot_base_pose[1]:.0f}, {robot_base_pose[2]:.0f}]")
    print()

    # Check each frame
    results = {}
    for frame_name in BIN_FRAMES:
        frame = RDK.Item(frame_name, ITEM_TYPE_FRAME)
        if not frame.Valid():
            print(f"  {frame_name}: FRAME NOT FOUND")
            results[frame_name] = "not found"
            continue

        pose = frame.PoseAbs()
        xyz = Pose_2_TxyzRxyz(pose)[:3]
        dist = math.sqrt(sum((xyz[i] - robot_base_pose[i]) ** 2 for i in range(3)))

        print(f"  {frame_name}:")
        print(f"    Position: [{xyz[0]:.0f}, {xyz[1]:.0f}, {xyz[2]:.0f}]  dist={dist:.0f}mm")

        # Try direct IK
        joints, ok = _solve_ik_locked_j7(robot, RDK, pose)
        if ok:
            cfg = get_config_flags(robot, joints)
            j5 = joints[4]
            print(f"    Direct IK: OK  config={config_key(cfg)}  J5={j5:.1f}")
            results[frame_name] = "direct OK"
            continue

        # Try Z-sweep
        j, rp, angle = try_ik_z_sweep(robot, RDK, pose, N=72,
                                        label=frame_name)
        if j is not None:
            cfg = get_config_flags(robot, j)
            print(f"    Z-sweep: OK at {angle:.0f} deg  config={config_key(cfg)}")
            results[frame_name] = f"z-sweep OK at {angle:.0f}"
        else:
            print(f"    UNREACHABLE at any Z-rotation")
            results[frame_name] = "UNREACHABLE"

    # Summary
    print(f"\n{'='*50}")
    print("BIN GRIP REACHABILITY SUMMARY")
    print(f"{'='*50}")
    all_ok = True
    for name, status in results.items():
        ok = "UNREACHABLE" not in status and "not found" not in status
        marker = "OK" if ok else "FAIL"
        print(f"  [{marker}] {name}: {status}")
        if not ok:
            all_ok = False

    if all_ok:
        print("\nAll bin frames reachable — gripping should work.")
    else:
        print("\nSome frames unreachable — bin position may need adjustment.")


if __name__ == "__main__":
    main()
