"""
Minimal IK test for robert_end_checker.py.

Tests 2 points (Base_Right_0_grab and Front_0_grab) with verbose debug
output at every step. Validates that SolveIK returns proper 7-joint
solutions, not garbage.

Usage:
    python robert_checker_stuff/test_ik_mini.py --robodk-ip 172.23.208.1
"""

import sys
import os
import json
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME, ITEM_TYPE_TARGET
from robodk.robomath import Pose_2_TxyzRxyz

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_CONFIG = os.path.join(SCRIPT_DIR, "test_config.json")
TEST_RESULTS = os.path.join(SCRIPT_DIR, "test_ik_results.json")


def connect(ip=None):
    if ip:
        return Robolink(robodk_ip=ip)
    try:
        rdk = Robolink()
        rdk.Item("")
        return rdk
    except Exception:
        return Robolink(robodk_ip="172.23.208.1")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robodk-ip", default=None)
    args = ap.parse_args()

    RDK = connect(args.robodk_ip)

    # Find robot
    robot = None
    for name in ROBOT_NAMES:
        r = RDK.Item(name, ITEM_TYPE_ROBOT)
        if r.Valid():
            robot = r
            print(f"[OK] Robot: '{name}'")
            break
    if not robot:
        print("[FAIL] No robot found")
        sys.exit(1)

    # Print robot info
    joints_now = robot.Joints()
    try:
        jl = joints_now.list()
    except:
        jl = list(joints_now)
    print(f"[INFO] Current joints ({len(jl)}): {[round(j,1) for j in jl]}")

    # Set tool
    tool = RDK.Item("pickup", ITEM_TYPE_TOOL)
    if tool.Valid():
        robot.setTool(tool)
        print(f"[OK] Tool 'pickup' set")
    else:
        all_tools = [i.Name() for i in RDK.ItemList(ITEM_TYPE_TOOL)]
        print(f"[FAIL] Tool 'pickup' not found. Available: {all_tools}")
        sys.exit(1)

    # Get world frame
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    print(f"[INFO] WorldFrame valid: {world_frame.Valid()}")

    # Get current reference frame
    current_frame = robot.getLink(ITEM_TYPE_FRAME)
    print(f"[INFO] Current robot frame: '{current_frame.Name() if current_frame.Valid() else 'None'}'")

    # Test targets
    test_names = ["Base_Right_0_grab", "Front_0_grab"]
    results = {}

    for tname in test_names:
        print(f"\n{'='*60}")
        print(f"  TESTING: {tname}")
        print(f"{'='*60}")

        target = RDK.Item(tname, ITEM_TYPE_TARGET)
        if not target.Valid():
            print(f"  [FAIL] Target '{tname}' not found in RoboDK")
            results[tname] = {"error": "not found"}
            continue

        print(f"  [OK] Target found")

        # Get pose multiple ways
        pose_local = target.Pose()
        pose_abs = target.PoseAbs()
        parent = target.Parent()
        parent_name = parent.Name() if parent.Valid() else "None"
        print(f"  Parent: '{parent_name}'")

        xyzrpw_local = Pose_2_TxyzRxyz(pose_local)
        xyzrpw_abs = Pose_2_TxyzRxyz(pose_abs)
        print(f"  Pose (local):    XYZ=({xyzrpw_local[0]:.1f}, {xyzrpw_local[1]:.1f}, {xyzrpw_local[2]:.1f})")
        print(f"  Pose (absolute): XYZ=({xyzrpw_abs[0]:.1f}, {xyzrpw_abs[1]:.1f}, {xyzrpw_abs[2]:.1f})")

        # Try SolveIK WITHOUT setPoseFrame
        print(f"\n  --- Test A: SolveIK(PoseAbs) WITHOUT setPoseFrame ---")
        result_a = robot.SolveIK(pose_abs)
        try:
            joints_a = result_a.list()
        except:
            joints_a = list(result_a)
        print(f"  Result type: {type(result_a).__name__}, len={len(result_a)}")
        print(f"  .list() = {joints_a}")
        print(f"  len(.list()) = {len(joints_a)}")

        # Try SolveIK WITH setPoseFrame(WorldFrame)
        print(f"\n  --- Test B: SolveIK(PoseAbs) WITH setPoseFrame(WorldFrame) ---")
        robot.setPoseFrame(world_frame)
        result_b = robot.SolveIK(pose_abs)
        try:
            joints_b = result_b.list()
        except:
            joints_b = list(result_b)
        print(f"  Result type: {type(result_b).__name__}, len={len(result_b)}")
        print(f"  .list() = {joints_b}")
        print(f"  len(.list()) = {len(joints_b)}")

        # Restore frame
        if current_frame.Valid():
            robot.setPoseFrame(current_frame)

        # Try SolveIK with local pose
        print(f"\n  --- Test C: SolveIK(Pose local) WITHOUT setPoseFrame ---")
        result_c = robot.SolveIK(pose_local)
        try:
            joints_c = result_c.list()
        except:
            joints_c = list(result_c)
        print(f"  Result type: {type(result_c).__name__}, len={len(result_c)}")
        print(f"  .list() = {joints_c}")
        print(f"  len(.list()) = {len(joints_c)}")

        # Try SolveIK with parent.PoseAbs() * pose_local
        if parent.Valid():
            print(f"\n  --- Test D: SolveIK(parent.PoseAbs() * Pose) ---")
            composed = parent.PoseAbs() * pose_local
            xyzrpw_d = Pose_2_TxyzRxyz(composed)
            print(f"  Composed XYZ=({xyzrpw_d[0]:.1f}, {xyzrpw_d[1]:.1f}, {xyzrpw_d[2]:.1f})")
            result_d = robot.SolveIK(composed)
            try:
                joints_d = result_d.list()
            except:
                joints_d = list(result_d)
            print(f"  Result type: {type(result_d).__name__}, len={len(result_d)}")
            print(f"  .list() = {joints_d}")
            print(f"  len(.list()) = {len(joints_d)}")

        # Try with setPoseFrame AND local pose
        print(f"\n  --- Test E: SolveIK(Pose local) WITH setPoseFrame(WorldFrame) ---")
        robot.setPoseFrame(world_frame)
        result_e = robot.SolveIK(pose_local)
        try:
            joints_e = result_e.list()
        except:
            joints_e = list(result_e)
        print(f"  Result type: {type(result_e).__name__}, len={len(result_e)}")
        print(f"  .list() = {joints_e}")
        print(f"  len(.list()) = {len(joints_e)}")

        if current_frame.Valid():
            robot.setPoseFrame(current_frame)

        # Collect best result
        for label, jts in [("A", joints_a), ("B", joints_b), ("C", joints_c), ("E", joints_e)]:
            if len(jts) >= 6 and not all(abs(j) < 1e-9 for j in jts):
                print(f"\n  [BEST] Test {label} produced {len(jts)} joints: {[round(j,2) for j in jts]}")
                results[tname] = {"joints": jts, "test": label, "ok": True}
                break
        else:
            print(f"\n  [FAIL] No test produced valid joints")
            results[tname] = {"ok": False}

    # Save and validate
    with open(TEST_RESULTS, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n{'='*60}")
    print(f"  RESULTS")
    print(f"{'='*60}")

    all_pass = True
    for tname, r in results.items():
        ok = r.get("ok", False)
        if ok:
            jts = r["joints"]
            non_zero = any(abs(j) > 0.01 for j in jts)
            has_enough = len(jts) >= 6
            test = r.get("test", "?")
            print(f"  {tname}: PASS (test {test}, {len(jts)} joints, non-zero={non_zero})")
            if not non_zero or not has_enough:
                print(f"    [WARN] joints look suspicious: {jts}")
                all_pass = False
        else:
            print(f"  {tname}: FAIL")
            all_pass = False

    if all_pass:
        print(f"\n  ALL TESTS PASSED")
    else:
        print(f"\n  SOME TESTS FAILED")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
