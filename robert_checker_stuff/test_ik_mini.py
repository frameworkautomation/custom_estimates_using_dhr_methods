"""
Minimal IK test — no setPoseFrame, no state mutation.
Tests that SolveIK returns proper joints for known-reachable targets.

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

    # Robot info
    joints_now = robot.Joints()
    try:
        jl = joints_now.list()
    except:
        jl = list(joints_now)
    print(f"[INFO] Current joints ({len(jl)}): {[round(j,1) for j in jl]}")
    print(f"[INFO] Robot PoseAbs XYZ: {[round(v,1) for v in Pose_2_TxyzRxyz(robot.PoseAbs())[:3]]}")

    # Current frame
    current_frame = robot.getLink(ITEM_TYPE_FRAME)
    print(f"[INFO] Robot frame: '{current_frame.Name() if current_frame.Valid() else 'None'}'")

    # Set tool
    tool = RDK.Item("pickup", ITEM_TYPE_TOOL)
    if tool.Valid():
        robot.setTool(tool)
        print(f"[OK] Tool 'pickup' set")
    else:
        all_tools = [i.Name() for i in RDK.ItemList(ITEM_TYPE_TOOL)]
        print(f"[FAIL] Tool 'pickup' not found. Available: {all_tools}")
        sys.exit(1)

    # Test targets
    test_names = ["Base_Right_0_grab", "Front_0_grab"]
    results = {}

    for tname in test_names:
        print(f"\n{'='*60}")
        print(f"  TESTING: {tname}")
        print(f"{'='*60}")

        target = RDK.Item(tname, ITEM_TYPE_TARGET)
        if not target.Valid():
            print(f"  [FAIL] Target '{tname}' not found")
            results[tname] = {"ok": False, "error": "not found"}
            continue

        # Get absolute pose
        pose_abs = target.PoseAbs()
        xyz = Pose_2_TxyzRxyz(pose_abs)
        print(f"  PoseAbs XYZ: ({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f})")

        # Test 1: SolveIK with PoseAbs (no setPoseFrame)
        print(f"\n  --- SolveIK(PoseAbs) ---")
        sol = robot.SolveIK(pose_abs)
        print(f"  type={type(sol).__name__} repr={repr(sol)}")
        try:
            jl = sol.list()
        except:
            jl = list(sol)
        print(f"  .list() len={len(jl)} vals={jl}")

        # Test 2: SolveIK_All with PoseAbs
        print(f"\n  --- SolveIK_All(PoseAbs) ---")
        all_sol = robot.SolveIK_All(pose_abs)
        print(f"  type={type(all_sol).__name__} repr len={len(all_sol)}")
        try:
            import numpy as np
            arr = np.array(all_sol)
            print(f"  numpy shape: {arr.shape}")
            if arr.size > 0:
                # SolveIK_All returns (n_joints, n_solutions)
                if arr.ndim == 2 and arr.shape[0] <= 7:
                    sols = arr.T  # transpose to (n_solutions, n_joints)
                    print(f"  {sols.shape[0]} solution(s), {sols.shape[1]} joints each")
                    for i, s in enumerate(sols[:3]):
                        print(f"    sol[{i}]: {[round(v,2) for v in s]}")
                    results[tname] = {"ok": True, "joints": sols[0].tolist(), "method": "SolveIK_All"}
                else:
                    print(f"  Unexpected shape: {arr.shape}")
        except Exception as e:
            print(f"  numpy failed: {e}")
            # Try without numpy
            try:
                all_list = all_sol.list()
                print(f"  .list() len={len(all_list)} vals={all_list[:14]}")
            except:
                pass

        if tname not in results:
            if len(jl) >= 6:
                results[tname] = {"ok": True, "joints": jl, "method": "SolveIK"}
            else:
                results[tname] = {"ok": False, "error": "no valid joints"}

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
            method = r.get("method", "?")
            print(f"  {tname}: PASS ({method}, {len(jts)} joints, non-zero={non_zero})")
            if not non_zero:
                print(f"    [WARN] All joints zero!")
                all_pass = False
        else:
            print(f"  {tname}: FAIL ({r.get('error', '?')})")
            all_pass = False

    print(f"\n  {'ALL TESTS PASSED' if all_pass else 'SOME TESTS FAILED'}")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
