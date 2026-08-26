"""
Reachability checker for robert_end_checker_config.json.

Reads end effectors and their points from the config JSON, connects to RoboDK,
solves IK for each point respecting special_track_conditions, and caches
results to robert_checker_stuff/ik_results.json.

Usage:
    python robert_checker_stuff/robert_end_checker.py
    python robert_checker_stuff/robert_end_checker.py --robodk-ip 172.23.208.1
"""

import sys
import os
import json
import math
import argparse
import datetime

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME, ITEM_TYPE_TARGET
from robodk.robomath import transl, invH, Mat, Pose_2_TxyzRxyz

# ── CONFIG ────────────────────────────────────────────────────────────────────
ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]
HOME_SEED   = [0.0] * 7
J7_TOL_MM   = 10.0

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, "robert_end_checker_config.json")
RESULTS_PATH = os.path.join(SCRIPT_DIR, "ik_results.json")
REPORT_PATH  = os.path.join(SCRIPT_DIR, "reachability_report.txt")

NUM_JOINTS = 7  # 6 arm + 1 rail


# ── CONNECT ──────────────────────────────────────────────────────────────────

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
            print(f"[INFO] Found robot: '{name}'")
            return r
    raise RuntimeError(f"Robot not found. Tried: {ROBOT_NAMES}")


def fmt_joints(joints):
    return "[" + ", ".join(f"{j:.2f}" for j in joints) + "]"


# ── IK SOLVERS ───────────────────────────────────────────────────────────────

def _solve_ik(robot, pose, seed):
    """Call robot.SolveIK with a seed. Returns (joints_list, ok).

    SolveIK returns an empty matrix (0 rows) on failure, not all-zeros.
    A valid solution with some joints at zero is still valid.
    """
    sol = robot.SolveIK(pose, seed)
    try:
        joints = sol.list()
    except AttributeError:
        joints = list(sol)
    # Empty or zero-length = no solution found
    if len(joints) == 0:
        return list(seed), False
    return joints, True


def solve_point(robot, pose, track_cond, label):
    """Solve IK using SolveIK with appropriate j7 seed based on special_track_conditions.

    Does NOT use OptimAxes or MoveJ — avoids messing up robot-rail connection.
    """
    ctype = track_cond.get("type", "None")
    seed = list(HOME_SEED)

    if ctype == "Locked_at_j7_0":
        seed[6] = 0.0
        joints, ok = _solve_ik(robot, pose, seed)
        # Verify j7 is actually locked
        if ok and len(joints) > 6 and abs(joints[6] - 0.0) > J7_TOL_MM:
            return joints, False
        return joints, ok

    elif ctype == "Locked_at_j7_pt":
        j7_val = float(track_cond["j7_value"])
        seed[6] = j7_val
        joints, ok = _solve_ik(robot, pose, seed)
        if ok and len(joints) > 6 and abs(joints[6] - j7_val) > J7_TOL_MM:
            return joints, False
        return joints, ok

    elif ctype == "Optimized_for_j7_at":
        j7_val = float(track_cond["j7_value"])
        seed[6] = j7_val
        joints, ok = _solve_ik(robot, pose, seed)
        # Soft preference — don't fail if j7 drifts, just return what we got
        return joints, ok

    else:  # "None" or unknown
        joints, ok = _solve_ik(robot, pose, seed)
        return joints, ok


# ── MAIN ─────────────────────────────────────────────────────────────────────

def print_report(all_results, robot_name, timestamp):
    """Build a human-readable report string, print it, and return it."""
    lines = []
    lines.append("=" * 70)
    lines.append(f"  REACHABILITY REPORT")
    lines.append(f"  Robot: {robot_name}")
    lines.append(f"  Generated: {timestamp}")
    lines.append("=" * 70)

    for ee_name, points in all_results.items():
        n_ok = sum(1 for r in points.values() if r["reachable"])
        lines.append(f"\n  End Effector: {ee_name}  ({n_ok}/{len(points)} reachable)")
        lines.append(f"  {'#':<4} {'Name':<40} {'Status':>6} {'j7 constraint':<25} {'j7 actual':>10}")
        lines.append("  " + "-" * 85)

        for i, (name, r) in enumerate(points.items()):
            status = "OK" if r["reachable"] else "FAIL"
            tc = r.get("special_track_conditions", {})
            ctype = tc.get("type", "None")
            if ctype == "Locked_at_j7_0":
                j7_con = "locked(0)"
            elif ctype == "Locked_at_j7_pt":
                j7_con = f"locked({tc.get('j7_value', '?')})"
            elif ctype == "Optimized_for_j7_at":
                j7_con = f"opt({tc.get('j7_value', '?')})"
            else:
                j7_con = "free"

            j7_act = f"{r['joints'][6]:.1f}" if r["reachable"] and len(r.get('joints', [])) > 6 else "-"
            lines.append(f"  {i:<4} {name:<40} {status:>6} {j7_con:<25} {j7_act:>10}")

        # List failures separately
        failures = [name for name, r in points.items() if not r["reachable"]]
        if failures:
            lines.append(f"\n  UNREACHABLE ({len(failures)}):")
            for name in failures:
                err = points[name].get("error", "IK failed")
                lines.append(f"    - {name}: {err}")

    lines.append("\n" + "=" * 70)
    report = "\n".join(lines)
    print(report)
    return report


def load_config():
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser(description="Check reachability for end effectors from robert_end_checker_config.json")
    ap.add_argument("--robodk-ip", default=None, help="RoboDK IP (default: localhost then 172.23.208.1)")
    args = ap.parse_args()

    config = load_config()
    end_effectors = config.get("end_effectors", [])
    if not end_effectors:
        print("[ERROR] No end_effectors in config JSON.")
        return

    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)

    # Set robot frame to WorldFrame so SolveIK works in world coordinates
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    all_results = {}

    try:
        for ee in end_effectors:
            ee_name = ee["end_effector_name"]
            points = ee.get("paths_and_points_to_check", [])

            # Try to set tool in RoboDK
            tool = RDK.Item(ee_name, ITEM_TYPE_TOOL)
            if tool.Valid():
                robot.setTool(tool)
                print(f"\n[EE] '{ee_name}' — tool set, {len(points)} point(s)")
            else:
                all_tools = [i.Name() for i in RDK.ItemList(ITEM_TYPE_TOOL)]
                raise RuntimeError(f"Tool '{ee_name}' not found in RoboDK. Available tools: {all_tools}")

            ee_results = {}

            for pt in points:
                if pt["type"] != "point":
                    print(f"  SKIP {pt['name']} (type={pt['type']}, not implemented)")
                    continue

                name = pt["name"]
                name_path = pt["name_path"]
                track_cond = pt.get("special_track_conditions", {"type": "None"})

                # Find the target in RoboDK by name_path
                # name_path is like "WaypointTargets/F0_grab" — use the last segment
                target_name = name_path.split("/")[-1]
                target = RDK.Item(target_name, ITEM_TYPE_TARGET)
                if not target.Valid():
                    print(f"  [{name}] TARGET '{target_name}' NOT FOUND — SKIP")
                    ee_results[name] = {
                        "reachable": False,
                        "error": f"Target '{target_name}' not found in RoboDK",
                        "joints": [0.0] * 7,
                        "special_track_conditions": track_cond,
                    }
                    continue

                # Get target's absolute pose (world space)
                pose = target.PoseAbs()

                joints, ok = solve_point(robot, pose, track_cond, name)

                ctype = track_cond.get("type", "None")
                j7_info = ""
                if ctype == "Locked_at_j7_0":
                    j7_info = " j7=locked(0)"
                elif ctype == "Locked_at_j7_pt":
                    j7_info = f" j7=locked({track_cond['j7_value']})"
                elif ctype == "Optimized_for_j7_at":
                    j7_info = f" j7=opt({track_cond['j7_value']})"

                status = "OK" if ok else "FAIL"
                j7_actual = f" j7_actual={joints[6]:.1f}" if ok and len(joints) > 6 else ""
                print(f"  [{name}] {status}{j7_info}{j7_actual}")

                ee_results[name] = {
                    "reachable": ok,
                    "joints": [float(v) for v in joints],
                    "special_track_conditions": track_cond,
                }

            # Summary
            n_ok = sum(1 for r in ee_results.values() if r["reachable"])
            print(f"  --- {ee_name}: {n_ok}/{len(ee_results)} reachable ---")
            all_results[ee_name] = ee_results

    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)

    # Save results
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output = {
        "generated": timestamp,
        "robot": robot.Name(),
        "end_effectors": all_results,
    }
    with open(RESULTS_PATH, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)
    print(f"\n[OK] Results saved to {RESULTS_PATH}")

    # Print and save report
    report = print_report(all_results, robot.Name(), timestamp)
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        f.write(report)
    print(f"[OK] Report saved to {REPORT_PATH}")


if __name__ == "__main__":
    main()
