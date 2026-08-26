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

# Import proven z-axis-free solver from move_to_base_cone_grab.py
REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(REPO_ROOT, "robodk_code"))
from move_to_base_cone_grab import solve_ik_locked_j7 as _z_free_solve_locked_j7

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

def _solve_ik_free(robot, pose):
    """SolveIK with no j7 constraint. Returns (joints_list, ok)."""
    result = robot.SolveIK(pose)
    try:
        joints = result.list()
    except AttributeError:
        joints = list(result)
    if len(joints) >= 6:
        return joints, True
    return [], False


_OPT_AXES_LOCKED = {
    "AbsOn_7": 1, "AbsW_7": 100,
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
    "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
    "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
}


def _solve_ik_locked_j7(robot, RDK, pose, j7_target):
    """Solve IK with j7 locked using OptimAxes + MoveJ.

    Copied from check_base_cone_reachability.py solve_ik_static_j7.
    Wraps with Render(False) and restores joints after.
    """
    props = dict(_OPT_AXES_LOCKED)
    props["AbsJnt_7"] = j7_target
    robot.setParam("OptimAxes", props)
    robot.setJoints(HOME_SEED)
    RDK.Render(False)
    try:
        robot.MoveJ(pose)
        # Re-enable render BEFORE reading joints to ensure state is flushed
        RDK.Render(True)
        raw = robot.Joints()
        try:
            joints = raw.list()
        except AttributeError:
            joints = list(raw)
        j7_actual = joints[6] if len(joints) > 6 else 0.0
        robot.setJoints(HOME_SEED)
        if abs(j7_actual - j7_target) > J7_TOL_MM:
            return joints, False
        return joints, True
    except Exception:
        robot.setJoints(HOME_SEED)
        RDK.Render(True)
        return [], False


Z_FREE_STEPS = 72  # 5 degree resolution for z-axis-free sweep


def _solve_ik_z_axis_free_locked_j7(robot, pose, j7_target, label=""):
    """Solve IK with z-axis rotation free + j7 locked.

    Uses the proven solve_ik_locked_j7 from move_to_base_cone_grab.py which does:
    Z-rotation sweep + rail shift trick + FK verification + closest-to-home selection.
    """
    tool_item = robot.getLink(ITEM_TYPE_TOOL)
    tool_offset = tool_item.PoseTool() if tool_item.Valid() else Mat([
        [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    preferred_j6 = HOME_SEED[:6]

    best, best_dist, best_angle, n_reachable = _z_free_solve_locked_j7(
        robot, pose, tool_offset, j7_target, pose, preferred_j6, Z_FREE_STEPS
    )

    if best is None and not hasattr(_solve_ik_z_axis_free_locked_j7, '_debug_done'):
        _solve_ik_z_axis_free_locked_j7._debug_done = True
        xyz = Pose_2_TxyzRxyz(pose)
        tool_xyz = Pose_2_TxyzRxyz(tool_offset)
        print(f"    [Z-FREE DEBUG] {label} FAILED — n_reachable={n_reachable}")
        print(f"    [Z-FREE DEBUG] pose XYZ=({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f})")
        print(f"    [Z-FREE DEBUG] tool name='{tool_item.Name() if tool_item.Valid() else 'None'}'")
        print(f"    [Z-FREE DEBUG] tool offset XYZ=({tool_xyz[0]:.1f}, {tool_xyz[1]:.1f}, {tool_xyz[2]:.1f})")
        print(f"    [Z-FREE DEBUG] j7_target={j7_target} FK_TOL_MM={FK_TOL_MM}")

    if best is not None:
        return best, True
    return [], False


def solve_point(robot, RDK, pose, track_cond, label, z_axis_free=False):
    """Dispatch to the right IK solver based on special_track_conditions."""
    ctype = track_cond.get("type", "None")

    if z_axis_free:
        if ctype != "Locked_at_j7_0":
            raise RuntimeError(
                f"z_axis_free is only supported with Locked_at_j7_0, got '{ctype}' for '{label}'")
        return _solve_ik_z_axis_free_locked_j7(robot, pose, 0.0, label)

    if ctype == "Locked_at_j7_0":
        return _solve_ik_locked_j7(robot, RDK, pose, 0.0)

    elif ctype == "Locked_at_j7_pt":
        j7_val = float(track_cond["j7_value"])
        return _solve_ik_locked_j7(robot, RDK, pose, j7_val)

    elif ctype == "Optimized_for_j7_at":
        # Use SolveIK (picks correct arm config) — OptimAxes gives rear config
        return _solve_ik_free(robot, pose)

    else:  # "None"
        return _solve_ik_free(robot, pose)


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

    # Delete old results so stale data can't be used by shower
    for f in [RESULTS_PATH, REPORT_PATH]:
        if os.path.exists(f):
            os.remove(f)
            print(f"[INFO] Deleted old {os.path.basename(f)}")

    config = load_config()
    end_effectors = config.get("end_effectors", [])
    if not end_effectors:
        print("[ERROR] No end_effectors in config JSON.")
        return

    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)

    # Do NOT call setPoseFrame — it can break the robot-rail connection.
    # SolveIK works with the robot's current frame setup.
    # Just get the pose in absolute (world) coordinates via PoseAbs().

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

                z_free = pt.get("z_axis_free", False)
                joints, ok = solve_point(robot, RDK, pose, track_cond, name, z_axis_free=z_free)

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
        pass

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
