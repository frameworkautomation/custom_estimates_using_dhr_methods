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
from robodk.robomath import transl, invH, Mat, Pose_2_TxyzRxyz, rotx, roty, rotz

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


class ZAxisFreeSolver:
    """Rotation sweep around target Z axis to find IK solutions.

    Creates a parent frame 'DiscoveredWaypoints' in the RoboDK station on init.
    Each call to solve() deletes and recreates it (clean slate), then sweeps N
    rotations around the target's Z axis, attempting IK at each angle via
    _solve_ik_locked_j7.
    """

    def __init__(self, RDK):
        self.RDK = RDK
        self._create_frames()

    def _create_frames(self):
        """Create DiscoveredWaypoints parent frame and temp child frame."""
        station = self.RDK.ActiveStation()
        self.parent_frame = self.RDK.AddFrame("DiscoveredWaypoints", station)
        self.temp_frame = self.RDK.AddFrame("temp", self.parent_frame)

    def _reset(self):
        """Delete and recreate DiscoveredWaypoints (clean slate per solve)."""
        if self.parent_frame.Valid():
            self.parent_frame.Delete()
        self._create_frames()

    def solve(self, robot, RDK, target_pose, j7_target, N=72, label=""):
        """Sweep N rotations around target Z axis, return first IK that works.

        Returns (joints_list, ok).
        """
        self._reset()

        for i in range(N):
            angle_deg = 360.0 * i / N
            angle_rad = angle_deg * math.pi / 180.0

            # Rotate the target pose around its own Z axis
            rz_mat = rotz(angle_rad)
            rotated_pose = target_pose * rz_mat

            # Set temp frame pose to the rotated pose
            self.temp_frame.setPose(rotated_pose)

            # Attempt IK
            joints, ok = _solve_ik_locked_j7(robot, RDK, rotated_pose, j7_target)
            if not ok or len(joints) < 6:
                continue

            # FK verification
            robot.MoveJ(joints)
            achieved = robot.Pose()
            t = Pose_2_TxyzRxyz(rotated_pose)
            a = Pose_2_TxyzRxyz(achieved)
            fk_err = math.sqrt(sum((t[k] - a[k]) ** 2 for k in range(3)))
            robot.setJoints(HOME_SEED)

            if fk_err > 50.0:
                continue

            # Success — create a visual waypoint frame for inspection
            wp_name = f"discovered_waypoint_{label}" if label else f"discovered_waypoint_{i}"
            wp_frame = RDK.AddFrame(wp_name, self.parent_frame)
            wp_frame.setPose(rotated_pose)

            print(f"    [z_axis_free] Found solution at {angle_deg:.1f} deg (err={fk_err:.1f}mm)")
            return joints, True

        print(f"    [z_axis_free] No solution found after {N} angles")
        return [], False


def _solve_ik_locked_j7(robot, RDK, pose, j7_target, j7_weight=100):
    """Solve IK with j7 constrained using OptimAxes + MoveJ.

    j7_weight controls how strongly j7 is pinned:
      100 = hard lock, 50 = firm, 20 = moderate, 5 = soft preference
    """
    props = dict(_OPT_AXES_LOCKED)
    props["AbsJnt_7"] = j7_target
    props["AbsW_7"] = j7_weight
    robot.setParam("OptimAxes", props)

    robot.setJoints(HOME_SEED)
    try:
        robot.MoveJ(pose)
        raw = robot.Joints()
        try:
            joints = raw.list()
        except AttributeError:
            joints = list(raw)
        robot.setJoints(HOME_SEED)
        # Reject if j7 drifted too far from target
        if len(joints) >= 7 and abs(joints[6] - j7_target) > J7_TOL_MM:
            return [], False
        return joints, True
    except Exception:
        robot.setJoints(HOME_SEED)
        return [], False


def solve_point(robot, RDK, pose, track_cond, label, z_axis_free=False, z_solver=None):
    """Dispatch to the right IK solver based on special_track_conditions."""
    ctype = track_cond.get("type", "None")

    if z_axis_free:
        if ctype not in ("Locked_at_j7_0", "Locked_at_j7_pt"):
            raise ValueError(
                f"z_axis_free only supported with Locked_at_j7_0 or Locked_at_j7_pt, got '{ctype}'"
            )
        if z_solver is None:
            raise ValueError("z_axis_free=True but no ZAxisFreeSolver instance provided")
        j7_val = 0.0 if ctype == "Locked_at_j7_0" else float(track_cond["j7_value"])
        return z_solver.solve(robot, RDK, pose, j7_val, label=label)

    if ctype == "Locked_at_j7_0":
        return _solve_ik_locked_j7(robot, RDK, pose, 0.0)

    elif ctype == "Locked_at_j7_pt":
        j7_val = float(track_cond["j7_value"])
        return _solve_ik_locked_j7(robot, RDK, pose, j7_val)

    elif ctype == "Optimized_for_j7_at":
        j7_val = float(track_cond["j7_value"])
        # Try with decreasing j7 weights, FK verify each
        import math
        for w in [100, 50, 20, 5]:
            joints, ok = _solve_ik_locked_j7(robot, RDK, pose, j7_val, j7_weight=w)
            if ok and len(joints) >= 6:
                robot.MoveJ(joints)
                achieved = robot.Pose()
                t = Pose_2_TxyzRxyz(pose)
                a = Pose_2_TxyzRxyz(achieved)
                err = math.sqrt(sum((t[i] - a[i])**2 for i in range(3)))
                robot.setJoints(HOME_SEED)
                if err <= 50.0:
                    return joints, True
        # Last resort — completely free
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


def load_config(config_path=None):
    path = config_path or CONFIG_PATH
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser(description="Check reachability for end effectors from robert_end_checker_config.json")
    ap.add_argument("--robodk-ip", default=None, help="RoboDK IP (default: localhost then 172.23.208.1)")
    ap.add_argument("--config", default=None, help="Path to config JSON (default: robert_end_checker_config.json)")
    args = ap.parse_args()

    config = load_config(args.config)
    end_effectors = config.get("end_effectors", [])
    if not end_effectors:
        print("[ERROR] No end_effectors in config JSON.")
        return

    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)

    # Ensure WorldFrame exists (create if missing) and set it as robot frame.
    # This is required for MoveJ(pose) to interpret PoseAbs() correctly.
    # Both check_base_cone_reachability.py and moving_a_cone.py do this.
    from robodk.robomath import eye
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    if not world_frame.Valid():
        print("[INFO] Creating WorldFrame (not found in station)")
        station = RDK.ActiveStation()
        world_frame = RDK.AddFrame("WorldFrame", station)
        world_frame.setPose(eye(4))
    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)
    print(f"[INFO] Robot frame set to WorldFrame")

    # Instantiate ZAxisFreeSolver once for the whole run
    z_solver = ZAxisFreeSolver(RDK)

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

                is_z_free = pt.get("z_axis_free", False)
                joints, ok = solve_point(robot, RDK, pose, track_cond, name,
                                         z_axis_free=is_z_free, z_solver=z_solver)

                ctype = track_cond.get("type", "None")
                j7_info = ""
                if ctype == "Locked_at_j7_0":
                    j7_info = " j7=locked(0)"
                elif ctype == "Locked_at_j7_pt":
                    j7_info = f" j7=locked({track_cond['j7_value']})"
                elif ctype == "Optimized_for_j7_at":
                    j7_info = f" j7=opt({track_cond['j7_value']})"

                # FK verify: move to joints, read TCP in WorldFrame, measure error
                import math
                fk_err = None
                if ok and len(joints) >= 6:
                    robot.MoveJ(joints)
                    achieved = robot.Pose()  # TCP in WorldFrame (since setPoseFrame)
                    target_xyzrpw = Pose_2_TxyzRxyz(pose)
                    achieved_xyzrpw = Pose_2_TxyzRxyz(achieved)
                    fk_err = math.sqrt(sum((target_xyzrpw[i] - achieved_xyzrpw[i])**2 for i in range(3)))
                    if fk_err > 50.0:
                        ok = False  # reject — joints don't actually reach target
                    robot.setJoints(HOME_SEED)

                status = "OK" if ok else "FAIL"
                j7_actual = f" j7_actual={joints[6]:.1f}" if ok and len(joints) > 6 else ""
                err_str = f" err={fk_err:.1f}mm" if fk_err is not None else ""
                print(f"  [{name}] {status}{j7_info}{j7_actual}{err_str}")

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
