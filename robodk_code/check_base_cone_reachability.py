"""
Reachability checker for all base_cone_grab_* and cone_grab_* targets.

Runs the IK computation from moving_a_cone_bender_request (OptimAxes Algorithm 3
DLS) for both base cones (j7 locked) and destination cones (j7 free), prints a
summary table, then offers an interactive mode to examine individual positions.

Usage:
    python robodk_code/check_base_cone_reachability.py
    python robodk_code/check_base_cone_reachability.py --tool pickup_closed
    python robodk_code/check_base_cone_reachability.py --recompute
"""

import sys
import os
import json
import re
import datetime
import argparse
import math

sys.path.append("C:/RoboDK/Python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET, ITEM_TYPE_FRAME
from robodk.robomath import transl, invH, rotz, Pose_2_TxyzRxyz, eye

from test_reach_base_cone import fmt_joints

pi = 3.141592653589793

# ── CONFIG ────────────────────────────────────────────────────────────────────
ROBOT_NAME         = "Fanuc R2000iC 125L"
TOOL_NAME          = "pickup_open"
APPROACH_OFFSET_MM = 200.0
J7_LOCKED          = 0.0
HOME_SEED          = [0.0] * 7
IK_SOLUTIONS_DIR   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ik_solutions")
ROBODK_OUTPUT_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output")

J7_TOL_MM = 10.0

# OptimAxes — j7 constrained (base cones)
OPT_AXES_STATIC_J7 = {
    "AbsJnt_7": 0,
    "AbsOn_7":  1,
    "AbsW_7":   100,
    "Algorithm": 3,
    "MaxIter":  500,
    "Tol":      0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
    "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
    "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
}

# OptimAxes — j7 free (destination cones)
OPT_AXES_FREE_J7 = {
    "Algorithm": 3,
    "MaxIter":  500,
    "Tol":      0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
    "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 0,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
    "RelW_5": 50, "RelW_6": 50,
}
# ─────────────────────────────────────────────────────────────────────────────


def connect():
    try:
        rdk = Robolink()
        rdk.Item("")
        print("[INFO] Connected to RoboDK on localhost")
        return rdk
    except Exception:
        print("[INFO] localhost failed, trying 172.23.208.1 ...")
        return Robolink(robodk_ip="172.23.208.1")


def _pose_xyz(pose):
    xyzrpw = Pose_2_TxyzRxyz(pose)
    return xyzrpw[0], xyzrpw[1], xyzrpw[2]


def make_approach_pose(grab_pose, offset_mm):
    return grab_pose * transl(0, 0, offset_mm)


def solve_ik_static_j7(robot, pose, label):
    """Solve IK with j7 constrained to J7_LOCKED. Returns (joints, converged)."""
    props = dict(OPT_AXES_STATIC_J7)
    props["AbsJnt_7"] = J7_LOCKED
    robot.setParam("OptimAxes", props)
    robot.setJoints(HOME_SEED)
    try:
        robot.MoveJ(pose)
        raw = robot.Joints()
        try:
            joints = raw.list()
        except AttributeError:
            joints = list(raw)
        j7_actual = joints[6]
        if abs(j7_actual - J7_LOCKED) > J7_TOL_MM:
            robot.setJoints(HOME_SEED)
            return [0.0] * 7, False
        robot.setJoints(HOME_SEED)
        return joints, True
    except Exception:
        robot.setJoints(HOME_SEED)
        return [0.0] * 7, False


def solve_ik_free_j7(robot, pose, label, seed=None):
    """Solve IK with j7 free. Returns (joints, converged)."""
    if seed is not None:
        params = dict(OPT_AXES_FREE_J7)
        params["AbsOn_7"]  = 1
        params["AbsJnt_7"] = float(seed[6])
        params["AbsW_7"]   = 100
    else:
        params = OPT_AXES_FREE_J7
    robot.setParam("OptimAxes", params)
    robot.setJoints(seed if seed is not None else HOME_SEED)
    try:
        robot.MoveJ(pose)
        raw = robot.Joints()
        try:
            joints = raw.list()
        except AttributeError:
            joints = list(raw)
        robot.setJoints(seed if seed is not None else HOME_SEED)
        return joints, True
    except Exception:
        robot.setJoints(seed if seed is not None else HOME_SEED)
        return [0.0] * 7, False


def _nat_key(t):
    parts = re.split(r'(\d+)', t.Name())
    return [int(p) if p.isdigit() else p.lower() for p in parts]


def _targets_under_waypoint_frame(RDK):
    parent = RDK.Item("WaypointTargets", ITEM_TYPE_FRAME)
    if not parent.Valid():
        return []
    return [t for t in parent.Childs() if t.Type() == ITEM_TYPE_TARGET]


def find_base_cones(RDK):
    """Return sorted base_cone_grab_* targets (excluding _approach)."""
    def _is_base_grab(t):
        return t.Name().startswith("base_cone_grab_") and not t.Name().endswith("_approach")

    targets = [t for t in RDK.ItemList(ITEM_TYPE_TARGET) if _is_base_grab(t)]
    if not targets:
        targets = [t for t in _targets_under_waypoint_frame(RDK) if _is_base_grab(t)]
    return sorted(targets, key=_nat_key)


def find_destination_cones(RDK):
    """Return sorted cone_grab_* targets (excluding _approach)."""
    def _is_dest(t):
        return t.Name().startswith("cone_grab_") and not t.Name().endswith("_approach")

    targets = [t for t in RDK.ItemList(ITEM_TYPE_TARGET) if _is_dest(t)]
    if not targets:
        targets = [t for t in _targets_under_waypoint_frame(RDK) if _is_dest(t)]
    return sorted(targets, key=_nat_key)


def get_last_human_target_joints(RDK):
    """Return joints of last target under human_targets frame (for dest IK seed)."""
    frame = RDK.Item("human_targets", ITEM_TYPE_FRAME)
    if not frame.Valid():
        return None
    targets = sorted(
        [t for t in frame.Childs() if t.Type() == ITEM_TYPE_TARGET],
        key=_nat_key,
    )
    if not targets:
        return None
    last = targets[-1]
    try:
        raw = last.Joints()
        joints = raw.list() if hasattr(raw, "list") else list(raw)
        if len(joints) >= 6:
            return joints
    except Exception:
        pass
    return None


def compute_base_cone_ik(RDK, robot, cone_targets):
    """Compute IK for base cones (j7 locked). Returns dict of results."""
    current_tool = robot.getLink(ITEM_TYPE_TOOL)
    print(f"\n[IK] Tool: '{current_tool.Name() if current_tool.Valid() else 'None'}'")
    print(f"[IK] j7 locked at {J7_LOCKED} mm, approach offset {APPROACH_OFFSET_MM} mm")
    print(f"[IK] Solver: OptimAxes Algorithm 3 (DLS)")

    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)
    RDK.Render(False)

    results = {}
    try:
        for target in cone_targets:
            name = target.Name()
            grab_pose = target.PoseAbs()
            app_pose = make_approach_pose(grab_pose, APPROACH_OFFSET_MM)

            grab_j, grab_ok = solve_ik_static_j7(robot, grab_pose, f"{name} grab")
            app_j,  app_ok  = solve_ik_static_j7(robot, app_pose,  f"{name} approach")

            results[name] = {
                "grab_ok": grab_ok, "grab_joints": [float(v) for v in grab_j],
                "app_ok":  app_ok,  "app_joints":  [float(v) for v in app_j],
            }
    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)
        robot.setJoints(HOME_SEED)
        RDK.Render(True)

    return results


def compute_dest_cone_ik(RDK, robot, dest_cones, seed=None):
    """Compute IK for destination cones (j7 free). Returns dict of results."""
    current_tool = robot.getLink(ITEM_TYPE_TOOL)
    print(f"\n[IK] Tool: '{current_tool.Name() if current_tool.Valid() else 'None'}'")
    if seed is not None:
        print(f"[IK] Seeding from last human_target: {[round(v,1) for v in seed]}")
    print(f"[IK] j7 free, approach offset {APPROACH_OFFSET_MM} mm")

    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)
    RDK.Render(False)

    results = {}
    try:
        for target in dest_cones:
            name = target.Name()
            grab_pose = target.PoseAbs()
            app_pose = make_approach_pose(grab_pose, APPROACH_OFFSET_MM)

            grab_j, grab_ok = solve_ik_free_j7(robot, grab_pose, f"{name} grab", seed=seed)
            app_j,  app_ok  = solve_ik_free_j7(robot, app_pose,  f"{name} approach", seed=seed)

            results[name] = {
                "grab_ok": grab_ok, "grab_joints": [float(v) for v in grab_j],
                "app_ok":  app_ok,  "app_joints":  [float(v) for v in app_j],
            }
    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)
        robot.setJoints(HOME_SEED)
        RDK.Render(True)

    return results


def print_table(title, targets, ik_map, show_j7=False):
    """Print a summary table of IK results."""
    print(f"\n{'=' * 70}")
    print(f"  {title}")
    print(f"{'=' * 70}")

    if show_j7:
        print(f"  {'#':<4} {'Name':<30} {'Grab':>8} {'Approach':>9} {'j7(grab)':>10}")
        print("  " + "-" * 65)
    else:
        print(f"  {'#':<4} {'Name':<30} {'Grab':>8} {'Approach':>9}")
        print("  " + "-" * 55)

    for i, t in enumerate(targets):
        name = t.Name()
        r = ik_map.get(name, {})
        gs  = "OK" if r.get("grab_ok") else "FAIL"
        as_ = "OK" if r.get("app_ok")  else "FAIL"
        if show_j7 and r.get("grab_ok"):
            j7_val = r["grab_joints"][6]
            print(f"  {i:<4} {name:<30} {gs:>8} {as_:>9} {j7_val:>9.1f}mm")
        else:
            print(f"  {i:<4} {name:<30} {gs:>8} {as_:>9}")

    n_both = sum(1 for t in targets
                 if ik_map.get(t.Name(), {}).get("grab_ok")
                 and ik_map.get(t.Name(), {}).get("app_ok"))
    print(f"\n  {n_both}/{len(targets)} fully reachable (grab + approach)")


def interactive_examine(RDK, robot, all_entries, all_ik):
    """Let user pick positions to examine by number. Moves robot and pauses."""
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    print(f"\n{'=' * 70}")
    print("  INTERACTIVE EXAMINATION")
    print(f"{'=' * 70}")
    print("  Enter a number to move the robot to that position.")
    print("  Type 'q' to quit.\n")

    # Build a flat numbered list of all positions (grab + approach for each)
    menu = []
    for target, section in all_entries:
        name = target.Name()
        r = all_ik.get(name, {})
        if r.get("grab_ok"):
            menu.append({
                "label": f"{name} (grab)",
                "joints": r["grab_joints"],
                "pose": target.PoseAbs(),
                "section": section,
            })
        if r.get("app_ok"):
            menu.append({
                "label": f"{name} (approach)",
                "joints": r["app_joints"],
                "pose": make_approach_pose(target.PoseAbs(), APPROACH_OFFSET_MM),
                "section": section,
            })

    if not menu:
        print("  No reachable positions to examine.")
        return

    for i, entry in enumerate(menu):
        print(f"  ({i:>3}) {entry['label']}")

    print()
    try:
        while True:
            ans = input("  Position number (or 'q' to quit): ").strip()
            if ans.lower() in ('q', 'quit', 'exit', ''):
                break
            try:
                idx = int(ans)
            except ValueError:
                print("  Enter a number or 'q'.")
                continue
            if not (0 <= idx < len(menu)):
                print(f"  Out of range (0–{len(menu)-1}).")
                continue

            entry = menu[idx]
            joints = entry["joints"]
            print(f"\n  Moving to: {entry['label']}")
            print(f"  Joints: {fmt_joints(joints)}")

            px, py, pz = _pose_xyz(entry["pose"])
            print(f"  Target XYZ: ({px:.1f}, {py:.1f}, {pz:.1f}) mm")

            try:
                robot.MoveJ(joints)
                achieved = robot.Pose()
                ax, ay, az = _pose_xyz(achieved)
                err = math.sqrt((px-ax)**2 + (py-ay)**2 + (pz-az)**2)
                print(f"  Achieved XYZ: ({ax:.1f}, {ay:.1f}, {az:.1f}) mm  (err={err:.2f}mm)")
            except Exception as e:
                print(f"  [ERROR] MoveJ failed: {e}")

            input("  Press Enter to return to home ...")
            robot.setJoints(HOME_SEED)
    finally:
        robot.setJoints(HOME_SEED)
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)


def save_results(base_ik, dest_ik, tool_name):
    """Save combined results to ik_solutions/."""
    os.makedirs(IK_SOLUTIONS_DIR, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(IK_SOLUTIONS_DIR, f"reachability_check_{timestamp}.json")
    with open(out_path, "w") as f:
        json.dump({
            "generated": timestamp,
            "robot": ROBOT_NAME,
            "tool": tool_name,
            "j7_locked": J7_LOCKED,
            "approach_offset_mm": APPROACH_OFFSET_MM,
            "base_cones": base_ik,
            "dest_cones": dest_ik,
        }, f, indent=2)
    print(f"\n[INFO] Results saved to: {out_path}")


def main():
    ap = argparse.ArgumentParser(description="Check reachability of base and destination cone targets.")
    ap.add_argument("--tool", default=TOOL_NAME,
                    help=f"Tool for base cone IK (default: {TOOL_NAME})")
    ap.add_argument("--dest-tool", default="pickup_closed",
                    help="Tool for destination cone IK (default: pickup_closed)")
    ap.add_argument("--recompute", action="store_true",
                    help="Force recompute (ignore any cached IK solutions)")
    ap.add_argument("--no-examine", action="store_true",
                    help="Skip interactive examination after printing the table")
    ap.add_argument("--base-only", action="store_true",
                    help="Only check base cones, skip destination cones")
    ap.add_argument("--dest-only", action="store_true",
                    help="Only check destination cones, skip base cones")
    args = ap.parse_args()

    RDK = connect()

    robot = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError(f"Robot '{ROBOT_NAME}' not found.")

    # ── Base cones (j7 locked) ────────────────────────────────────────────────
    base_cones = []
    base_ik = {}
    if not args.dest_only:
        tool = RDK.Item(args.tool, ITEM_TYPE_TOOL)
        if tool.Valid():
            robot.setTool(tool)
            print(f"[INFO] Tool set to '{args.tool}'")
        else:
            all_tools = [i.Name() for i in RDK.ItemList(ITEM_TYPE_TOOL)]
            print(f"[WARN] Tool '{args.tool}' not found. Available: {all_tools}")

        base_cones = find_base_cones(RDK)
        if base_cones:
            print(f"\nFound {len(base_cones)} base cone target(s).")
            base_ik = compute_base_cone_ik(RDK, robot, base_cones)
            print_table("BASE CONES (j7 locked at {:.0f}mm)".format(J7_LOCKED),
                        base_cones, base_ik)
        else:
            print("\n[WARN] No base_cone_grab_* targets found.")

    # ── Destination cones (j7 free) ───────────────────────────────────────────
    dest_cones = []
    dest_ik = {}
    if not args.base_only:
        dest_tool = RDK.Item(args.dest_tool, ITEM_TYPE_TOOL)
        if dest_tool.Valid():
            robot.setTool(dest_tool)
            print(f"\n[INFO] Tool switched to '{args.dest_tool}' for dest cone IK")
        else:
            print(f"\n[WARN] Dest tool '{args.dest_tool}' not found — using current tool.")

        dest_cones = find_destination_cones(RDK)
        if dest_cones:
            print(f"Found {len(dest_cones)} destination cone target(s).")
            dest_seed = get_last_human_target_joints(RDK)
            dest_ik = compute_dest_cone_ik(RDK, robot, dest_cones, seed=dest_seed)
            print_table("DESTINATION CONES (j7 free)", dest_cones, dest_ik, show_j7=True)
        else:
            print("[WARN] No cone_grab_* targets found.")

    # ── Save results ──────────────────────────────────────────────────────────
    if base_ik or dest_ik:
        save_results(base_ik, dest_ik, args.tool)

    # ── Interactive examination ───────────────────────────────────────────────
    if not args.no_examine and (base_ik or dest_ik):
        all_entries = []
        for t in base_cones:
            all_entries.append((t, "base"))
        for t in dest_cones:
            all_entries.append((t, "dest"))

        # Merge IK maps
        combined_ik = {}
        combined_ik.update(base_ik)
        combined_ik.update(dest_ik)

        interactive_examine(RDK, robot, all_entries, combined_ik)


if __name__ == "__main__":
    main()
