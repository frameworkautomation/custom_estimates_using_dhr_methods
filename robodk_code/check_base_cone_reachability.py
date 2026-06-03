"""
Reachability checker for all base_cone_grab_* targets.

For each cone and each configured pose (grab, approach, …):
  1. Tests the native orientation with OptimAxes (j7 locked at J7_LOCKED)
  2. Sweeps ORIENT_SWEEP_STEPS orientations around the pose's local Z-axis
  3. Adds RoboDK reference frames for visualization
  4. Prints a reachability grid (cones × angles) per pose type
  5. Saves a human-readable summary txt and a machine-readable JSON to ik_solutions/

To add a new pose type, append an entry to POSE_CONFIGS:
    ("label", lambda grab_pose: <derived pose Mat>)
The label appears in the output and filenames.

J7_LOCKED: rail position held fixed during all IK solves.
ORIENT_SWEEP_STEPS: number of orientations to test (360 / steps = degrees each).
"""

import sys
import os
import json
import datetime
import argparse

sys.path.append("C:/RoboDK/Python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET, ITEM_TYPE_FRAME
from robodk.robomath import eye, transl, rotz

from test_reach_base_cone import pos_and_z, fmt_joints

pi = 3.141592653589793

# ── CONFIG ────────────────────────────────────────────────────────────────────
APPROACH_OFFSET_MM = 200.0    # offset from grab point along grab Z-axis
J7_LOCKED          = 0.0      # rail position held fixed during all solves (mm)
HOME_SEED          = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
VIZ_GROUP_NAME     = "ReachabilityCheck"
ROBOT_NAME         = "Fanuc R2000iC 125L"
TOOL_NAME          = "pickup_closed"
IK_SOLUTIONS_DIR   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ik_solutions")
ORIENT_SWEEP_STEPS = 12       # 360 / 12 = 30° per step

# Poses to check for each cone.  Add more entries here as needed.
# Each entry: (label, function(grab_pose) -> pose Mat)
POSE_CONFIGS = [
    ("grab",     lambda gp: gp),
    ("approach", lambda gp: gp * transl(0, 0, APPROACH_OFFSET_MM)),
]

# OptimAxes parameters — mirrors DHR's OptimizationKinematicsModel.
# Algorithm 3 = damped least squares (numerical).  RoboDK's solver natively
# respects all coupled joint limits (e.g. the R2000iC J2/J3 interference zone).
OPT_AXES_STATIC_J7 = {
    "AbsJnt_7": 0,    # overridden per call with J7_LOCKED
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

J7_TOL_MM = 10.0  # j7 must stay within this of J7_LOCKED to count as SUCCESS
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


def solve_pose(robot, pose, j7_locked, label):
    """Solve IK for one pose via OptimAxes.  Reports FAIL if j7 drifts.

    Returns (joints, pos_err, ok).
    """
    props = dict(OPT_AXES_STATIC_J7)
    props["AbsJnt_7"] = j7_locked
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
        if abs(j7_actual - j7_locked) > J7_TOL_MM:
            robot.setJoints(HOME_SEED)
            print(f"    [FAIL   ] {label:35s}  j7={j7_actual:.0f}mm (drifted — unreachable with rail fixed)")
            return [0.0] * 7, 999.0, False

        achieved = robot.Pose()
        tp = [pose[0,3], pose[1,3], pose[2,3]]
        ap = [achieved[0,3], achieved[1,3], achieved[2,3]]
        pos_err = (sum((t-a)**2 for t,a in zip(tp, ap)))**0.5
        print(f"    [SUCCESS] {label:35s}  pos_err={pos_err:.2f}mm  j7={j7_actual:.1f}mm")
        print(f"           target  XYZ: ({tp[0]:.1f}, {tp[1]:.1f}, {tp[2]:.1f})")
        print(f"           achieved XYZ: ({ap[0]:.1f}, {ap[1]:.1f}, {ap[2]:.1f})")
        print(f"           joints: {fmt_joints(joints)}")
        robot.setJoints(HOME_SEED)
        return joints, pos_err, True
    except Exception as e:
        robot.setJoints(HOME_SEED)
        print(f"    [FAIL   ] {label:35s}  ({e})")
        return [0.0] * 7, 999.0, False


def sweep_orientations(robot, pose, j7_locked, n_steps):
    """Rotate pose around its local Z-axis in n_steps equal increments and
    test each with OptimAxes + j7 drift check.

    Returns:
      sweep: list of (angle_deg, ok, detail_str, joints_or_None)
      best_joints: joints from first successful orientation, or None
    """
    step_deg = 360.0 / n_steps
    props = dict(OPT_AXES_STATIC_J7)
    props["AbsJnt_7"] = j7_locked

    sweep = []
    best_joints = None

    for i in range(n_steps):
        angle_deg = i * step_deg
        rotated   = pose * rotz(angle_deg * pi / 180.0)

        robot.setParam("OptimAxes", props)
        robot.setJoints(HOME_SEED)
        try:
            robot.MoveJ(rotated)
            raw = robot.Joints()
            try:
                joints = raw.list()
            except AttributeError:
                joints = list(raw)

            j7_actual = joints[6]
            if abs(j7_actual - j7_locked) > J7_TOL_MM:
                sweep.append((angle_deg, False, f"j7={j7_actual:.0f}mm (drifted)", None))
            else:
                achieved = robot.Pose()
                tp    = [pose[0,3], pose[1,3], pose[2,3]]
                ap    = [achieved[0,3], achieved[1,3], achieved[2,3]]
                pos_err = (sum((t-a)**2 for t,a in zip(tp, ap)))**0.5
                sweep.append((angle_deg, True, f"j7={j7_actual:.1f}mm  pos_err={pos_err:.2f}mm", joints))
                if best_joints is None:
                    best_joints = joints
        except Exception as e:
            sweep.append((angle_deg, False, f"exception ({e})", None))

        robot.setJoints(HOME_SEED)

    return sweep, best_joints


def add_frame(RDK, name, pose, parent):
    existing = RDK.Item(name, ITEM_TYPE_FRAME)
    if existing.Valid():
        existing.Delete()
    frame = RDK.AddFrame(name, parent)
    frame.setPose(pose)
    return frame


def print_grid(results, pose_label, angles, name_w):
    """Print one orientation sweep grid for a given pose label."""
    col_w = 5
    print(f"\nPOSE: {pose_label}")
    header = " " * (name_w + 4)
    for a in angles:
        header += f"{int(a):>{col_w}}°"
    print(header)
    print("  " + "-" * (name_w + 2 + len(angles) * (col_w + 1)))
    for r in results:
        pd = r["poses"][pose_label]
        row = f"  {r['name']:<{name_w}}  "
        for a in angles:
            cell = "  OK" if a in pd["reachable_angles"] else "   ."
            row += f"{cell:>{col_w}} "
        print(row)


def save_summary_txt(results, angles, timestamp, out_path, tool_name=TOOL_NAME):
    """Write a human-readable summary txt to out_path."""
    step_deg = angles[1] - angles[0] if len(angles) > 1 else 360.0
    name_w   = max(len(r["name"]) for r in results)
    col_w    = 5

    lines = []
    lines.append("BASE CONE REACHABILITY SUMMARY")
    lines.append(f"Generated    : {timestamp}")
    lines.append(f"Robot        : {ROBOT_NAME}")
    lines.append(f"Tool         : {tool_name}")
    lines.append(f"j7 locked    : {J7_LOCKED} mm")
    lines.append(f"Sweep steps  : {ORIENT_SWEEP_STEPS} ({step_deg:.0f}° per step)")
    lines.append(f"Pose configs : {', '.join(label for label, _ in POSE_CONFIGS)}")
    lines.append("")

    for pose_label, _ in POSE_CONFIGS:
        lines.append("=" * (name_w + 4 + len(angles) * (col_w + 1)))
        lines.append(f"POSE: {pose_label}")
        lines.append("  OK = reachable at j7={}mm   . = not reachable".format(J7_LOCKED))
        lines.append("")

        header = " " * (name_w + 4)
        for a in angles:
            header += f"{int(a):>{col_w}}°"
        lines.append(header)
        lines.append("  " + "-" * (name_w + 2 + len(angles) * (col_w + 1)))

        for r in results:
            pd  = r["poses"][pose_label]
            row = f"  {r['name']:<{name_w}}  "
            for a in angles:
                cell = "  OK" if a in pd["reachable_angles"] else "   ."
                row += f"{cell:>{col_w}} "
            native = "OK" if pd["native_ok"] else "FAIL"
            row += f"   native={native}  pos_err={pd['native_pos_err']:.2f}mm"
            lines.append(row)
        lines.append("")

    # Common reachable angles across ALL cones and ALL poses
    lines.append("=" * 60)
    lines.append("ANGLES REACHABLE FOR ALL CONES AT ALL POSES")
    all_sets = []
    for r in results:
        for pose_label, _ in POSE_CONFIGS:
            s = set(r["poses"][pose_label]["reachable_angles"])
            all_sets.append(s)
    if all_sets:
        common = sorted(all_sets[0].intersection(*all_sets[1:]))
    else:
        common = []
    lines.append(f"  {common if common else 'None'}")
    lines.append("")

    with open(out_path, "w") as f:
        f.write("\n".join(lines))


def run_regular_ik(robot, RDK, cone_targets, world_frame, group):
    """--regular_IK mode: solve grab + approach for each cone (no orientation sweep),
    move the robot to each solved position, and pause for inspection.

    Uses the same OptimAxes solver as compute_all_offsets in moving_a_cone.
    Prints the same verbose diagnostics as solve_pose.
    """
    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    print(f"\nRegular IK mode — no orientation sweep.")
    print(f"  j7 locked at {J7_LOCKED} mm  |  approach offset {APPROACH_OFFSET_MM} mm")
    print(f"  Solver: RoboDK OptimAxes Algorithm 3 (DLS), MaxIter=500")
    print("=" * 72)

    summary = []

    try:
        for target in cone_targets:
            name      = target.Name()
            grab_pose = target.PoseAbs()
            app_pose  = grab_pose * transl(0, 0, APPROACH_OFFSET_MM)

            pos, z = pos_and_z(grab_pose)
            print(f"\n{name}")
            print(f"  Grab pos : X={pos[0]:.1f}  Y={pos[1]:.1f}  Z={pos[2]:.1f}")
            print(f"  Grab Z   : ({z[0]:.4f}, {z[1]:.4f}, {z[2]:.4f})")

            print(f"  [grab]")
            grab_joints, grab_err, grab_ok = solve_pose(
                robot, grab_pose, J7_LOCKED, f"{name} grab"
            )
            if grab_ok:
                robot.MoveJ(grab_joints)
                input("    ↳ Robot at GRAB position.  Press Enter to continue ...")
                robot.setJoints(HOME_SEED)

            print(f"  [approach]")
            app_joints, app_err, app_ok = solve_pose(
                robot, app_pose, J7_LOCKED, f"{name} approach"
            )
            if app_ok:
                robot.MoveJ(app_joints)
                input("    ↳ Robot at APPROACH position.  Press Enter to continue ...")
                robot.setJoints(HOME_SEED)

            gs  = "SUCCESS" if grab_ok else "FAIL"
            as_ = "SUCCESS" if app_ok  else "FAIL"
            summary.append((name, gs, as_))

            add_frame(RDK, f"viz_grab_{name}",     grab_pose, group)
            add_frame(RDK, f"viz_approach_{name}", app_pose,  group)

    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)
        robot.setJoints(HOME_SEED)

    print("\n" + "=" * 60)
    print("SUMMARY")
    print(f"  {'Cone':<28} {'Grab':>8}   {'Approach':>9}")
    print("  " + "-" * 52)
    for name, gs, as_ in summary:
        print(f"  {name:<28} {gs:>8}   {as_:>9}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tool", default=TOOL_NAME,
                    help=f"RoboDK tool name to mount during IK solve (default: {TOOL_NAME})")
    ap.add_argument("--regular_IK", action="store_true",
                    help="Solve grab + approach per cone (no orientation sweep) using "
                         "the same OptimAxes method as moving_a_cone. Moves the robot "
                         "to each solved position and pauses for inspection.")
    args = ap.parse_args()

    tool_name = args.tool

    RDK = connect()

    robot = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError(f"Robot '{ROBOT_NAME}' not found.")

    tool = RDK.Item(tool_name, ITEM_TYPE_TOOL)
    if not tool.Valid():
        all_tools = [i.Name() for i in RDK.ItemList(ITEM_TYPE_TOOL)]
        raise RuntimeError(f"Tool '{tool_name}' not found. Available: {all_tools}")
    robot.setTool(tool)

    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    if not world_frame.Valid():
        raise RuntimeError("'WorldFrame' not found.")

    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    group = RDK.Item(VIZ_GROUP_NAME, ITEM_TYPE_FRAME)
    if not group.Valid():
        group = RDK.AddFrame(VIZ_GROUP_NAME, world_frame)
        group.setPose(eye(4))

    cone_targets = sorted(
        [t for t in RDK.ItemList(ITEM_TYPE_TARGET)
         if t.Name().startswith("base_cone_grab_")],
        key=lambda t: t.Name(),
    )
    if not cone_targets:
        raise RuntimeError("No base_cone_grab_* targets found in station.")

    print(f"\nFound {len(cone_targets)} cone targets.")

    if args.regular_IK:
        run_regular_ik(robot, RDK, cone_targets, world_frame, group)
        return

    step_deg = 360.0 / ORIENT_SWEEP_STEPS
    angles   = [i * step_deg for i in range(ORIENT_SWEEP_STEPS)]

    print(f"Poses           : {', '.join(l for l, _ in POSE_CONFIGS)}")
    print(f"j7 locked at    : {J7_LOCKED} mm")
    print(f"Approach offset : {APPROACH_OFFSET_MM} mm along grab Z-axis")
    print(f"Sweep           : {ORIENT_SWEEP_STEPS} orientations ({step_deg:.0f}° steps)")
    print(f"Solver          : RoboDK OptimAxes Algorithm 3 (DLS), MaxIter=500")
    print("=" * 72)

    results = []

    try:
        for target in cone_targets:
            name      = target.Name()
            grab_pose = target.PoseAbs()

            pos, z = pos_and_z(grab_pose)
            print(f"\n{name}")
            print(f"  Grab pos : X={pos[0]:.1f}  Y={pos[1]:.1f}  Z={pos[2]:.1f}")
            print(f"  Grab Z   : ({z[0]:.4f}, {z[1]:.4f}, {z[2]:.4f})")

            pose_results = {}

            for pose_label, pose_fn in POSE_CONFIGS:
                pose = pose_fn(grab_pose)
                print(f"  [{pose_label}]")

                # Native solve (0° — no rotation applied)
                joints, pos_err, native_ok = solve_pose(
                    robot, pose, J7_LOCKED, pose_label
                )

                # Orientation sweep — always run for every pose
                print(f"    Sweeping {ORIENT_SWEEP_STEPS} orientations ({int(step_deg)}° steps) ...")
                sweep, best_joints = sweep_orientations(
                    robot, pose, J7_LOCKED, ORIENT_SWEEP_STEPS
                )
                reachable_angles = [a for a, ok, _, _ in sweep if ok]
                for angle_deg, ok, detail, _ in sweep:
                    status = "SUCCESS" if ok else "  FAIL "
                    print(f"      [{status}] {angle_deg:5.1f}°  {detail}")
                n_ok = len(reachable_angles)
                print(f"    => {n_ok}/{ORIENT_SWEEP_STEPS} orientations reachable: {reachable_angles}")

                # Use best swept joints if native failed
                best = joints if native_ok else (best_joints if best_joints is not None else joints)

                pose_results[pose_label] = {
                    "native_ok":        native_ok,
                    "native_joints":    [float(v) for v in joints],
                    "native_pos_err":   pos_err,
                    "reachable_angles": reachable_angles,
                    "best_joints":      [float(v) for v in best] if best else [],
                    "swept_ok":         best_joints is not None,
                }

                # Viz frame for this pose
                add_frame(RDK, f"viz_{pose_label}_{name}", pose, group)

            results.append({"name": name, "poses": pose_results})

    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)
        robot.setJoints(HOME_SEED)

    # ── Print grids ───────────────────────────────────────────────────────────
    name_w = max(len(r["name"]) for r in results)

    print("\n" + "=" * 80)
    print("ORIENTATION SWEEP GRIDS")
    print(f"  OK = reachable at j7={J7_LOCKED}mm    . = not reachable")
    for pose_label, _ in POSE_CONFIGS:
        print_grid(results, pose_label, angles, name_w)

    # ── Summary counts ────────────────────────────────────────────────────────
    print("\n" + "=" * 80)
    print("REACHABILITY COUNTS")
    for pose_label, _ in POSE_CONFIGS:
        n = len(results)
        n_native = sum(1 for r in results if r["poses"][pose_label]["native_ok"])
        n_swept  = sum(1 for r in results if not r["poses"][pose_label]["native_ok"]
                       and r["poses"][pose_label]["swept_ok"])
        n_none   = sum(1 for r in results if not r["poses"][pose_label]["native_ok"]
                       and not r["poses"][pose_label]["swept_ok"])
        print(f"  {pose_label:<12}  native={n_native}/{n}  swept={n_swept}/{n}  unreachable={n_none}/{n}")

    # Common angles across all cones and all poses
    all_sets = []
    for r in results:
        for pose_label, _ in POSE_CONFIGS:
            all_sets.append(set(r["poses"][pose_label]["reachable_angles"]))
    common = sorted(all_sets[0].intersection(*all_sets[1:])) if all_sets else []
    print(f"\n  Angles reachable for ALL cones at ALL poses: {common if common else 'None'}")

    print(f"\nVisualization frames added under '{VIZ_GROUP_NAME}' in the station tree.")

    # ── Save files ────────────────────────────────────────────────────────────
    os.makedirs(IK_SOLUTIONS_DIR, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    json_path = os.path.join(IK_SOLUTIONS_DIR, f"base_cone_ik_{timestamp}.json")
    with open(json_path, "w") as f:
        json.dump({
            "generated":         timestamp,
            "j7_locked":         J7_LOCKED,
            "approach_offset_mm": APPROACH_OFFSET_MM,
            "orient_sweep_steps": ORIENT_SWEEP_STEPS,
            "solver":            "OptimAxes Algorithm 3 DLS",
            "tool":              tool_name,
            "robot":             ROBOT_NAME,
            "pose_configs":      [l for l, _ in POSE_CONFIGS],
            "solutions":         results,
        }, f, indent=2)
    print(f"\nIK solutions (JSON) : {json_path}")

    txt_path = os.path.join(IK_SOLUTIONS_DIR, f"base_cone_summary_{timestamp}.txt")
    save_summary_txt(results, angles, timestamp, txt_path, tool_name=tool_name)
    print(f"Summary (txt)       : {txt_path}")


if __name__ == "__main__":
    main()
