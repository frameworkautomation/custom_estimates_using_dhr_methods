"""
Reachability checker for all base_cone_grab_* targets.

For each cone grab target found in the RoboDK station:
  1. Solves IK at the grab pose        (j7 locked at J7_LOCKED)
  2. Solves IK at the approach pose    (APPROACH_OFFSET_MM along grab Z-axis)
  3. Adds RoboDK reference frames for visualization (XYZ triads) at both points
  4. Prints a full reachability summary table

APPROACH_OFFSET_MM: offset from grab point along the grab Z-axis.
  Positive = in the direction the TCP Z-axis points (away from the cone if
  Z points outward from the grab surface, i.e. the natural approach direction).

J7_LOCKED: rail position held fixed during all IK solves.
"""

import sys
import os
import json
import datetime
import numpy as np

sys.path.append("C:/RoboDK/Python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET, ITEM_TYPE_FRAME
from robodk.robomath import eye, transl

from test_reach_base_cone import custom_ik_pos_and_zaxis, pos_and_z, fmt_joints

# ── CONFIG ────────────────────────────────────────────────────────────────────
APPROACH_OFFSET_MM  = 200.0   # 20 cm offset along grab Z-axis
J7_LOCKED           = 0.0     # rail position locked for all solves (mm)
HOME_SEED           = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Multiple arm seeds to try in order.  The R2000iC has coupled J2/J3 limits
# (interference zone): some seeds lead to configurations that pass our
# per-joint clip but violate the coupled limit check RoboDK enforces during
# MoveJ.  We try each seed and keep the first solution RoboDK accepts.
ARM_SEEDS = [
    [0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0],   # home
    [0.0,  30.0, -90.0,   0.0, -30.0,   0.0,   0.0],   # elbow-up
    [0.0, -30.0,  60.0,   0.0,  30.0,   0.0,   0.0],   # elbow-down alt
    [0.0,  10.0, -60.0,   0.0, -60.0,   0.0,   0.0],   # mid-elbow-up
    [0.0, -10.0,  30.0,   0.0,  10.0,   0.0,   0.0],   # mid-elbow-down
]
POS_TOL_MM          = 0.5
ANGLE_TOL_DEG       = 2.0
MAX_ITERS           = 200
VERBOSE_IK          = False   # True for per-iteration output
VIZ_GROUP_NAME      = "ReachabilityCheck"  # parent frame grouping viz frames
ROBOT_NAME          = "Fanuc R2000iC 125L"
TOOL_NAME           = "pickup_closed"      # set to your tool name in RoboDK
IK_SOLUTIONS_DIR    = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ik_solutions")
# ─────────────────────────────────────────────────────────────────────────────


def connect():
    try:
        rdk = Robolink()
        rdk.Item("")  # probe connection
        print("[INFO] Connected to RoboDK on localhost")
        return rdk
    except Exception:
        print("[INFO] localhost failed, trying 172.23.208.1 ...")
        return Robolink(robodk_ip="172.23.208.1")


def make_approach_pose(grab_pose, offset_mm):
    """Offset grab_pose by offset_mm along its own local Z-axis."""
    return grab_pose * transl(0, 0, offset_mm)


def joints_accepted_by_robodk(robot, joints):
    """Return True if RoboDK accepts these joints for a MoveJ (coupled limit check).

    Uses setJoints (no motion planning) then reads back Joints() to see if
    RoboDK silently clamped any value, which indicates a limit violation.
    Also does a quick CollisionCheck via robot.JointPose if available.
    """
    robot.setJoints(joints)
    actual = list(robot.Joints().list())
    for k, (sent, got) in enumerate(zip(joints, actual)):
        if abs(sent - got) > 0.5:   # 0.5 deg/mm tolerance
            return False
    return True


def run_ik(robot, pose, seed, j7_locked, label):
    """Run LM IK solver across multiple arm seeds; return first solution RoboDK accepts.

    The R2000iC has coupled J2/J3 limits.  The LM solver clips joints
    individually but the coupling is only detected when RoboDK validates the
    configuration.  We check via setJoints/readback after each solve.
    """
    best = None
    for arm_seed in ARM_SEEDS:
        s = list(arm_seed)
        s[6] = j7_locked
        # Warm-start from the explicit seed if provided (not home)
        if seed is not None and seed != list(arm_seed):
            s = list(seed)
            s[6] = j7_locked
        result, pos_err, angle_deg, converged = custom_ik_pos_and_zaxis(
            robot, pose, s,
            pos_tol=POS_TOL_MM,
            angle_tol_deg=ANGLE_TOL_DEG,
            max_iters=MAX_ITERS,
            verbose=VERBOSE_IK,
        )
        if converged and joints_accepted_by_robodk(robot, result):
            tag = "SUCCESS"
            print(f"    [{tag}] {label:30s}  pos_err={pos_err:7.3f} mm  angle={angle_deg:6.3f} deg")
            print(f"           joints: {fmt_joints(result)}")
            return result, pos_err, angle_deg, True
        if converged and best is None:
            best = (result, pos_err, angle_deg)   # converged but coupled-limit fail

    # Fall back to best converged result (might still fail MoveJ)
    if best is not None:
        result, pos_err, angle_deg = best
        print(f"    [WARN ] {label:30s}  pos_err={pos_err:7.3f} mm  angle={angle_deg:6.3f} deg  (coupled limit suspect)")
        return result, pos_err, angle_deg, True
    print(f"    [FAIL ] {label:30s}  no seed converged")
    return [0.0] * 7, 999.0, 999.0, False


def add_frame(RDK, name, pose, parent):
    """Add (or replace) a reference frame in the station at the given pose."""
    existing = RDK.Item(name, ITEM_TYPE_FRAME)
    if existing.Valid():
        existing.Delete()
    frame = RDK.AddFrame(name, parent)
    frame.setPose(pose)
    return frame


def main():
    RDK = connect()

    robot = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError("Robot '" + ROBOT_NAME + "' not found.")

    tool = RDK.Item(TOOL_NAME, ITEM_TYPE_TOOL)
    if not tool.Valid():
        all_tools = [i.Name() for i in RDK.ItemList(ITEM_TYPE_TOOL)]
        raise RuntimeError(
            "Tool '" + TOOL_NAME + "' not found. Tools in station: " + str(all_tools)
        )
    robot.setTool(tool)

    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    if not world_frame.Valid():
        raise RuntimeError("'WorldFrame' not found.")

    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    # Parent frame to group all viz frames in the station tree
    group = RDK.Item(VIZ_GROUP_NAME, ITEM_TYPE_FRAME)
    if not group.Valid():
        group = RDK.AddFrame(VIZ_GROUP_NAME, world_frame)
        group.setPose(eye(4))

    # Collect all base_cone_grab_* targets, sorted by name
    cone_targets = sorted(
        [t for t in RDK.ItemList(ITEM_TYPE_TARGET)
         if t.Name().startswith("base_cone_grab_")],
        key=lambda t: t.Name(),
    )

    if not cone_targets:
        raise RuntimeError("No base_cone_grab_* targets found in station.")

    print(f"\nFound {len(cone_targets)} cone targets.")
    print(f"Approach offset : {APPROACH_OFFSET_MM} mm along grab Z-axis")
    print(f"j7 locked at    : {J7_LOCKED} mm")
    print(f"Tolerances      : pos < {POS_TOL_MM} mm,  angle < {ANGLE_TOL_DEG} deg")
    print("=" * 72)

    results = []

    try:
        for target in cone_targets:
            name = target.Name()
            grab_pose = target.PoseAbs()
            app_pose  = make_approach_pose(grab_pose, APPROACH_OFFSET_MM)

            pos, z = pos_and_z(grab_pose)
            print(f"\n{name}")
            print(f"  Grab pos : X={pos[0]:.1f}  Y={pos[1]:.1f}  Z={pos[2]:.1f}")
            print(f"  Grab Z   : ({z[0]:.4f}, {z[1]:.4f}, {z[2]:.4f})")

            grab_joints, grab_pos_err, grab_angle, grab_ok = run_ik(
                robot, grab_pose, HOME_SEED, J7_LOCKED, "grab"
            )

            # Seed approach from grab result if converged, else home
            app_seed = grab_joints if grab_ok else list(HOME_SEED)
            app_joints, app_pos_err, app_angle, app_ok = run_ik(
                robot, app_pose, app_seed, J7_LOCKED, f"approach (+{APPROACH_OFFSET_MM:.0f}mm)"
            )

            # Add XYZ triad frames in station for visualization
            add_frame(RDK, f"viz_grab_{name}",     grab_pose, group)
            add_frame(RDK, f"viz_approach_{name}", app_pose,  group)

            results.append({
                "name":         name,
                "grab_ok":      grab_ok,
                "grab_pos_err": grab_pos_err,
                "grab_angle":   grab_angle,
                "grab_joints":  [float(v) for v in grab_joints],
                "app_ok":       app_ok,
                "app_pos_err":  app_pos_err,
                "app_angle":    app_angle,
                "app_joints":   [float(v) for v in app_joints],
            })

    finally:
        # Always restore frame and home the robot
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)
        robot.setJoints(HOME_SEED)

    # ── Summary table ────────────────────────────────────────────────────────
    print("\n" + "=" * 90)
    print("SUCCESSABILITY SUMMARY")
    print(f"  j7 locked={J7_LOCKED} mm   approach offset={APPROACH_OFFSET_MM} mm")
    print(f"  {'Cone':<25} {'Grab':>7} {'pos err':>9} {'ang err':>9}   {'Approach':>9} {'pos err':>9} {'ang err':>9}")
    print("-" * 90)
    for r in results:
        gs  = "SUCCESS" if r["grab_ok"] else "FAIL"
        as_ = "SUCCESS" if r["app_ok"]  else "FAIL"
        print(
            f"  {r['name']:<25} {gs:>7} {r['grab_pos_err']:>8.3f}mm {r['grab_angle']:>8.3f}deg"
            f"   {as_:>9} {r['app_pos_err']:>8.3f}mm {r['app_angle']:>8.3f}deg"
        )
    print("=" * 90)

    n       = len(results)
    n_grab  = sum(1 for r in results if r["grab_ok"])
    n_app   = sum(1 for r in results if r["app_ok"])
    print(f"Grab reachable    : {n_grab}/{n}")
    print(f"Approach reachable: {n_app}/{n}")
    print(f"\nVisualization frames added under '{VIZ_GROUP_NAME}' in the station tree.")
    print("(Red=X, Green=Y, Blue=Z arrows visible in RoboDK viewport)")

    # ── Save IK solutions ─────────────────────────────────────────────────────
    os.makedirs(IK_SOLUTIONS_DIR, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(IK_SOLUTIONS_DIR, f"base_cone_ik_{timestamp}.json")
    payload = {
        "generated":       timestamp,
        "j7_locked":       J7_LOCKED,
        "approach_offset_mm": APPROACH_OFFSET_MM,
        "pos_tol_mm":      POS_TOL_MM,
        "angle_tol_deg":   ANGLE_TOL_DEG,
        "tool":            TOOL_NAME,
        "robot":           ROBOT_NAME,
        "solutions":       results,
    }
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\nIK solutions saved to: {out_path}")


if __name__ == "__main__":
    main()
