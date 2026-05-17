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

sys.path.append("C:/RoboDK/Python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET, ITEM_TYPE_FRAME
from robodk.robomath import eye, transl, rotz

from test_reach_base_cone import pos_and_z, fmt_joints

pi = 3.141592653589793

# ── CONFIG ────────────────────────────────────────────────────────────────────
APPROACH_OFFSET_MM  = 200.0   # offset from grab point along grab Z-axis
J7_LOCKED           = 0.0     # rail position held fixed during all solves (mm)
HOME_SEED           = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
VIZ_GROUP_NAME      = "ReachabilityCheck"
ROBOT_NAME          = "Fanuc R2000iC 125L"
TOOL_NAME           = "pickup_closed"
IK_SOLUTIONS_DIR    = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ik_solutions")
ORIENT_SWEEP_STEPS  = 12      # orientations to test when approach fails (360/12 = 30° each)

# OptimAxes parameters — mirrors DHR's OptimizationKinematicsModel.
# Algorithm 3 = damped least squares (numerical).  RoboDK's solver natively
# respects all coupled joint limits (e.g. the R2000iC J2/J3 interference zone)
# so we don't need to handle them manually.  AbsJnt_7 + AbsOn_7 + AbsW_7
# strongly constrain the rail to J7_LOCKED without hard-locking it.
OPT_AXES_STATIC_J7 = {
    "AbsJnt_7": 0,    # overridden per call with J7_LOCKED
    "AbsOn_7":  1,    # enable absolute constraint on j7
    "AbsW_7":   100,  # weight (matches DHR)
    "Algorithm": 3,   # damped least squares
    "MaxIter":  500,
    "Tol":      0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
    "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
    "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
}
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


J7_TOL_MM = 10.0  # j7 must stay within this of j7_locked to count as SUCCESS


def run_ik(robot, pose, j7_locked, label):
    """Solve IK via RoboDK OptimAxes (Algorithm 3 DLS) with j7 constrained.

    Reports FAIL if:
      - MoveJ throws (unreachable / joint limit)
      - j7 drifts more than J7_TOL_MM from j7_locked (OptimAxes found a valid
        solution on a different branch that requires moving the rail)

    The j7 drift check is critical for reachability assessment: a "solution"
    that requires j7=1756mm when J7_LOCKED=0 means the pose is NOT reachable
    with the rail fixed, even if the Cartesian position is hit.

    Returns (joints, pos_err, angle_err, converged).
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
        j7_err = abs(j7_actual - j7_locked)
        if j7_err > J7_TOL_MM:
            robot.setJoints(HOME_SEED)
            print(f"    [FAIL   ] {label:30s}  (j7 drifted to {j7_actual:.1f}mm, required {j7_locked:.1f}mm — pose unreachable with rail fixed)")
            return [0.0] * 7, 999.0, 999.0, False

        # Compare target vs achieved TCP position
        achieved = robot.Pose()
        tp = [pose[0,3], pose[1,3], pose[2,3]]
        ap = [achieved[0,3], achieved[1,3], achieved[2,3]]
        pos_err = (sum((t-a)**2 for t,a in zip(tp,ap)))**0.5
        print(f"    [SUCCESS] {label:30s}  pos_err={pos_err:.2f}mm  j7={j7_actual:.1f}mm")
        print(f"           target  XYZ: ({tp[0]:.1f}, {tp[1]:.1f}, {tp[2]:.1f})")
        print(f"           achieved XYZ: ({ap[0]:.1f}, {ap[1]:.1f}, {ap[2]:.1f})")
        print(f"           joints: {fmt_joints(joints)}")
        robot.setJoints(HOME_SEED)
        return joints, pos_err, 0.0, True
    except Exception as e:
        robot.setJoints(HOME_SEED)
        print(f"    [FAIL   ] {label:30s}  ({e})")
        return [0.0] * 7, 999.0, 999.0, False


def sweep_approach_orientations(robot, app_pose, j7_locked, n_steps):
    """Rotate the approach pose around its local Z-axis in n_steps equal increments
    and test each orientation with OptimAxes + j7 drift check.

    Rotating around the LOCAL Z-axis of the approach pose (= grab Z-axis) covers
    all TCP roll angles around the approach direction.  This tells us whether the
    approach position is reachable at j7_locked under any wrist orientation, and
    which orientations work — directly informing tool design.

    Returns:
      sweep: list of (angle_deg, ok, detail_str, joints_or_None)
      best_joints: joints from the first successful orientation, or None
    """
    step_deg = 360.0 / n_steps
    props = dict(OPT_AXES_STATIC_J7)
    props["AbsJnt_7"] = j7_locked

    sweep = []
    best_joints = None

    for i in range(n_steps):
        angle_deg = i * step_deg
        rotated_pose = app_pose * rotz(angle_deg * pi / 180.0)

        robot.setParam("OptimAxes", props)
        robot.setJoints(HOME_SEED)
        try:
            robot.MoveJ(rotated_pose)
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
                tp = [app_pose[0,3], app_pose[1,3], app_pose[2,3]]
                ap_xyz = [achieved[0,3], achieved[1,3], achieved[2,3]]
                pos_err = (sum((t-a)**2 for t,a in zip(tp, ap_xyz)))**0.5
                sweep.append((angle_deg, True, f"j7={j7_actual:.1f}mm  pos_err={pos_err:.2f}mm", joints))
                if best_joints is None:
                    best_joints = joints
        except Exception as e:
            sweep.append((angle_deg, False, f"exception ({e})", None))

        robot.setJoints(HOME_SEED)

    return sweep, best_joints


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
    print(f"Solver          : RoboDK OptimAxes Algorithm 3 (DLS), MaxIter=500")
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
                robot, grab_pose, J7_LOCKED, "grab"
            )

            app_joints, app_pos_err, app_angle, app_ok = run_ik(
                robot, app_pose, J7_LOCKED, f"approach (+{APPROACH_OFFSET_MM:.0f}mm)",
            )

            # If approach fails at native orientation, sweep around grab Z-axis
            sweep_data = []
            app_swept_ok = False
            reachable_angles = []
            if not app_ok:
                step_deg = int(360 / ORIENT_SWEEP_STEPS)
                print(f"    Sweeping {ORIENT_SWEEP_STEPS} orientations ({step_deg}° steps) around grab Z-axis ...")
                sweep_data, swept_joints = sweep_approach_orientations(
                    robot, app_pose, J7_LOCKED, ORIENT_SWEEP_STEPS
                )
                for angle_deg, ok, detail, _ in sweep_data:
                    status = "SUCCESS" if ok else "  FAIL "
                    print(f"      [{status}] {angle_deg:5.1f}°  {detail}")
                reachable_angles = [a for a, ok, _, _ in sweep_data if ok]
                if swept_joints is not None:
                    app_swept_ok = True
                    app_joints   = swept_joints   # save best joints for this approach
                    print(f"    => {len(reachable_angles)}/{ORIENT_SWEEP_STEPS} orientations reachable: {reachable_angles}")
                else:
                    print(f"    => Approach position unreachable at j7={J7_LOCKED}mm at any orientation")

            # Add XYZ triad frames in station for visualization
            add_frame(RDK, f"viz_grab_{name}",     grab_pose, group)
            add_frame(RDK, f"viz_approach_{name}", app_pose,  group)

            results.append({
                "name":              name,
                "grab_ok":           grab_ok,
                "grab_pos_err":      grab_pos_err,
                "grab_angle":        grab_angle,
                "grab_joints":       [float(v) for v in grab_joints],
                "app_ok":            app_ok,
                "app_swept_ok":      app_swept_ok,
                "reachable_angles":  reachable_angles,
                "app_pos_err":       app_pos_err,
                "app_angle":         app_angle,
                "app_joints":        [float(v) for v in app_joints],
            })

    finally:
        # Always restore frame and home the robot
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)
        robot.setJoints(HOME_SEED)

    # ── Summary table ────────────────────────────────────────────────────────
    print("\n" + "=" * 100)
    print("REACHABILITY SUMMARY")
    print(f"  j7 locked={J7_LOCKED} mm   approach offset={APPROACH_OFFSET_MM} mm   orientation sweep={ORIENT_SWEEP_STEPS} steps")
    print(f"  {'Cone':<25} {'Grab':>7}   {'Approach (native)':>17}   {'Approach (any orient.)':>22}   Reachable angles")
    print("-" * 100)
    for r in results:
        gs   = "SUCCESS" if r["grab_ok"]     else "FAIL"
        as_  = "SUCCESS" if r["app_ok"]      else "FAIL"
        sw   = f"{len(r['reachable_angles'])}/{ORIENT_SWEEP_STEPS} ok" if not r["app_ok"] else "n/a"
        angs = str(r["reachable_angles"]) if r["reachable_angles"] else ("—" if not r["app_ok"] else "native ok")
        print(f"  {r['name']:<25} {gs:>7}   {as_:>17}   {sw:>22}   {angs}")
    print("=" * 100)

    n        = len(results)
    n_grab   = sum(1 for r in results if r["grab_ok"])
    n_app    = sum(1 for r in results if r["app_ok"])
    n_swept  = sum(1 for r in results if not r["app_ok"] and r["app_swept_ok"])
    n_none   = sum(1 for r in results if not r["app_ok"] and not r["app_swept_ok"])
    print(f"Grab reachable              : {n_grab}/{n}")
    print(f"Approach reachable (native) : {n_app}/{n}")
    print(f"Approach reachable (swept)  : {n_swept}/{n}  (needed orientation change)")
    print(f"Approach unreachable        : {n_none}/{n}  (no orientation works at j7={J7_LOCKED}mm)")
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
        "solver":          "OptimAxes Algorithm 3 DLS",
        "tool":            TOOL_NAME,
        "robot":           ROBOT_NAME,
        "solutions":       results,
    }
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\nIK solutions saved to: {out_path}")


if __name__ == "__main__":
    main()
