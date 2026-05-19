"""
Reachability checker for all base_cone_grab_* targets — with collision avoidance.

Extends check_base_cone_reachability.py by adding a per-solution collision check.
After OptimAxes lands the robot at each IK solution, RoboDK.Collisions() is queried
to see whether the robot arm intersects any geometry in AVOID_ITEM_NAMES.

Result categories per (cone, pose, angle):
  OK    — IK solved, j7 in range, no collision with avoid items
  CL    — IK solved, j7 in range, but collides with an avoid item
  .     — IK failed (j7 drifted or exception)

If AVOID_ITEM_NAMES is empty the script behaves identically to the original and
no collision state changes are made to the station.

HOW COLLISION CHECKING WORKS
  1. Before the solve loop, global collision checking is enabled via
     RDK.setCollisionActive(COLLISION_ON).
  2. After each robot.MoveJ(pose) the robot is sitting at the IK solution.
     RDK.Collisions() returns the number of colliding pairs at that instant.
  3. RDK.CollisionPairs() returns [[item1, item2, id1, id2], ...].
     We report a collision only if at least one item in a pair is an avoid item.
  4. The original global collision state is restored in a finally block.

CONFIGURATION
  AVOID_ITEM_NAMES  — list of RoboDK item names (strings) to treat as obstacles.
                      Leave empty to skip collision checking entirely.
  AVOID_ALL_COLLISIONS — if True, any collision counts (not just avoid items).
                         Useful when you trust the station is otherwise clean.
"""

import sys
import os
import json
import datetime

sys.path.append("C:/RoboDK/Python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robodk.robolink import (
    Robolink,
    ITEM_TYPE_ROBOT,
    ITEM_TYPE_TOOL,
    ITEM_TYPE_TARGET,
    ITEM_TYPE_FRAME,
    COLLISION_ON,
    COLLISION_OFF,
)
from robodk.robomath import eye, transl, rotz

from test_reach_base_cone import pos_and_z, fmt_joints

pi = 3.141592653589793

# ── CONFIG ────────────────────────────────────────────────────────────────────
APPROACH_OFFSET_MM = 200.0    # offset from grab point along grab Z-axis
J7_LOCKED          = 0.0      # rail position held fixed during all solves (mm)
HOME_SEED          = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
VIZ_GROUP_NAME     = "ReachabilityCheckAvoidance"
ROBOT_NAME         = "Fanuc R2000iC 125L"
TOOL_NAME          = "pickup_closed"
IK_SOLUTIONS_DIR   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ik_solutions")
ORIENT_SWEEP_STEPS = 12       # 360 / 12 = 30° per step

# ── COLLISION AVOIDANCE CONFIG ────────────────────────────────────────────────
# Names of station items the robot must not collide with.
# These must match the item names exactly as shown in the RoboDK station tree.
# Leave empty to disable collision checking (identical behaviour to the original).
AVOID_ITEM_NAMES: list = [
    # "factory_wall",
    # "conveyor_frame",
]

# If True, ANY collision detected counts as a failure (not just avoid items).
# Only meaningful when AVOID_ITEM_NAMES is non-empty (since that's what triggers
# collision checking). Set True when the station is clean and any hit is bad.
# Set False when you only care about specific named geometry.
AVOID_ALL_COLLISIONS: bool = False
# ─────────────────────────────────────────────────────────────────────────────

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


# ── HELPERS ───────────────────────────────────────────────────────────────────

def connect():
    try:
        rdk = Robolink()
        rdk.Item("")
        print("[INFO] Connected to RoboDK on localhost")
        return rdk
    except Exception:
        print("[INFO] localhost failed, trying 172.23.208.1 ...")
        return Robolink(robodk_ip="172.23.208.1")


def check_collision(RDK, avoid_items: list) -> tuple:
    """Query current collision state.

    Returns (in_collision: bool, detail_str: str).
    in_collision is True if any currently-colliding pair involves an avoid item,
    OR if AVOID_ALL_COLLISIONS is True and any collision exists at all.
    """
    n = RDK.Collisions()
    if n == 0:
        return False, "clear"

    pairs = RDK.CollisionPairs()  # [[item1, item2, id1, id2], ...]
    avoid_names = {item.Name() for item in avoid_items}

    colliding_names = []
    hit_avoid = False
    for item1, item2, id1, id2 in pairs:
        n1, n2 = item1.Name(), item2.Name()
        colliding_names.append(f"{n1}(link{id1}) vs {n2}(link{id2})")
        if n1 in avoid_names or n2 in avoid_names:
            hit_avoid = True

    detail = "; ".join(colliding_names) if colliding_names else f"{n} collision(s)"

    if AVOID_ALL_COLLISIONS:
        return True, detail
    return hit_avoid, detail


def solve_pose(robot, pose, j7_locked, label, RDK=None, avoid_items=None):
    """Solve IK for one pose via OptimAxes.  Reports FAIL if j7 drifts.

    If RDK and avoid_items are provided, also checks for collisions at the
    solved configuration.

    Returns (joints, pos_err, ik_ok, collision_clear, collision_detail).
      ik_ok          — True if IK succeeded and j7 stayed in range
      collision_clear — True if no collision with avoid items (or no checking)
      collision_detail — human-readable string describing any collision
    """
    check_coll = RDK is not None and avoid_items is not None

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
            return [0.0] * 7, 999.0, False, True, "n/a"

        achieved = robot.Pose()
        tp = [pose[0,3], pose[1,3], pose[2,3]]
        ap = [achieved[0,3], achieved[1,3], achieved[2,3]]
        pos_err = (sum((t-a)**2 for t,a in zip(tp, ap)))**0.5

        # Collision check while robot is still at the IK solution
        collision_clear, coll_detail = True, "clear"
        if check_coll:
            collision_clear, coll_detail = check_collision(RDK, avoid_items)

        coll_tag = "" if not check_coll else (f"  COLLISION={coll_detail}" if not collision_clear else "  no-collision")
        print(f"    [SUCCESS] {label:35s}  pos_err={pos_err:.2f}mm  j7={j7_actual:.1f}mm{coll_tag}")
        print(f"           target  XYZ: ({tp[0]:.1f}, {tp[1]:.1f}, {tp[2]:.1f})")
        print(f"           achieved XYZ: ({ap[0]:.1f}, {ap[1]:.1f}, {ap[2]:.1f})")
        print(f"           joints: {fmt_joints(joints)}")
        robot.setJoints(HOME_SEED)
        return joints, pos_err, True, collision_clear, coll_detail
    except Exception as e:
        robot.setJoints(HOME_SEED)
        print(f"    [FAIL   ] {label:35s}  ({e})")
        return [0.0] * 7, 999.0, False, True, "n/a"


def sweep_orientations(robot, pose, j7_locked, n_steps, RDK=None, avoid_items=None):
    """Rotate pose around its local Z-axis in n_steps equal increments and
    test each with OptimAxes + j7 drift check + optional collision check.

    Returns:
      sweep: list of (angle_deg, ik_ok, collision_clear, detail_str, joints_or_None)
      best_joints: joints from first collision-free successful orientation, or
                   first successful (even if colliding) if none are clear, or None
    """
    check_coll = RDK is not None and avoid_items is not None
    step_deg = 360.0 / n_steps
    props = dict(OPT_AXES_STATIC_J7)
    props["AbsJnt_7"] = j7_locked

    sweep = []
    best_joints = None          # first ok + collision_clear
    best_joints_fallback = None # first ok (may be colliding)

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
                sweep.append((angle_deg, False, True, f"j7={j7_actual:.0f}mm (drifted)", None))
            else:
                achieved = robot.Pose()
                tp    = [pose[0,3], pose[1,3], pose[2,3]]
                ap    = [achieved[0,3], achieved[1,3], achieved[2,3]]
                pos_err = (sum((t-a)**2 for t,a in zip(tp, ap)))**0.5

                collision_clear, coll_detail = True, "clear"
                if check_coll:
                    collision_clear, coll_detail = check_collision(RDK, avoid_items)

                coll_tag = "" if not check_coll else (f"  COLLISION={coll_detail}" if not collision_clear else "")
                detail = f"j7={j7_actual:.1f}mm  pos_err={pos_err:.2f}mm{coll_tag}"
                sweep.append((angle_deg, True, collision_clear, detail, joints))

                if collision_clear and best_joints is None:
                    best_joints = joints
                if best_joints_fallback is None:
                    best_joints_fallback = joints

        except Exception as e:
            sweep.append((angle_deg, False, True, f"exception ({e})", None))

        robot.setJoints(HOME_SEED)

    # Prefer collision-clear; fall back to any valid IK solution
    final_best = best_joints if best_joints is not None else best_joints_fallback
    return sweep, final_best


def add_frame(RDK, name, pose, parent):
    existing = RDK.Item(name, ITEM_TYPE_FRAME)
    if existing.Valid():
        existing.Delete()
    frame = RDK.AddFrame(name, parent)
    frame.setPose(pose)
    return frame


def print_grid(results, pose_label, angles, name_w, collision_checking: bool):
    """Print one orientation sweep grid for a given pose label.

    Cell legend:
      OK  — IK ok, no collision (or collision checking disabled)
      CL  — IK ok, but collides with an avoid item
      .   — IK failed
    """
    col_w = 5
    print(f"\nPOSE: {pose_label}")
    header = " " * (name_w + 4)
    for a in angles:
        header += f"{int(a):>{col_w}}°"
    print(header)
    print("  " + "-" * (name_w + 2 + len(angles) * (col_w + 1)))
    for r in results:
        pd  = r["poses"][pose_label]
        row = f"  {r['name']:<{name_w}}  "
        for a in angles:
            if a in pd["reachable_clear_angles"]:
                cell = "  OK"
            elif collision_checking and a in pd["reachable_collision_angles"]:
                cell = "  CL"
            else:
                cell = "   ."
            row += f"{cell:>{col_w}} "
        print(row)


def save_summary_txt(results, angles, timestamp, out_path, collision_checking: bool):
    """Write a human-readable summary txt to out_path."""
    step_deg = angles[1] - angles[0] if len(angles) > 1 else 360.0
    name_w   = max(len(r["name"]) for r in results)
    col_w    = 5

    lines = []
    lines.append("BASE CONE REACHABILITY SUMMARY (WITH COLLISION AVOIDANCE)")
    lines.append(f"Generated         : {timestamp}")
    lines.append(f"Robot             : {ROBOT_NAME}")
    lines.append(f"Tool              : {TOOL_NAME}")
    lines.append(f"j7 locked         : {J7_LOCKED} mm")
    lines.append(f"Sweep steps       : {ORIENT_SWEEP_STEPS} ({step_deg:.0f}° per step)")
    lines.append(f"Pose configs      : {', '.join(label for label, _ in POSE_CONFIGS)}")
    lines.append(f"Collision checking: {'ON' if collision_checking else 'OFF'}")
    if collision_checking:
        lines.append(f"Avoid items       : {AVOID_ITEM_NAMES}")
        lines.append(f"Avoid all coll.   : {AVOID_ALL_COLLISIONS}")
    lines.append("")
    legend = "  OK = IK ok, no collision    CL = IK ok, collides    . = IK fail"
    if not collision_checking:
        legend = "  OK = reachable   . = not reachable   (collision checking disabled)"

    for pose_label, _ in POSE_CONFIGS:
        lines.append("=" * (name_w + 4 + len(angles) * (col_w + 1)))
        lines.append(f"POSE: {pose_label}")
        lines.append(legend)
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
                if a in pd["reachable_clear_angles"]:
                    cell = "  OK"
                elif collision_checking and a in pd["reachable_collision_angles"]:
                    cell = "  CL"
                else:
                    cell = "   ."
                row += f"{cell:>{col_w}} "
            native = "OK" if pd["native_ok"] else "FAIL"
            native_coll = ""
            if collision_checking and pd["native_ok"]:
                native_coll = " (collision)" if not pd["native_collision_clear"] else " (clear)"
            row += f"   native={native}{native_coll}  pos_err={pd['native_pos_err']:.2f}mm"
            lines.append(row)
        lines.append("")

    # Common clear angles across ALL cones and ALL poses
    lines.append("=" * 60)
    lines.append("ANGLES CLEAR (IK OK + NO COLLISION) FOR ALL CONES AT ALL POSES")
    all_sets = []
    for r in results:
        for pose_label, _ in POSE_CONFIGS:
            s = set(r["poses"][pose_label]["reachable_clear_angles"])
            all_sets.append(s)
    if all_sets:
        common = sorted(all_sets[0].intersection(*all_sets[1:]))
    else:
        common = []
    lines.append(f"  {common if common else 'None'}")
    lines.append("")

    with open(out_path, "w") as f:
        f.write("\n".join(lines))


# ── MAIN ──────────────────────────────────────────────────────────────────────

def main():
    RDK = connect()

    robot = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError(f"Robot '{ROBOT_NAME}' not found.")

    tool = RDK.Item(TOOL_NAME, ITEM_TYPE_TOOL)
    if not tool.Valid():
        all_tools = [i.Name() for i in RDK.ItemList(ITEM_TYPE_TOOL)]
        raise RuntimeError(f"Tool '{TOOL_NAME}' not found. Available: {all_tools}")
    robot.setTool(tool)

    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    if not world_frame.Valid():
        raise RuntimeError("'WorldFrame' not found.")

    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    # ── Resolve avoidance items ────────────────────────────────────────────────
    collision_checking = bool(AVOID_ITEM_NAMES)
    avoid_items = []
    original_collision_state = None

    if collision_checking:
        for name in AVOID_ITEM_NAMES:
            item = RDK.Item(name)
            if item.Valid():
                avoid_items.append(item)
                print(f"[AVOIDANCE] Will check collisions against: {name}")
            else:
                print(f"[WARNING]   Avoidance item not found in station: '{name}'")
        if not avoid_items:
            print("[WARNING] No valid avoidance items found — collision checking disabled.")
            collision_checking = False
        else:
            # Remember previous state so we can restore it, then enable
            original_collision_state = COLLISION_OFF  # conservative: assume it was off
            RDK.setCollisionActive(COLLISION_ON)
            print(f"[AVOIDANCE] Collision checking ENABLED for {len(avoid_items)} item(s).")
    else:
        print("[INFO] AVOID_ITEM_NAMES is empty — no collision checking.")
    # ──────────────────────────────────────────────────────────────────────────

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

    step_deg = 360.0 / ORIENT_SWEEP_STEPS
    angles   = [i * step_deg for i in range(ORIENT_SWEEP_STEPS)]

    print(f"\nFound {len(cone_targets)} cone targets.")
    print(f"Poses           : {', '.join(l for l, _ in POSE_CONFIGS)}")
    print(f"j7 locked at    : {J7_LOCKED} mm")
    print(f"Approach offset : {APPROACH_OFFSET_MM} mm along grab Z-axis")
    print(f"Sweep           : {ORIENT_SWEEP_STEPS} orientations ({step_deg:.0f}° steps)")
    print(f"Solver          : RoboDK OptimAxes Algorithm 3 (DLS), MaxIter=500")
    print(f"Collision check : {'ON — ' + str(AVOID_ITEM_NAMES) if collision_checking else 'OFF'}")
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
                joints, pos_err, native_ok, native_coll_clear, native_coll_detail = solve_pose(
                    robot, pose, J7_LOCKED, pose_label,
                    RDK=RDK if collision_checking else None,
                    avoid_items=avoid_items if collision_checking else None,
                )

                # Orientation sweep
                print(f"    Sweeping {ORIENT_SWEEP_STEPS} orientations ({int(step_deg)}° steps) ...")
                sweep, best_joints = sweep_orientations(
                    robot, pose, J7_LOCKED, ORIENT_SWEEP_STEPS,
                    RDK=RDK if collision_checking else None,
                    avoid_items=avoid_items if collision_checking else None,
                )

                reachable_clear_angles     = [a for a, ik_ok, coll_clear, _, _ in sweep if ik_ok and coll_clear]
                reachable_collision_angles = [a for a, ik_ok, coll_clear, _, _ in sweep if ik_ok and not coll_clear]

                for angle_deg, ik_ok, coll_clear, detail, _ in sweep:
                    if not ik_ok:
                        status = "  FAIL "
                    elif not coll_clear:
                        status = "COLLIDE"
                    else:
                        status = "SUCCESS"
                    print(f"      [{status}] {angle_deg:5.1f}°  {detail}")

                n_clear = len(reachable_clear_angles)
                n_coll  = len(reachable_collision_angles)
                if collision_checking:
                    print(f"    => clear={n_clear}/{ORIENT_SWEEP_STEPS}  colliding={n_coll}/{ORIENT_SWEEP_STEPS}  "
                          f"clear_angles={reachable_clear_angles}")
                else:
                    print(f"    => {n_clear}/{ORIENT_SWEEP_STEPS} orientations reachable: {reachable_clear_angles}")

                # Best joints: prefer clear, fall back to any valid IK
                if native_ok and native_coll_clear:
                    best = joints
                elif best_joints is not None:
                    best = best_joints
                else:
                    best = joints

                pose_results[pose_label] = {
                    "native_ok":                native_ok,
                    "native_joints":            [float(v) for v in joints],
                    "native_pos_err":           pos_err,
                    "native_collision_clear":   native_coll_clear,
                    "native_collision_detail":  native_coll_detail,
                    "reachable_clear_angles":   reachable_clear_angles,
                    "reachable_collision_angles": reachable_collision_angles,
                    "best_joints":              [float(v) for v in best] if best else [],
                    "swept_ok":                 best_joints is not None,
                }

                # Viz frame for this pose
                add_frame(RDK, f"viz_{pose_label}_{name}", pose, group)

            results.append({"name": name, "poses": pose_results})

    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)
        robot.setJoints(HOME_SEED)
        # Restore collision state
        if original_collision_state is not None:
            RDK.setCollisionActive(original_collision_state)
            print("\n[AVOIDANCE] Collision checking restored to original state.")

    # ── Print grids ───────────────────────────────────────────────────────────
    name_w = max(len(r["name"]) for r in results)

    print("\n" + "=" * 80)
    print("ORIENTATION SWEEP GRIDS")
    if collision_checking:
        print(f"  OK = IK ok + no collision    CL = IK ok + collides    . = IK fail")
    else:
        print(f"  OK = reachable at j7={J7_LOCKED}mm    . = not reachable")
    for pose_label, _ in POSE_CONFIGS:
        print_grid(results, pose_label, angles, name_w, collision_checking)

    # ── Summary counts ────────────────────────────────────────────────────────
    print("\n" + "=" * 80)
    print("REACHABILITY COUNTS")
    for pose_label, _ in POSE_CONFIGS:
        n = len(results)
        n_native_clear = sum(1 for r in results
                             if r["poses"][pose_label]["native_ok"]
                             and r["poses"][pose_label]["native_collision_clear"])
        n_native_coll  = sum(1 for r in results
                             if r["poses"][pose_label]["native_ok"]
                             and not r["poses"][pose_label]["native_collision_clear"])
        n_swept_clear  = sum(1 for r in results
                             if not r["poses"][pose_label]["native_ok"]
                             and bool(r["poses"][pose_label]["reachable_clear_angles"]))
        n_none         = sum(1 for r in results
                             if not r["poses"][pose_label]["native_ok"]
                             and not r["poses"][pose_label]["reachable_clear_angles"]
                             and not r["poses"][pose_label]["reachable_collision_angles"])
        if collision_checking:
            print(f"  {pose_label:<12}  native_clear={n_native_clear}/{n}  "
                  f"native_collide={n_native_coll}/{n}  "
                  f"swept_clear={n_swept_clear}/{n}  unreachable={n_none}/{n}")
        else:
            n_native = n_native_clear
            n_swept  = n_swept_clear
            print(f"  {pose_label:<12}  native={n_native}/{n}  swept={n_swept}/{n}  unreachable={n_none}/{n}")

    # Common clear angles across all cones and all poses
    all_sets = []
    for r in results:
        for pose_label, _ in POSE_CONFIGS:
            all_sets.append(set(r["poses"][pose_label]["reachable_clear_angles"]))
    common = sorted(all_sets[0].intersection(*all_sets[1:])) if all_sets else []
    label_str = "Angles clear (IK ok + no collision)" if collision_checking else "Angles reachable"
    print(f"\n  {label_str} for ALL cones at ALL poses: {common if common else 'None'}")

    print(f"\nVisualization frames added under '{VIZ_GROUP_NAME}' in the station tree.")

    # ── Save files ────────────────────────────────────────────────────────────
    os.makedirs(IK_SOLUTIONS_DIR, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    json_path = os.path.join(IK_SOLUTIONS_DIR, f"base_cone_ik_avoidance_{timestamp}.json")
    with open(json_path, "w") as f:
        json.dump({
            "generated":              timestamp,
            "j7_locked":              J7_LOCKED,
            "approach_offset_mm":     APPROACH_OFFSET_MM,
            "orient_sweep_steps":     ORIENT_SWEEP_STEPS,
            "solver":                 "OptimAxes Algorithm 3 DLS",
            "tool":                   TOOL_NAME,
            "robot":                  ROBOT_NAME,
            "pose_configs":           [l for l, _ in POSE_CONFIGS],
            "collision_checking":     collision_checking,
            "avoid_item_names":       AVOID_ITEM_NAMES,
            "avoid_all_collisions":   AVOID_ALL_COLLISIONS,
            "solutions":              results,
        }, f, indent=2)
    print(f"\nIK solutions (JSON) : {json_path}")

    txt_path = os.path.join(IK_SOLUTIONS_DIR, f"base_cone_summary_avoidance_{timestamp}.txt")
    save_summary_txt(results, angles, timestamp, txt_path, collision_checking)
    print(f"Summary (txt)       : {txt_path}")


if __name__ == "__main__":
    main()
