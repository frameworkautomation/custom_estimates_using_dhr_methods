"""
moving_a_cone.py

Simulates picking a cone from a base cone grab point and placing it at a
target cone position. Prompts for confirmation at every step.

Sequence:
  0. Close gripper (set MovingPart to closed_angle)
  1. Find base_cone_grab_* targets; load or compute IK + approach offsets
  2. Find cone_grab_* targets (under Cones > Cone_<N> > cone_grab_<N>); compute IK
  3. Prompt: base cone index, destination cone index (separate lists)
  4. Delete destination Cone_<N> from station (removes cone mesh + grab target)
  5. [Proceed?] MoveJ to base cone approach
  6. [Proceed?] MoveJ to base cone grab
  7. [Proceed?] MoveJ to destination cone approach
  8. [Proceed?] MoveJ to destination cone place
  9. [Proceed?] Return to home (all joints 0)
"""

import sys
import os
import json
import datetime

sys.path.append("C:/RoboDK/Python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT
from robodk.robomath import transl, invH, rotz
import tkinter as tk
from tkinter import messagebox

from test_reach_base_cone import custom_ik_pos_and_zaxis, pos_and_z, fmt_joints

# ── CONFIG ────────────────────────────────────────────────────────────────────
ROBOT_NAME         = "Fanuc R2000iC 125L"
TOOL_NAME          = "pickup_closed"
APPROACH_OFFSET_MM = 200.0    # offset along grab Z-axis for approach waypoint
J7_LOCKED          = 0.0      # rail position held fixed during all IK solves
HOME_SEED          = [0.0] * 7
POS_TOL_MM         = 0.5
ANGLE_TOL_DEG      = 2.0
MAX_ITERS          = 200
SPEED_J_DEG_S      = 200
SPEED_MM_S         = 200
IK_SOLUTIONS_DIR   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ik_solutions")
# ─────────────────────────────────────────────────────────────────────────────

pi = 3.141592653589793


# ── Helpers ───────────────────────────────────────────────────────────────────

def connect():
    try:
        rdk = Robolink()
        rdk.Item("")
        print("[INFO] Connected on localhost")
        return rdk
    except Exception:
        print("[INFO] localhost failed, trying 172.23.208.1 ...")
        return Robolink(robodk_ip="172.23.208.1")


def proceed(title, message):
    """Modal OK/Cancel dialog. Returns True to continue, False to abort."""
    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)
    result = messagebox.askokcancel(title, message, parent=root)
    root.destroy()
    return result


def make_approach_pose(grab_pose, offset_mm):
    """Offset grab_pose by offset_mm along its own local Z-axis."""
    return grab_pose * transl(0, 0, offset_mm)


def solve_ik(robot, pose, seed, label):
    """Run LM IK. Returns (joints, pos_err, angle_deg, converged)."""
    seed = list(seed)
    seed[6] = J7_LOCKED
    result, pos_err, angle_deg, converged = custom_ik_pos_and_zaxis(
        robot, pose, seed,
        pos_tol=POS_TOL_MM,
        angle_tol_deg=ANGLE_TOL_DEG,
        max_iters=MAX_ITERS,
        verbose=False,
    )
    tag = "SUCCESS" if converged else "FAIL   "
    print(f"    [{tag}] {label:<35} pos={pos_err:7.3f}mm  angle={angle_deg:6.3f}deg")
    return result, pos_err, angle_deg, converged


def find_moving_part(RDK):
    """Find the MovingPart gripper item and parse its angles from its name.

    Returns (moving, open_angle, closed_angle, import_angle, axis_offset).
    axis_offset is computed ONCE from the current pose (assumed to be at
    import_angle at this point).  Pass it to set_gripper_angle on every
    subsequent call so the computation is never repeated on an already-rotated
    pose.
    """
    moving = None
    for item in RDK.ItemList():
        if item.Name().startswith("MovingPart|"):
            moving = item
            break
    if moving is None:
        moving = RDK.Item("MovingPart")
    if not moving.Valid():
        print("[WARN] MovingPart not found — gripper animation skipped.")
        return None, None, None, None, None

    parts = moving.Name().split("|")
    angles = {}
    for part in parts[1:]:
        k, v = part.split("=")
        angles[k.strip()] = float(v.strip())

    import_angle = angles.get("import", 0.0)
    # Compute axis_offset once while the part is at its import pose.
    axis_offset = moving.Pose() * invH(rotz(import_angle * pi / 180.0))
    return moving, angles.get("open", 0.0), angles.get("closed", 0.0), import_angle, axis_offset


def set_gripper_angle(RDK, moving, axis_offset, import_angle, delta_deg):
    """Rotate MovingPart to (import_angle + delta_deg) using a pre-computed axis_offset."""
    if moving is None:
        return
    total_rad = (import_angle + delta_deg) * pi / 180.0
    moving.setPose(axis_offset * rotz(total_rad))
    RDK.Render()


def find_base_cones(RDK):
    """Return sorted list of all base_cone_grab_* targets (pickup sources)."""
    return sorted(
        [t for t in RDK.ItemList(ITEM_TYPE_TARGET)
         if t.Name().startswith("base_cone_grab_")],
        key=lambda t: t.Name(),
    )


def find_destination_cones(RDK):
    """Return sorted list of all cone_grab_* targets (placement destinations).
    These live under Cones > Cone_<N> > cone_grab_<N> in the station tree."""
    return sorted(
        [t for t in RDK.ItemList(ITEM_TYPE_TARGET)
         if t.Name().startswith("cone_grab_")],
        key=lambda t: t.Name(),
    )


def compute_all_offsets(RDK, robot, cone_targets):
    """Compute and print approach offset IK for all cone targets. Returns dict of results."""
    print("\nComputing IK for all cone targets...")
    print(f"  {'Cone':<28} {'Grab':>8} {'pos err':>9} {'ang err':>9}   {'Approach':>9} {'pos err':>9} {'ang err':>9}")
    print("  " + "-" * 86)

    all_results = {}
    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    try:
        for target in cone_targets:
            name = target.Name()
            grab_pose = target.PoseAbs()
            app_pose  = make_approach_pose(grab_pose, APPROACH_OFFSET_MM)

            grab_j, grab_pos_err, grab_angle, grab_ok = solve_ik(
                robot, grab_pose, HOME_SEED, f"{name} grab"
            )
            app_seed = grab_j if grab_ok else list(HOME_SEED)
            app_j, app_pos_err, app_angle, app_ok = solve_ik(
                robot, app_pose, app_seed, f"{name} approach"
            )

            gs  = "SUCCESS" if grab_ok else "FAIL"
            as_ = "SUCCESS" if app_ok  else "FAIL"
            print(
                f"  {name:<28} {gs:>8} {grab_pos_err:>8.3f}mm {grab_angle:>8.3f}deg"
                f"   {as_:>9} {app_pos_err:>8.3f}mm {app_angle:>8.3f}deg"
            )

            all_results[name] = {
                "grab_ok": grab_ok, "grab_joints": [float(v) for v in grab_j],
                "grab_pos_err": grab_pos_err, "grab_angle": grab_angle,
                "app_ok":  app_ok,  "app_joints":  [float(v) for v in app_j],
                "app_pos_err":  app_pos_err,  "app_angle":  app_angle,
            }
    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)
        robot.setJoints(HOME_SEED)

    return all_results


def _solve_ik(robot, pose):
    """Call robot.SolveIK and return a flat list of joints, or [] on failure.

    SolveIK returns a Mat (matrix object); len(Mat) gives the number of rows
    (always 1 for a solution row), not the number of joint values.  Use
    Mat.list() to get a flat list first.
    """
    result = robot.SolveIK(pose)
    try:
        joints = result.list()          # robodk Mat → flat list
    except AttributeError:
        joints = list(result)           # already a plain list
    if len(joints) >= 6:
        return joints
    return []


def compute_dest_ik(RDK, robot, dest_cones):
    """Solve IK for destination cones using RoboDK's built-in SolveIK.

    SolveIK handles j7 (rail) automatically — it returns a full 7-DOF solution.
    The robot's pose frame must be WorldFrame so PoseAbs() poses are interpreted
    in the correct coordinate system.  SolveIK returns a Mat; use .list() to
    get a flat joint list (len(Mat) gives rows=1, not the number of joints).

    Returns a dict keyed by cone name with the same schema as compute_all_offsets.
    """
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    print("\nComputing IK for destination cones (RoboDK SolveIK) ...")
    print(f"  {'Cone':<28} {'Grab':>8}   {'Approach':>9}")
    print("  " + "-" * 52)

    results = {}
    try:
        for target in dest_cones:
            name = target.Name()
            grab_pose = target.PoseAbs()
            app_pose  = make_approach_pose(grab_pose, APPROACH_OFFSET_MM)

            grab_j = _solve_ik(robot, grab_pose)
            grab_ok = len(grab_j) >= 6
            app_j  = _solve_ik(robot, app_pose)
            app_ok  = len(app_j) >= 6

            if not grab_ok:
                grab_j = list(HOME_SEED)
            if not app_ok:
                app_j = list(HOME_SEED)

            gs  = "SUCCESS" if grab_ok else "FAIL"
            as_ = "SUCCESS" if app_ok  else "FAIL"
            print(f"  {name:<28} {gs:>8}   {as_:>9}")

            results[name] = {
                "grab_ok":      grab_ok,
                "grab_joints":  [float(v) for v in grab_j],
                "grab_pos_err": 0.0,
                "grab_angle":   0.0,
                "app_ok":       app_ok,
                "app_joints":   [float(v) for v in app_j],
                "app_pos_err":  0.0,
                "app_angle":    0.0,
            }
    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)

    return results


def save_solutions(all_results):
    os.makedirs(IK_SOLUTIONS_DIR, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(IK_SOLUTIONS_DIR, f"moving_a_cone_ik_{timestamp}.json")
    with open(out_path, "w") as f:
        json.dump({
            "generated": timestamp,
            "j7_locked": J7_LOCKED,
            "approach_offset_mm": APPROACH_OFFSET_MM,
            "solutions": all_results,
        }, f, indent=2)
    print(f"\n[INFO] IK solutions saved to: {out_path}")


def load_latest_base_cone_ik():
    """Load the most recent base_cone_ik_*.json from ik_solutions/. Returns dict or None."""
    if not os.path.isdir(IK_SOLUTIONS_DIR):
        return None
    candidates = sorted(
        [f for f in os.listdir(IK_SOLUTIONS_DIR) if f.startswith("base_cone_ik_") and f.endswith(".json")],
        reverse=True,
    )
    if not candidates:
        return None
    path = os.path.join(IK_SOLUTIONS_DIR, candidates[0])
    with open(path) as f:
        data = json.load(f)
    # Convert solutions list to dict keyed by name
    solutions = data.get("solutions", [])
    if isinstance(solutions, list):
        result = {s["name"]: s for s in solutions}
    else:
        result = solutions
    print(f"[INFO] Loaded saved IK solutions from: {path}")
    print(f"       (generated {data.get('generated', '?')}, "
          f"j7={data.get('j7_locked', '?')} mm, "
          f"approach offset={data.get('approach_offset_mm', '?')} mm)")
    return result


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    RDK = connect()

    robot = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError(f"Robot '{ROBOT_NAME}' not found.")

    tool = RDK.Item(TOOL_NAME, ITEM_TYPE_TOOL)
    if tool.Valid():
        robot.setTool(tool)
        print(f"[INFO] Tool set to '{TOOL_NAME}'")
    else:
        all_tools = [i.Name() for i in RDK.ItemList(ITEM_TYPE_TOOL)]
        print(f"[WARN] Tool '{TOOL_NAME}' not found. Available: {all_tools}")

    # ── Step 0: close gripper ─────────────────────────────────────────────────
    moving, open_angle, closed_angle, import_angle, axis_offset = find_moving_part(RDK)
    if moving is not None:
        print(f"[INFO] Closing gripper to closed_angle={closed_angle} deg ...")
        set_gripper_angle(RDK, moving, axis_offset, import_angle, closed_angle)
        print("[INFO] Gripper closed.")

    # ── Step 1: find base cones, load or compute IK ──────────────────────────
    base_cones = find_base_cones(RDK)
    if not base_cones:
        raise RuntimeError("No base_cone_grab_* targets found in station.")

    print(f"\nBase cones (pickup sources) — {len(base_cones)} found:")
    for i, t in enumerate(base_cones):
        print(f"  [{i}] {t.Name()}")

    base_ik_map = load_latest_base_cone_ik()
    if base_ik_map is not None:
        missing = [t for t in base_cones if t.Name() not in base_ik_map]
        if missing:
            print(f"[INFO] {len(missing)} base cone(s) not in saved solutions — recomputing missing only.")
            new_ik = compute_all_offsets(RDK, robot, missing)
            base_ik_map.update(new_ik)
            save_solutions(base_ik_map)
        else:
            print(f"[INFO] All {len(base_cones)} base cones found in saved solutions — skipping IK recompute.")
            print(f"\n  {'Cone':<28} {'Grab':>8} {'pos err':>9} {'ang err':>9}   {'Approach':>9} {'pos err':>9} {'ang err':>9}")
            print("  " + "-" * 86)
            for t in base_cones:
                r = base_ik_map[t.Name()]
                gs  = "SUCCESS" if r["grab_ok"] else "FAIL"
                as_ = "SUCCESS" if r["app_ok"]  else "FAIL"
                print(
                    f"  {t.Name():<28} {gs:>8} {r['grab_pos_err']:>8.3f}mm {r['grab_angle']:>8.3f}deg"
                    f"   {as_:>9} {r['app_pos_err']:>8.3f}mm {r['app_angle']:>8.3f}deg"
                )
    else:
        print("[INFO] No saved IK solutions found — computing fresh.")
        base_ik_map = compute_all_offsets(RDK, robot, base_cones)
        save_solutions(base_ik_map)

    # ── Step 2: find destination cones, compute IK ───────────────────────────
    dest_cones = find_destination_cones(RDK)
    if not dest_cones:
        raise RuntimeError("No cone_grab_* targets found in station (expected under Cones > Cone_<N> > cone_grab_<N>).")

    print(f"\nDestination cones (placement targets) — {len(dest_cones)} found:")
    for i, t in enumerate(dest_cones):
        print(f"  [{i}] {t.Name()}")

    dest_ik_map = compute_dest_ik(RDK, robot, dest_cones)

    # ── Step 3: prompt for base and destination cone numbers ──────────────────
    print()
    while True:
        try:
            base_idx = int(input(f"Base cone number (0–{len(base_cones)-1}): ").strip())
            dest_idx = int(input(f"Destination cone number (0–{len(dest_cones)-1}): ").strip())
            if not (0 <= base_idx < len(base_cones)):
                print(f"[ERROR] Base cone index out of range (0–{len(base_cones)-1}).")
                continue
            if not (0 <= dest_idx < len(dest_cones)):
                print(f"[ERROR] Destination cone index out of range (0–{len(dest_cones)-1}).")
                continue
            break
        except ValueError:
            print("[ERROR] Enter an integer.")

    base_target = base_cones[base_idx]
    tgt_target  = dest_cones[dest_idx]
    base_name   = base_target.Name()
    tgt_name    = tgt_target.Name()

    print(f"\n[INFO] Base        : {base_name}")
    print(f"[INFO] Destination : {tgt_name}")

    base_ik = base_ik_map[base_name]
    tgt_ik  = dest_ik_map[tgt_name]

    if not base_ik["grab_ok"] or not base_ik["app_ok"]:
        raise RuntimeError(f"Base cone '{base_name}' IK did not converge — cannot proceed.")
    if not tgt_ik["grab_ok"] or not tgt_ik["app_ok"]:
        raise RuntimeError(f"Destination cone '{tgt_name}' IK did not converge — cannot proceed.")

    base_app_joints  = base_ik["app_joints"]
    base_grab_joints = base_ik["grab_joints"]
    tgt_app_joints   = tgt_ik["app_joints"]
    tgt_grab_joints  = tgt_ik["grab_joints"]

    # ── Step 4: delete destination Cone_<N> from station ─────────────────────
    # tgt_target is cone_grab_<N>; its parent is Cone_<N>; deleting the parent
    # removes both the cone geometry and the grab target.
    cone_parent = tgt_target.Parent()
    if cone_parent.Valid():
        print(f"[INFO] Deleting '{cone_parent.Name()}' (and its child '{tgt_name}') from station ...")
        cone_parent.Delete()
    else:
        print(f"[WARN] Could not find parent of '{tgt_name}' — skipping deletion.")

    # ── Motion sequence ───────────────────────────────────────────────────────
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)
    robot.setSpeed(speed_linear=SPEED_MM_S, speed_joints=SPEED_J_DEG_S)

    try:
        # Step 4: approach base cone
        if not proceed(
            "Step 1/4 — Move to base approach",
            f"Ready to move to APPROACH of:\n  {base_name}\n\n"
            f"Joints: {fmt_joints(base_app_joints)}\n\n"
            f"Approach offset: {APPROACH_OFFSET_MM:.0f} mm along grab Z-axis\n\n"
            "Click OK to proceed, Cancel to abort."
        ):
            print("[ABORT] User cancelled at base approach.")
            return
        lims = robot.JointLimits()
        lo = [float(lims[0][i, 0]) for i in range(7)]
        hi = [float(lims[1][i, 0]) for i in range(7)]
        print(f"[DEBUG] base_app_joints : {[round(v,3) for v in base_app_joints]}")
        print(f"[DEBUG] joint lo limits : {[round(v,3) for v in lo]}")
        print(f"[DEBUG] joint hi limits : {[round(v,3) for v in hi]}")
        for k, (v, l, h) in enumerate(zip(base_app_joints, lo, hi)):
            if v < l or v > h:
                print(f"[DEBUG] *** j{k+1}={v:.4f} OUTSIDE [{l:.4f}, {h:.4f}]")
        print(f"[INFO] Moving to base approach: {base_name} ...")
        robot.MoveJ(base_app_joints)
        print("[INFO] At base approach.")

        # Step 5: base cone grab
        if not proceed(
            "Step 2/4 — Move to base grab",
            f"Ready to move to GRAB position:\n  {base_name}\n\n"
            f"Joints: {fmt_joints(base_grab_joints)}\n\n"
            "Click OK to proceed, Cancel to abort."
        ):
            print("[ABORT] User cancelled at base grab.")
            return
        print(f"[INFO] Moving to base grab: {base_name} ...")
        robot.MoveJ(base_grab_joints)
        print("[INFO] At base grab.")

        # Step 6: target cone approach
        if not proceed(
            "Step 3/4 — Move to target approach",
            f"Ready to move to APPROACH of target:\n  {tgt_name}\n\n"
            f"Joints: {fmt_joints(tgt_app_joints)}\n\n"
            "Click OK to proceed, Cancel to abort."
        ):
            print("[ABORT] User cancelled at target approach.")
            return
        print(f"[INFO] Moving to target approach: {tgt_name} ...")
        robot.MoveJ(tgt_app_joints)
        print("[INFO] At target approach.")

        # Step 7: target cone place
        if not proceed(
            "Step 4/4 — Move to target place",
            f"Ready to move to PLACE position:\n  {tgt_name}\n\n"
            f"Joints: {fmt_joints(tgt_grab_joints)}\n\n"
            "Click OK to proceed, Cancel to abort."
        ):
            print("[ABORT] User cancelled at target place.")
            return
        print(f"[INFO] Moving to target place: {tgt_name} ...")
        robot.MoveJ(tgt_grab_joints)
        print("[INFO] At target place.")

        # Step 8: return home
        if not proceed(
            "Return to home",
            "Ready to return to HOME (all joints = 0).\n\n"
            "Click OK to proceed, Cancel to stay."
        ):
            print("[INFO] User chose to stay at target. Done.")
            return
        print("[INFO] Returning to home ...")
        robot.MoveJ(HOME_SEED)
        print("[INFO] At home.")

    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)

    print("\n[INFO] Done.")


if __name__ == "__main__":
    main()
