"""
moving_a_cone.py

Simulates picking a cone from a base cone grab point and placing it at a
target cone position. Prompts for confirmation at every step.

Sequence:
  0. Close gripper (set MovingPart to closed_angle)
  1. Compute and display IK + approach offsets for ALL cone targets in station
  2. Prompt: base cone number, target cone number
  3. Solve IK for: base approach, base grab, target approach, target place
  4. Delete target cone object from station (simulates empty slot)
  5. [Proceed?] MoveJ to base cone approach
  6. [Proceed?] MoveJ to base cone grab
  7. [Proceed?] MoveJ to target cone approach
  8. [Proceed?] MoveJ to target cone place
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
    """Find the MovingPart gripper item and parse its angles from its name."""
    moving = None
    for item in RDK.ItemList():
        if item.Name().startswith("MovingPart|"):
            moving = item
            break
    if moving is None:
        moving = RDK.Item("MovingPart")
    if not moving.Valid():
        print("[WARN] MovingPart not found — gripper animation skipped.")
        return None, None, None, None

    parts = moving.Name().split("|")
    angles = {}
    for part in parts[1:]:
        k, v = part.split("=")
        angles[k.strip()] = float(v.strip())
    return moving, angles.get("open", 0.0), angles.get("closed", 0.0), angles.get("import", 0.0)


def set_gripper_angle(RDK, moving, import_angle, delta_deg):
    """Rotate MovingPart to (import_angle + delta_deg)."""
    if moving is None:
        return
    axis_offset = moving.Pose() * invH(rotz(import_angle * pi / 180.0))
    total_rad = (import_angle + delta_deg) * pi / 180.0
    moving.setPose(axis_offset * rotz(total_rad))
    RDK.Render()


def find_cone_targets(RDK):
    """Return sorted list of all base_cone_grab_* targets in the station."""
    return sorted(
        [t for t in RDK.ItemList(ITEM_TYPE_TARGET)
         if t.Name().startswith("base_cone_grab_")],
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
    moving, open_angle, closed_angle, import_angle = find_moving_part(RDK)
    if moving is not None:
        print(f"[INFO] Closing gripper to closed_angle={closed_angle} deg ...")
        set_gripper_angle(RDK, moving, import_angle, closed_angle)
        print("[INFO] Gripper closed.")

    # ── Step 1: find all cone targets, load or compute IK ────────────────────
    cone_targets = find_cone_targets(RDK)
    if not cone_targets:
        raise RuntimeError("No base_cone_grab_* targets found in station.")

    print(f"\nFound {len(cone_targets)} cone targets:")
    for i, t in enumerate(cone_targets):
        print(f"  [{i}] {t.Name()}")

    all_ik = load_latest_base_cone_ik()
    if all_ik is not None:
        # Check all targets are covered; recompute any missing ones
        missing = [t for t in cone_targets if t.Name() not in all_ik]
        if missing:
            print(f"[INFO] {len(missing)} target(s) not in saved solutions — recomputing missing only.")
            new_ik = compute_all_offsets(RDK, robot, missing)
            all_ik.update(new_ik)
            save_solutions(all_ik)
        else:
            print(f"[INFO] All {len(cone_targets)} targets found in saved solutions — skipping IK recompute.")
            # Still print the summary from saved data
            print(f"\n  {'Cone':<28} {'Grab':>8} {'pos err':>9} {'ang err':>9}   {'Approach':>9} {'pos err':>9} {'ang err':>9}")
            print("  " + "-" * 86)
            for t in cone_targets:
                r = all_ik[t.Name()]
                gs  = "SUCCESS" if r["grab_ok"] else "FAIL"
                as_ = "SUCCESS" if r["app_ok"]  else "FAIL"
                print(
                    f"  {t.Name():<28} {gs:>8} {r['grab_pos_err']:>8.3f}mm {r['grab_angle']:>8.3f}deg"
                    f"   {as_:>9} {r['app_pos_err']:>8.3f}mm {r['app_angle']:>8.3f}deg"
                )
    else:
        print("[INFO] No saved IK solutions found — computing fresh.")
        all_ik = compute_all_offsets(RDK, robot, cone_targets)
        save_solutions(all_ik)

    # ── Step 2: prompt for base and target cone numbers ───────────────────────
    print()
    while True:
        try:
            base_idx = int(input(f"Base cone number (0–{len(cone_targets)-1}): ").strip())
            tgt_idx  = int(input(f"Target cone number (0–{len(cone_targets)-1}): ").strip())
            if base_idx == tgt_idx:
                print("[ERROR] Base and target must be different.")
                continue
            if not (0 <= base_idx < len(cone_targets) and 0 <= tgt_idx < len(cone_targets)):
                print("[ERROR] Number out of range.")
                continue
            break
        except ValueError:
            print("[ERROR] Enter an integer.")

    base_target  = cone_targets[base_idx]
    tgt_target   = cone_targets[tgt_idx]
    base_name    = base_target.Name()
    tgt_name     = tgt_target.Name()

    print(f"\n[INFO] Base  : {base_name}")
    print(f"[INFO] Target: {tgt_name}")

    base_ik = all_ik[base_name]
    tgt_ik  = all_ik[tgt_name]

    if not base_ik["grab_ok"] or not base_ik["app_ok"]:
        raise RuntimeError(f"Base cone '{base_name}' IK did not converge — cannot proceed.")
    if not tgt_ik["grab_ok"] or not tgt_ik["app_ok"]:
        raise RuntimeError(f"Target cone '{tgt_name}' IK did not converge — cannot proceed.")

    base_app_joints  = base_ik["app_joints"]
    base_grab_joints = base_ik["grab_joints"]
    tgt_app_joints   = tgt_ik["app_joints"]
    tgt_grab_joints  = tgt_ik["grab_joints"]

    # ── Step 3: delete target cone object from station ────────────────────────
    # Try to find and delete an object associated with the target cone slot.
    # Looks for an item whose name contains the target number.
    tgt_number = tgt_name.replace("base_cone_grab_", "")
    deleted_any = False
    for item in RDK.ItemList():
        n = item.Name()
        if tgt_number in n and item.Type() == ITEM_TYPE_OBJECT:
            print(f"[INFO] Deleting target cone object: '{n}'")
            item.Delete()
            deleted_any = True
    if not deleted_any:
        print(f"[WARN] No object found matching target cone '{tgt_number}' — continuing without deletion.")

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
