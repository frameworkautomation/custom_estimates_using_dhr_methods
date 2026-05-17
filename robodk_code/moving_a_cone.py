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
import argparse
import math

sys.path.append("C:/RoboDK/Python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT
from robodk.robomath import transl, invH, rotz, Pose_2_TxyzRxyz
import tkinter as tk
from tkinter import messagebox

from test_reach_base_cone import fmt_joints

# ── CONFIG ────────────────────────────────────────────────────────────────────
ROBOT_NAME         = "Fanuc R2000iC 125L"
TOOL_NAME          = "pickup_closed"
APPROACH_OFFSET_MM = 200.0    # offset along grab Z-axis for approach waypoint
J7_LOCKED          = 0.0      # rail position held fixed during all IK solves
HOME_SEED          = [0.0] * 7
SPEED_J_DEG_S      = 200
SPEED_MM_S         = 200
IK_SOLUTIONS_DIR   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ik_solutions")
ROBODK_OUTPUT_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output")
GRIPPER_CACHE_PATH = os.path.join(ROBODK_OUTPUT_DIR, "gripper_axis_offset.json")

# OptimAxes parameters — mirrors DHR's approach.  RoboDK's Algorithm 3 (DLS)
# handles coupled joint limits (R2000iC J2/J3 interference zone) internally.
# OptimAxes for destination cones — j7 free to move to wherever the target needs it.
OPT_AXES_FREE_J7 = {
    "Algorithm": 3,
    "MaxIter":  500,
    "Tol":      0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
    "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
    "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
}

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
# ─────────────────────────────────────────────────────────────────────────────

pi = 3.141592653589793

_NO_POPUPS = False   # set True via --no-popups; makes proceed() auto-return True
_DEBUG_LOG = None    # open file handle written to throughout the run


def _log(msg):
    print(msg)
    if _DEBUG_LOG is not None:
        _DEBUG_LOG.write(msg + "\n")
        _DEBUG_LOG.flush()


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
    """Modal OK/Cancel dialog. Returns True to continue, False to abort.
    In ai mode always returns True. Falls back to a terminal y/n prompt if
    tkinter has no display (e.g. WSL without a GUI server)."""
    if _NO_POPUPS:
        _log(f"[AUTO ] {title}")
        return True
    try:
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        result = messagebox.askokcancel(title, message, parent=root)
        root.destroy()
        return result
    except Exception:
        # No display — fall back to terminal prompt
        print(f"\n{'─' * 60}")
        print(f"  {title}")
        print(f"{'─' * 60}")
        print(message)
        while True:
            ans = input("\n  Proceed? [y/n]: ").strip().lower()
            if ans in ("y", "yes", ""):
                return True
            if ans in ("n", "no"):
                return False


def _pose_xyz(pose):
    """Return (x, y, z) mm from a RoboDK Mat pose."""
    xyzrpw = Pose_2_TxyzRxyz(pose)
    return xyzrpw[0], xyzrpw[1], xyzrpw[2]


def do_move(robot, joints, label, expected_pose=None):
    """MoveJ to joints, log achieved position (and error vs expected_pose).
    Returns True on success, False on MoveJ exception."""
    try:
        robot.MoveJ(joints)
    except Exception as e:
        _log(f"[ERROR] MoveJ failed — {label}: {e}")
        _log(f"        joints: {[round(v, 3) for v in joints]}")
        return False

    achieved = robot.Pose()
    ax, ay, az = _pose_xyz(achieved)
    _log(f"[MOVE ] {label}")
    _log(f"         achieved  XYZ = ({ax:.2f}, {ay:.2f}, {az:.2f}) mm")

    if expected_pose is not None:
        tx, ty, tz = _pose_xyz(expected_pose)
        err = math.sqrt((tx - ax)**2 + (ty - ay)**2 + (tz - az)**2)
        _log(f"         expected  XYZ = ({tx:.2f}, {ty:.2f}, {tz:.2f}) mm")
        _log(f"         pos_err       = {err:.2f} mm")

    return True


def make_approach_pose(grab_pose, offset_mm):
    """Offset grab_pose by offset_mm along its own local Z-axis."""
    return grab_pose * transl(0, 0, offset_mm)


J7_TOL_MM = 10.0  # j7 must stay within this of J7_LOCKED to count as SUCCESS


def solve_ik(robot, pose, label):
    """Solve IK via RoboDK OptimAxes (Algorithm 3 DLS) with j7 constrained.

    Reports FAIL if MoveJ throws or if j7 drifts more than J7_TOL_MM from
    J7_LOCKED — the latter means the pose is unreachable with the rail fixed.

    Returns (joints, 0.0, 0.0, converged).
    """
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
            print(f"    [FAIL   ] {label}  (j7 drifted to {j7_actual:.1f}mm — unreachable with rail fixed)")
            return [0.0] * 7, 999.0, 999.0, False
        robot.setJoints(HOME_SEED)
        print(f"    [SUCCESS] {label}  j7={j7_actual:.1f}mm")
        return joints, 0.0, 0.0, True
    except Exception as e:
        robot.setJoints(HOME_SEED)
        print(f"    [FAIL   ] {label}  ({e})")
        return [0.0] * 7, 999.0, 999.0, False


def solve_ik_free_j7(robot, pose, label):
    """Solve IK via OptimAxes (Algorithm 3 DLS) with j7 free to move as needed.

    Used for destination cones where the rail moves to optimally reach the target.
    Returns (joints, converged).
    """
    robot.setParam("OptimAxes", OPT_AXES_FREE_J7)
    robot.setJoints(HOME_SEED)
    try:
        robot.MoveJ(pose)
        raw = robot.Joints()
        try:
            joints = raw.list()
        except AttributeError:
            joints = list(raw)
        robot.setJoints(HOME_SEED)
        print(f"    [SUCCESS] {label}  j7={joints[6]:.1f}mm")
        return joints, True
    except Exception as e:
        robot.setJoints(HOME_SEED)
        print(f"    [FAIL   ] {label}  ({e})")
        return [0.0] * 7, False


def find_moving_part(RDK):
    """Find the MovingPart gripper item and parse its angles from its name.

    Returns (moving, open_angle, closed_angle, import_angle, axis_offset).
    axis_offset is loaded from GRIPPER_CACHE_PATH if it exists (correct even
    when the part is no longer at import_angle), or computed fresh on the very
    first run (when the station has just loaded and the part IS at import_angle)
    and written to the cache for all future runs.
    """
    from robodk.robomath import Pose_2_TxyzRxyz, TxyzRxyz_2_Pose

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
    os.makedirs(ROBODK_OUTPUT_DIR, exist_ok=True)

    if os.path.isfile(GRIPPER_CACHE_PATH):
        with open(GRIPPER_CACHE_PATH) as f:
            cache = json.load(f)
        axis_offset = TxyzRxyz_2_Pose(cache["axis_offset_xyzrpw"])
        print(f"[INFO] Loaded gripper axis_offset from cache.")
    else:
        # First run: part assumed to be at import_angle (freshly loaded station).
        axis_offset = moving.Pose() * invH(rotz(import_angle * pi / 180.0))
        xyzrpw = list(Pose_2_TxyzRxyz(axis_offset))
        with open(GRIPPER_CACHE_PATH, "w") as f:
            json.dump({"axis_offset_xyzrpw": xyzrpw, "import_angle": import_angle}, f, indent=2)
        print(f"[INFO] Saved gripper axis_offset to cache: {GRIPPER_CACHE_PATH}")

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
    """Compute IK for base cone targets using RoboDK OptimAxes. Returns dict of results."""
    print("\nComputing IK for base cone targets (RoboDK OptimAxes, j7 constrained) ...")
    print(f"  {'Cone':<28} {'Grab':>8}   {'Approach':>9}")
    print("  " + "-" * 52)

    all_results = {}
    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    try:
        for target in cone_targets:
            name = target.Name()
            grab_pose = target.PoseAbs()
            app_pose  = make_approach_pose(grab_pose, APPROACH_OFFSET_MM)

            grab_j, _, _, grab_ok = solve_ik(robot, grab_pose, f"{name} grab")
            app_j,  _, _, app_ok  = solve_ik(robot, app_pose,  f"{name} approach")

            gs  = "SUCCESS" if grab_ok else "FAIL"
            as_ = "SUCCESS" if app_ok  else "FAIL"
            print(f"  {name:<28} {gs:>8}   {as_:>9}")

            all_results[name] = {
                "grab_ok": grab_ok, "grab_joints": [float(v) for v in grab_j],
                "grab_pos_err": 0.0, "grab_angle": 0.0,
                "app_ok":  app_ok,  "app_joints":  [float(v) for v in app_j],
                "app_pos_err":  0.0, "app_angle":  0.0,
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
    """Solve IK for destination cones using OptimAxes (Algorithm 3 DLS), j7 free.

    The rail moves to whatever position is needed to reach each destination.
    SolveIK was replaced because it returned out-of-limits branches (e.g. j3=200°)
    that caused MoveJ to silently clamp and land at the wrong position.

    Returns a dict keyed by cone name with the same schema as compute_all_offsets.
    """
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    print("\nComputing IK for destination cones (OptimAxes, j7 free) ...")
    print(f"  {'Cone':<28} {'Grab':>8}   {'Approach':>9}")
    print("  " + "-" * 52)

    results = {}
    try:
        for target in dest_cones:
            name = target.Name()
            grab_pose = target.PoseAbs()
            app_pose  = make_approach_pose(grab_pose, APPROACH_OFFSET_MM)

            grab_j, grab_ok = solve_ik_free_j7(robot, grab_pose, f"{name} grab")
            app_j,  app_ok  = solve_ik_free_j7(robot, app_pose,  f"{name} approach")

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
    """Load the most recent base_cone_ik_*.json from ik_solutions/.

    Handles both formats:
      - Old flat format: each entry has grab_ok, grab_joints, app_ok, app_joints
      - New nested format: each entry has a "poses" dict with "grab" and "approach" sub-dicts

    Returns a dict keyed by cone name with normalised keys:
      grab_ok, grab_joints, grab_pos_err, grab_angle,
      app_ok,  app_joints,  app_pos_err,  app_angle
    Returns None if no file found.
    """
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
    print(f"[INFO] Loaded saved IK solutions from: {path}")
    print(f"       (generated {data.get('generated', '?')}, "
          f"j7={data.get('j7_locked', '?')} mm, "
          f"approach offset={data.get('approach_offset_mm', '?')} mm)")

    solutions = data.get("solutions", [])
    result = {}

    for s in solutions if isinstance(solutions, list) else solutions.values():
        name = s["name"] if isinstance(s, dict) and "name" in s else s

        if "poses" in s:
            # New nested format from refactored checker
            grab = s["poses"].get("grab", {})
            app  = s["poses"].get("approach", {})

            grab_ok = grab.get("native_ok", False) or grab.get("swept_ok", False)
            grab_j  = (grab.get("native_joints") if grab.get("native_ok")
                       else grab.get("best_joints")) or [0.0] * 7

            app_ok = app.get("native_ok", False) or app.get("swept_ok", False)
            app_j  = (app.get("native_joints") if app.get("native_ok")
                      else app.get("best_joints")) or [0.0] * 7

            result[name] = {
                "grab_ok":      grab_ok,
                "grab_joints":  grab_j,
                "grab_pos_err": grab.get("native_pos_err", 0.0),
                "grab_angle":   0.0,
                "app_ok":       app_ok,
                "app_joints":   app_j,
                "app_pos_err":  app.get("native_pos_err", 0.0),
                "app_angle":    0.0,
            }
        else:
            # Old flat format — use as-is
            result[name] = s

    return result


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global _NO_POPUPS, _DEBUG_LOG

    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["human", "ai"], default="human",
                    help="human (default): prompts + confirmation dialogs. "
                         "ai: no prompts, uses --base/--dest, auto-proceeds all moves.")
    ap.add_argument("--base", type=int, default=None, metavar="N",
                    help="Base cone index. Required in ai mode; prompts in human mode.")
    ap.add_argument("--dest", type=int, default=None, metavar="N",
                    help="Destination cone index. Required in ai mode; prompts in human mode.")
    args = ap.parse_args()

    if args.mode == "ai":
        _NO_POPUPS = True
        if args.base is None:
            args.base = 0
        if args.dest is None:
            args.dest = 0

    os.makedirs(ROBODK_OUTPUT_DIR, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    debug_path = os.path.join(ROBODK_OUTPUT_DIR, f"move_debug_{ts}.txt")
    _DEBUG_LOG = open(debug_path, "w")
    _log(f"[DEBUG] Log opened: {debug_path}")
    _log(f"[DEBUG] no_popups={_NO_POPUPS}  base={args.base}  dest={args.dest}")

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

    if base_cones:
        print(f"\nBase cones (pickup sources) — {len(base_cones)} found in station:")
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
                print(f"\n  {'Cone':<28} {'Grab':>8} {'pos err':>9}   {'Approach':>9} {'pos err':>9}")
                print("  " + "-" * 70)
                for t in base_cones:
                    r = base_ik_map[t.Name()]
                    gs  = "SUCCESS" if r["grab_ok"] else "FAIL"
                    as_ = "SUCCESS" if r["app_ok"]  else "FAIL"
                    print(
                        f"  {t.Name():<28} {gs:>8} {r['grab_pos_err']:>8.3f}mm"
                        f"   {as_:>9} {r['app_pos_err']:>8.3f}mm"
                    )
        else:
            print("[INFO] No saved IK solutions found — computing fresh.")
            base_ik_map = compute_all_offsets(RDK, robot, base_cones)
            save_solutions(base_ik_map)

        cone_names = [t.Name() for t in base_cones]

    else:
        print("[WARN] No base_cone_grab_* targets in station — falling back to saved IK JSON.")
        base_ik_map = load_latest_base_cone_ik()
        if base_ik_map is None:
            raise RuntimeError(
                "No base_cone_grab_* targets in station and no saved IK JSON in ik_solutions/."
            )
        cone_names = sorted(base_ik_map.keys())
        print(f"\nBase cones (from saved IK) — {len(cone_names)} found:")
        print(f"  {'Cone':<28} {'Grab':>8}   {'Approach':>9}")
        print("  " + "-" * 52)
        for i, name in enumerate(cone_names):
            r = base_ik_map[name]
            gs  = "SUCCESS" if r["grab_ok"] else "FAIL"
            as_ = "SUCCESS" if r["app_ok"]  else "FAIL"
            print(f"  [{i}] {name:<28} {gs:>8}   {as_:>9}")

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
    if args.base is not None and args.dest is not None:
        base_idx = args.base
        dest_idx = args.dest
        _log(f"[AUTO ] base={base_idx}  dest={dest_idx}")
    else:
        while True:
            try:
                base_idx = int(input(f"Base cone number (0–{len(cone_names)-1}): ").strip())
                dest_idx = int(input(f"Destination cone number (0–{len(dest_cones)-1}): ").strip())
                if not (0 <= base_idx < len(cone_names)):
                    print(f"[ERROR] Base cone index out of range (0–{len(cone_names)-1}).")
                    continue
                if not (0 <= dest_idx < len(dest_cones)):
                    print(f"[ERROR] Destination cone index out of range (0–{len(dest_cones)-1}).")
                    continue
                if not (dest_ik_map[dest_cones[dest_idx].Name()]["grab_ok"] and
                        dest_ik_map[dest_cones[dest_idx].Name()]["app_ok"]):
                    print(f"[ERROR] '{dest_cones[dest_idx].Name()}' IK failed — pick a SUCCESS cone.")
                    continue
                break
            except ValueError:
                print("[ERROR] Enter an integer.")

    base_name  = cone_names[base_idx]
    tgt_target = dest_cones[dest_idx]
    tgt_name   = tgt_target.Name()

    print(f"\n[INFO] Base        : {base_name}")
    print(f"[INFO] Destination : {tgt_name}")

    base_ik = base_ik_map.get(base_name)
    if base_ik is None:
        raise RuntimeError(f"No IK solution found for base cone '{base_name}'.")
    tgt_ik  = dest_ik_map[tgt_name]

    if not base_ik["grab_ok"] or not base_ik["app_ok"]:
        raise RuntimeError(f"Base cone '{base_name}' IK did not converge — cannot proceed.")
    if not tgt_ik["grab_ok"] or not tgt_ik["app_ok"]:
        raise RuntimeError(f"Destination cone '{tgt_name}' IK did not converge — cannot proceed.")

    base_app_joints  = base_ik["app_joints"]
    base_grab_joints = base_ik["grab_joints"]
    tgt_app_joints   = tgt_ik["app_joints"]
    tgt_grab_joints  = tgt_ik["grab_joints"]

    # Log what the IK says the target positions should be
    tgt_grab_pose = tgt_target.PoseAbs()
    tgt_app_pose  = make_approach_pose(tgt_grab_pose, APPROACH_OFFSET_MM)
    gx, gy, gz = _pose_xyz(tgt_grab_pose)
    ax, ay, az = _pose_xyz(tgt_app_pose)
    _log(f"\n[DEBUG] {tgt_name} grab  target XYZ = ({gx:.2f}, {gy:.2f}, {gz:.2f}) mm")
    _log(f"[DEBUG] {tgt_name} approach target XYZ = ({ax:.2f}, {ay:.2f}, {az:.2f}) mm")
    _log(f"[DEBUG] tgt_grab_joints  = {[round(v,3) for v in tgt_grab_joints]}")
    _log(f"[DEBUG] tgt_app_joints   = {[round(v,3) for v in tgt_app_joints]}")

    # ── Step 4: delete destination cone mesh from station ────────────────────
    # Tree: Cones → cone_<x> (frame) → cone_<x> (OBJECT mesh)   ← delete this
    #                                 → cone_grab_<x> (TARGET)   ← keep this
    cone_frame = tgt_target.Parent()
    if cone_frame.Valid():
        mesh = next(
            (c for c in cone_frame.Childs() if c.Type() == ITEM_TYPE_OBJECT),
            None,
        )
        if mesh is not None and mesh.Valid():
            print(f"[INFO] Deleting cone mesh '{mesh.Name()}' from station ...")
            mesh.Delete()
        else:
            print(f"[WARN] No mesh object found under '{cone_frame.Name()}' — nothing deleted.")
    else:
        print(f"[WARN] Could not find parent frame of '{tgt_name}' — skipping deletion.")

    # ── Motion sequence ───────────────────────────────────────────────────────
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)
    robot.setSpeed(speed_linear=SPEED_MM_S, speed_joints=SPEED_J_DEG_S)

    try:
        # Step 1/4: base cone approach
        if not proceed(
            "Step 1/4 — Move to base approach",
            f"Ready to move to APPROACH of:\n  {base_name}\n\n"
            f"Joints: {fmt_joints(base_app_joints)}\n\n"
            f"Approach offset: {APPROACH_OFFSET_MM:.0f} mm along grab Z-axis\n\n"
            "Click OK to proceed, Cancel to abort."
        ):
            _log("[ABORT] User cancelled at base approach.")
            return
        if not do_move(robot, base_app_joints, f"base approach ({base_name})"):
            return

        # Step 2/4: base cone grab
        if not proceed(
            "Step 2/4 — Move to base grab",
            f"Ready to move to GRAB position:\n  {base_name}\n\n"
            f"Joints: {fmt_joints(base_grab_joints)}\n\n"
            "Click OK to proceed, Cancel to abort."
        ):
            _log("[ABORT] User cancelled at base grab.")
            return
        if not do_move(robot, base_grab_joints, f"base grab ({base_name})"):
            return

        # Step 3/4: target cone approach
        if not proceed(
            "Step 3/4 — Move to target approach",
            f"Ready to move to APPROACH of target:\n  {tgt_name}\n\n"
            f"Joints: {fmt_joints(tgt_app_joints)}\n\n"
            "Click OK to proceed, Cancel to abort."
        ):
            _log("[ABORT] User cancelled at target approach.")
            return
        if not do_move(robot, tgt_app_joints, f"target approach ({tgt_name})",
                       expected_pose=tgt_app_pose):
            return

        # Step 4/4: target cone place
        if not proceed(
            "Step 4/4 — Move to target place",
            f"Ready to move to PLACE position:\n  {tgt_name}\n\n"
            f"Joints: {fmt_joints(tgt_grab_joints)}\n\n"
            "Click OK to proceed, Cancel to abort."
        ):
            _log("[ABORT] User cancelled at target place.")
            return
        if not do_move(robot, tgt_grab_joints, f"target place ({tgt_name})",
                       expected_pose=tgt_grab_pose):
            return

        # Return home
        if not proceed(
            "Return to home",
            "Ready to return to HOME (all joints = 0).\n\n"
            "Click OK to proceed, Cancel to stay."
        ):
            _log("[INFO] User chose to stay at target. Done.")
            return
        do_move(robot, HOME_SEED, "home")
        _log("\n[INFO] Done.")

    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)
        if _DEBUG_LOG is not None:
            _DEBUG_LOG.close()


if __name__ == "__main__":
    main()
