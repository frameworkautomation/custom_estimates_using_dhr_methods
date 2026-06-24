"""
Reachability checker for Base_* waypoints in path_config.yaml.

Reads waypoints from path_config.yaml, builds poses from Cartesian data,
converts robot_local → world using robot_base_world offset, and uses
j7 field to decide whether the rail is locked or free.

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

import yaml
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET, ITEM_TYPE_FRAME
from robodk.robomath import transl, invH, rotz, Pose_2_TxyzRxyz, eye, Mat

from test_reach_base_cone import fmt_joints

pi = 3.141592653589793

# ── CONFIG ────────────────────────────────────────────────────────────────────
ROBOT_NAMES        = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]
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


def _nat_key_str(name):
    parts = re.split(r'(\d+)', name)
    return [int(p) if p.isdigit() else p.lower() for p in parts]


def build_pose(wp):
    """Build a RoboDK Mat from waypoint dict (x/y/z mm, rx/ry/rz degrees).
    GH exports ZYX Euler angles: R = Rz * Ry * Rx."""
    x, y, z = float(wp["x"]), float(wp["y"]), float(wp["z"])
    rx = math.radians(float(wp["rx"]))
    ry = math.radians(float(wp["ry"]))
    rz = math.radians(float(wp["rz"]))
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return Mat([
        [cy*cz,  cz*sx*sy - cx*sz,  cx*cz*sy + sx*sz,  x],
        [cy*sz,  cx*cz + sx*sy*sz,  cx*sy*sz - cz*sx,  y],
        [-sy,    cy*sx,             cx*cy,              z],
        [0,      0,                 0,                  1],
    ])


def build_robot_base_pose(config):
    """Build a translation Mat from robot_base_world in path_config."""
    rb = config.get("robot_base_world", {})
    return transl(float(rb.get("x", 0)), float(rb.get("y", 0)), float(rb.get("z", 0)))


def load_base_waypoints_from_yaml():
    """Load Base_* waypoints from path_config.yaml. Returns (config, waypoints_list)."""
    yaml_path = os.path.join(ROBODK_OUTPUT_DIR, "path_config.yaml")
    with open(yaml_path, "r", encoding="utf-8") as fh:
        config = yaml.safe_load(fh)

    raw_wps = config.get("waypoints") or {}
    waypoints = []
    for name, attrs in raw_wps.items():
        if not isinstance(attrs, dict):
            continue
        if not name.startswith("Base_"):
            continue
        if name.endswith("_approach"):
            continue
        # Must have Cartesian data
        if "x" not in attrs:
            continue
        wp = {"name": name, **attrs}
        waypoints.append(wp)

    waypoints.sort(key=lambda w: _nat_key_str(w["name"]))
    return config, waypoints


def compute_ik_from_yaml(RDK, robot, waypoints):
    """Compute IK for YAML waypoints. Uses j7 field to decide locked vs free.

    j7: null  → j7 locked at J7_LOCKED (base cones are robot-relative)
    j7: <num> → j7 locked at that value
    """
    current_tool = robot.getLink(ITEM_TYPE_TOOL)
    print(f"\n[IK] Tool: '{current_tool.Name() if current_tool.Valid() else 'None'}'")
    print(f"[IK] Approach offset: {APPROACH_OFFSET_MM} mm")
    print(f"[IK] Solver: OptimAxes Algorithm 3 (DLS)")

    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)
    RDK.Render(False)

    results = {}
    try:
        for wp in waypoints:
            name = wp["name"]
            # YAML coords are already in world space (GH exports world-space
            # values even though frame says robot_local). Use directly.
            grab_pose = build_pose(wp)

            app_pose = make_approach_pose(grab_pose, APPROACH_OFFSET_MM)

            # j7 handling: null → locked at J7_LOCKED, number → locked at that value
            j7_val = wp.get("j7")
            if j7_val is None:
                j7_lock = J7_LOCKED
                grab_j, grab_ok = solve_ik_static_j7(robot, grab_pose, f"{name} grab")
                app_j,  app_ok  = solve_ik_static_j7(robot, app_pose,  f"{name} approach")
                j7_mode = f"j7 locked at {j7_lock:.0f}mm"
            else:
                j7_lock = float(j7_val)
                # Use static j7 solver but override the lock value
                saved_lock = OPT_AXES_STATIC_J7.get("AbsJnt_7")
                OPT_AXES_STATIC_J7["AbsJnt_7"] = j7_lock
                grab_j, grab_ok = solve_ik_static_j7(robot, grab_pose, f"{name} grab")
                app_j,  app_ok  = solve_ik_static_j7(robot, app_pose,  f"{name} approach")
                OPT_AXES_STATIC_J7["AbsJnt_7"] = saved_lock
                j7_mode = f"j7 locked at {j7_lock:.0f}mm"

            results[name] = {
                "grab_ok": grab_ok, "grab_joints": [float(v) for v in grab_j],
                "app_ok":  app_ok,  "app_joints":  [float(v) for v in app_j],
                "j7_mode": j7_mode,
                "world_pose": grab_pose,
            }
    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)
        robot.setJoints(HOME_SEED)
        RDK.Render(True)

    return results


def print_table(title, waypoints, ik_map):
    """Print a summary table of IK results."""
    print(f"\n{'=' * 70}")
    print(f"  {title}")
    print(f"{'=' * 70}")

    print(f"  {'#':<4} {'Name':<35} {'Grab':>6} {'Appr':>6} {'j7(grab)':>10}")
    print("  " + "-" * 65)

    for i, wp in enumerate(waypoints):
        name = wp["name"]
        r = ik_map.get(name, {})
        gs  = "OK" if r.get("grab_ok") else "FAIL"
        as_ = "OK" if r.get("app_ok")  else "FAIL"
        if r.get("grab_ok"):
            j7_val = r["grab_joints"][6]
            print(f"  {i:<4} {name:<35} {gs:>6} {as_:>6} {j7_val:>9.1f}mm")
        else:
            print(f"  {i:<4} {name:<35} {gs:>6} {as_:>6}")

    n_both = sum(1 for wp in waypoints
                 if ik_map.get(wp["name"], {}).get("grab_ok")
                 and ik_map.get(wp["name"], {}).get("app_ok"))
    print(f"\n  {n_both}/{len(waypoints)} fully reachable (grab + approach)")


def interactive_examine(RDK, robot, waypoints, ik_map):
    """Let user pick positions to examine by number. Moves robot and pauses."""
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    saved_frame = robot.getLink(ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    print(f"\n{'=' * 70}")
    print("  INTERACTIVE EXAMINATION")
    print(f"{'=' * 70}")
    print("  Enter a number to move the robot to that position.")
    print("  Type 'q' to quit.\n")

    menu = []
    for wp in waypoints:
        name = wp["name"]
        r = ik_map.get(name, {})
        world_pose = r.get("world_pose")
        if r.get("grab_ok") and world_pose is not None:
            menu.append({
                "label": f"{name} (grab)",
                "joints": r["grab_joints"],
                "pose": world_pose,
            })
        if r.get("app_ok") and world_pose is not None:
            menu.append({
                "label": f"{name} (approach)",
                "joints": r["app_joints"],
                "pose": make_approach_pose(world_pose, APPROACH_OFFSET_MM),
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


def save_results(ik_map, tool_name, robot_name):
    """Save results to ik_solutions/. Strips non-serializable world_pose."""
    os.makedirs(IK_SOLUTIONS_DIR, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(IK_SOLUTIONS_DIR, f"reachability_check_{timestamp}.json")
    # Strip world_pose (Mat objects) before serializing
    serializable = {}
    for name, data in ik_map.items():
        serializable[name] = {k: v for k, v in data.items() if k != "world_pose"}
    with open(out_path, "w") as f:
        json.dump({
            "generated": timestamp,
            "robot": robot_name,
            "tool": tool_name,
            "approach_offset_mm": APPROACH_OFFSET_MM,
            "base_cones": serializable,
        }, f, indent=2)
    print(f"\n[INFO] Results saved to: {out_path}")


def main():
    ap = argparse.ArgumentParser(description="Check reachability of Base_* waypoints from path_config.yaml.")
    ap.add_argument("--tool", default=TOOL_NAME,
                    help=f"Tool for IK (default: {TOOL_NAME})")
    ap.add_argument("--recompute", action="store_true",
                    help="Force recompute (ignore any cached IK solutions)")
    ap.add_argument("--no-examine", action="store_true",
                    help="Skip interactive examination after printing the table")
    args = ap.parse_args()

    # ── Load waypoints from YAML ─────────────────────────────────────────────
    config, waypoints = load_base_waypoints_from_yaml()
    if not waypoints:
        print("[WARN] No Base_*_grab waypoints found in path_config.yaml.")
        return

    print(f"Found {len(waypoints)} Base_* grab waypoint(s) in path_config.yaml.")
    for wp in waypoints:
        j7 = wp.get("j7")
        j7_str = f"j7={j7}" if j7 is not None else "j7=locked(0)"
        print(f"  {wp['name']}  frame={wp.get('frame','world')}  {j7_str}")

    # ── Connect to RoboDK ────────────────────────────────────────────────────
    RDK = connect()

    robot = None
    for name in ROBOT_NAMES:
        r = RDK.Item(name, ITEM_TYPE_ROBOT)
        if r.Valid():
            robot = r
            print(f"[INFO] Found robot: '{name}'")
            break
    if robot is None:
        raise RuntimeError(f"Robot not found. Tried: {ROBOT_NAMES}")

    tool = RDK.Item(args.tool, ITEM_TYPE_TOOL)
    if tool.Valid():
        robot.setTool(tool)
        print(f"[INFO] Tool set to '{args.tool}'")
    else:
        all_tools = [i.Name() for i in RDK.ItemList(ITEM_TYPE_TOOL)]
        print(f"[WARN] Tool '{args.tool}' not found. Available: {all_tools}")

    # ── Compute IK ───────────────────────────────────────────────────────────
    ik_map = compute_ik_from_yaml(RDK, robot, waypoints)
    print_table("BASE WAYPOINTS", waypoints, ik_map)

    # ── Save results ─────────────────────────────────────────────────────────
    if ik_map:
        save_results(ik_map, args.tool, robot.Name())

    # ── Interactive examination ──────────────────────────────────────────────
    if not args.no_examine and ik_map:
        interactive_examine(RDK, robot, waypoints, ik_map)


if __name__ == "__main__":
    main()
