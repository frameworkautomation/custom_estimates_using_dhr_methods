"""
Interactive viewer for cached IK solutions from robert_end_checker.py.

Reads ik_results.json (produced by robert_end_checker.py), prompts the user
to pick an end effector and then a point, moves the robot to the solved
joint position in RoboDK, then resets. Respects the j7 constraints from
the original solve.

Usage:
    python robert_checker_stuff/robert_end_shower.py
    python robert_checker_stuff/robert_end_shower.py --robodk-ip 172.23.208.1
"""

import sys
import os
import json
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME
from robodk.robomath import Pose_2_TxyzRxyz

# ── CONFIG ────────────────────────────────────────────────────────────────────
ROBOT_NAMES  = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]
HOME_SEED    = [0.0] * 7

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
RESULTS_PATH = os.path.join(SCRIPT_DIR, "ik_results.json")
CONFIG_PATH  = os.path.join(SCRIPT_DIR, "robert_end_checker_config.json")


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


def is_quit(s):
    return s.lower() in ('q', 'quit', 'quit()', 'exit', '')


# ── MAIN ─────────────────────────────────────────────────────────────────────

def load_results():
    if not os.path.exists(RESULTS_PATH):
        print(f"[ERROR] No results file found at {RESULTS_PATH}")
        print("        Run robert_end_checker.py first to generate IK solutions.")
        sys.exit(1)
    with open(RESULTS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser(description="Interactively view IK solutions from robert_end_checker.py")
    ap.add_argument("--robodk-ip", default=None, help="RoboDK IP (default: localhost then 172.23.208.1)")
    args = ap.parse_args()

    results = load_results()
    ee_results = results.get("end_effectors", {})

    if not ee_results:
        print("[ERROR] No end effector results in ik_results.json.")
        return

    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)

    # Do NOT call setPoseFrame — it breaks the robot-rail connection in this station.
    # MoveJ with joint values works regardless of frame setting.

    ee_names = list(ee_results.keys())

    try:
        while True:
            # ── Pick end effector ────────────────────────────────────────
            print(f"\n{'=' * 60}")
            print("  END EFFECTORS")
            print(f"{'=' * 60}")
            for i, name in enumerate(ee_names):
                n_pts = len(ee_results[name])
                n_ok = sum(1 for r in ee_results[name].values() if r.get("reachable"))
                print(f"  ({i}) {name}  ({n_ok}/{n_pts} reachable)")

            ans = input("\n  Select end effector number (or 'q' to quit): ").strip()
            if is_quit(ans):
                break
            try:
                ee_idx = int(ans)
            except ValueError:
                print("  Enter a number or 'q'.")
                continue
            if not (0 <= ee_idx < len(ee_names)):
                print(f"  Out of range (0–{len(ee_names)-1}).")
                continue

            ee_name = ee_names[ee_idx]
            points = ee_results[ee_name]

            # Try to set tool in RoboDK
            tool = RDK.Item(ee_name, ITEM_TYPE_TOOL)
            if tool.Valid():
                robot.setTool(tool)
                print(f"  Tool '{ee_name}' set.")
            else:
                print(f"  [WARN] Tool '{ee_name}' not found in RoboDK, using current tool.")

            # ── Pick point loop ──────────────────────────────────────────
            point_names = list(points.keys())
            while True:
                print(f"\n  {'─' * 55}")
                print(f"  POINTS for '{ee_name}'")
                print(f"  {'─' * 55}")
                for i, pname in enumerate(point_names):
                    pr = points[pname]
                    reachable = pr.get("reachable", False)
                    status = "OK" if reachable else "FAIL"
                    tc = pr.get("special_track_conditions", {})
                    ctype = tc.get("type", "None")
                    j7_info = ""
                    if ctype == "Locked_at_j7_0":
                        j7_info = " j7=locked(0)"
                    elif ctype == "Locked_at_j7_pt":
                        j7_info = f" j7=locked({tc.get('j7_value', '?')})"
                    elif ctype == "Optimized_for_j7_at":
                        j7_info = f" j7=opt({tc.get('j7_value', '?')})"
                    j7_actual = ""
                    if reachable and len(pr.get('joints', [])) > 6:
                        j7_actual = f" j7_actual={pr['joints'][6]:.1f}"
                    print(f"  ({i:>3}) [{status:>4}] {pname}{j7_info}{j7_actual}")

                ans = input("\n  Select point number (or 'q' to go back): ").strip()
                if is_quit(ans):
                    break
                try:
                    pt_idx = int(ans)
                except ValueError:
                    print("  Enter a number or 'q'.")
                    continue
                if not (0 <= pt_idx < len(point_names)):
                    print(f"  Out of range (0–{len(point_names)-1}).")
                    continue

                pname = point_names[pt_idx]
                pr = points[pname]

                joints = pr["joints"]
                if not pr.get("reachable"):
                    err = pr.get("error", "IK failed")
                    print(f"\n  [WARN] '{pname}' is not reachable ({err}) — moving as close as possible")
                else:
                    print(f"\n  Moving to: {pname}")
                print(f"  Joints: {fmt_joints(joints)}")

                try:
                    # Find the RoboDK target to compare positions
                    from robodk.robolink import ITEM_TYPE_TARGET
                    target = RDK.Item(pname, ITEM_TYPE_TARGET)
                    if target.Valid():
                        tgt_xyz = Pose_2_TxyzRxyz(target.PoseAbs())
                        print(f"  Target XYZ (PoseAbs): ({tgt_xyz[0]:.1f}, {tgt_xyz[1]:.1f}, {tgt_xyz[2]:.1f}) mm")

                    print(f"  MoveJ type={type(joints).__name__} len={len(joints)}")
                    robot.MoveJ(joints)

                    # Get EE world position via SolveFK
                    fk_pose = robot.SolveFK(joints)
                    # fk_pose is flange in world. Get tool offset and compute TCP world.
                    from robodk.robolink import ITEM_TYPE_TOOL as _ITT
                    tool_item = robot.getLink(_ITT)
                    if tool_item.Valid():
                        tcp_world = fk_pose * tool_item.PoseTool()
                    else:
                        tcp_world = fk_pose
                    ee_xyzrpw = Pose_2_TxyzRxyz(tcp_world)
                    print(f"  EE World (FK*Tool):")
                    print(f"    XYZ: ({ee_xyzrpw[0]:.1f}, {ee_xyzrpw[1]:.1f}, {ee_xyzrpw[2]:.1f}) mm")
                    print(f"    RPW: ({ee_xyzrpw[3]:.2f}, {ee_xyzrpw[4]:.2f}, {ee_xyzrpw[5]:.2f}) deg")

                    if target.Valid():
                        tgt_full = Pose_2_TxyzRxyz(target.PoseAbs())
                        print(f"  Target (PoseAbs):")
                        print(f"    XYZ: ({tgt_full[0]:.1f}, {tgt_full[1]:.1f}, {tgt_full[2]:.1f}) mm")
                        print(f"    RPW: ({tgt_full[3]:.2f}, {tgt_full[4]:.2f}, {tgt_full[5]:.2f}) deg")
                        import math
                        err = math.sqrt(sum((tgt_full[i] - ee_xyzrpw[i])**2 for i in range(3)))
                        print(f"  Position error: {err:.1f} mm")
                except Exception as e:
                    print(f"  [ERROR] MoveJ failed: {e}")

                input("  Press Enter to return to home ...")
                robot.setJoints(HOME_SEED)

    finally:
        robot.setJoints(HOME_SEED)


if __name__ == "__main__":
    main()
