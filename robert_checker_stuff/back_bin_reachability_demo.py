"""
Back bin reachability demo — build a RoboDK program that proves the robot
(6-DOF, no rail, j7=0) can reach the cone bin, pick up cone_bin_buffer,
and return home.

Creates:
  - Targets from each approach/retract frame
  - A home (transport) joint target
  - A main program "back_bin_demo" with MoveJ/MoveL instructions
  - Helper sub-programs "grab_cone_bin_buffer" and "release_cone_bin_buffer"

The program can be stepped through in RoboDK's GUI.

Usage:
    python robert_checker_stuff/back_bin_reachability_demo.py --robodk-ip 172.23.208.1 --use-current
"""

import sys
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME,
    ITEM_TYPE_OBJECT, ITEM_TYPE_TARGET, ITEM_TYPE_PROGRAM,
    INSTRUCTION_CALL_PROGRAM,
)
from robodk.robomath import Pose_2_TxyzRxyz

# ── CONFIG ──────────────────────────────────────────────────────────────────

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]

TOOL_CHANGER_NAME = "ToolChanger"
GRIPPER_NAME = "GrabbingGripper"
GRAB_OBJECT = "cone_bin_buffer"

# Movement sequence: (frame_name, move_type, label)
# MoveJ for coarse approach/return, MoveL for precise moves near the bin
APPROACH_SEQUENCE = [
    ("ApproachConeBinBuffer",      "J", "approach_coarse"),
    ("ApproachConeBinBufferBelow", "L", "approach_below"),
    ("Cone_Bin_Frame",             "L", "at_bin"),
]
RETRACT_SEQUENCE = [
    ("ApproachConeBinBufferUp",    "L", "retract_up"),
    ("ApproachConeBinBuffer",      "L", "retract_clear"),
]

# DHR's transport pose (6-DOF)
TRANSPORT_JOINTS = [0, -50, 15, 0, -15, -90]

PROGRAM_NAME = "back_bin_demo"


# ── CONNECT ─────────────────────────────────────────────────────────────────

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
            return r
    return None


def describe_pose(pose):
    t = Pose_2_TxyzRxyz(pose)
    return f"x={t[0]:.1f} y={t[1]:.1f} z={t[2]:.1f}"


# ── MAIN ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Back bin reachability demo")
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    ap.add_argument("--use-current", action="store_true",
                    help="Use the currently open station")
    args = ap.parse_args()

    RDK = connect(args.robodk_ip)

    # ── Find items ────────────────────────────────────────────────────
    print("\n[FIND] Looking for items in station...")

    robot = find_robot(RDK)
    assert robot is not None, f"Robot not found. Tried: {ROBOT_NAMES}"
    print(f"  Robot: {robot.Name()}")

    robot_base = RDK.Item("RobotBase", ITEM_TYPE_FRAME)
    assert robot_base.Valid(), "RobotBase frame not found"
    print(f"  Base:  {robot_base.Name()}")

    tool_changer = RDK.Item(TOOL_CHANGER_NAME, ITEM_TYPE_TOOL)
    assert tool_changer.Valid(), f"Tool '{TOOL_CHANGER_NAME}' not found"
    print(f"  ToolChanger: {tool_changer.Name()}")

    gripper = RDK.Item(GRIPPER_NAME, ITEM_TYPE_TOOL)
    assert gripper.Valid(), f"Tool '{GRIPPER_NAME}' not found"
    print(f"  Gripper: {gripper.Name()}")

    # Collect all unique frame names from approach + retract
    all_steps = APPROACH_SEQUENCE + RETRACT_SEQUENCE
    all_frame_names = set(s[0] for s in all_steps)
    frames = {}
    for fname in all_frame_names:
        f = RDK.Item(fname, ITEM_TYPE_FRAME)
        assert f.Valid(), f"Frame '{fname}' not found in station"
        pose = f.PoseWrt(robot_base)
        print(f"  Frame: {fname} -> {describe_pose(pose)}")
        frames[fname] = f

    grab_obj = RDK.Item(GRAB_OBJECT, ITEM_TYPE_OBJECT)
    assert grab_obj.Valid(), f"Object '{GRAB_OBJECT}' not found"
    print(f"  Object: {grab_obj.Name()}")

    print("\n[OK] All items found.")

    # ── Attach GrabbingGripper to ToolChanger ─────────────────────────
    print("\n[SETUP] Attaching GrabbingGripper to ToolChanger...")
    gripper.setParent(tool_changer)
    robot.setTool(gripper)
    print(f"  GrabbingGripper parent: {gripper.Parent().Name()}")

    # ── Clean up old program/targets if re-running ────────────────────
    old_prog = RDK.Item(PROGRAM_NAME, ITEM_TYPE_PROGRAM)
    if old_prog.Valid():
        old_prog.Delete()
        print(f"[CLEAN] Deleted old program '{PROGRAM_NAME}'")

    old_folder = RDK.Item("bin_demo_targets", ITEM_TYPE_FRAME)
    if old_folder.Valid():
        old_folder.Delete()
        print("[CLEAN] Deleted old bin_demo_targets folder")

    for helper_name in ["grab_cone_bin_buffer", "release_cone_bin_buffer"]:
        old = RDK.Item(helper_name, ITEM_TYPE_PROGRAM)
        if old.Valid():
            old.Delete()
            print(f"[CLEAN] Deleted old program '{helper_name}'")

    # ── Create targets ────────────────────────────────────────────────
    print("\n[TARGETS] Creating targets...")
    target_folder = RDK.AddFrame("bin_demo_targets")
    robot.setPoseFrame(robot_base)

    # Home / transport target
    home_target = RDK.AddTarget("bin_home", target_folder, robot)
    home_target.setJoints(TRANSPORT_JOINTS)
    home_target.setAsJointTarget()
    print(f"  Created: bin_home (joints: {TRANSPORT_JOINTS})")

    # Targets from frames
    targets = {}
    for fname in all_frame_names:
        tname = f"bin_{fname}"
        pose = frames[fname].PoseWrt(robot_base)
        tgt = RDK.AddTarget(tname, target_folder, robot)
        tgt.setPose(pose)
        targets[fname] = tgt
        print(f"  Created: {tname} -> {describe_pose(pose)}")

    # ── Create helper sub-programs ────────────────────────────────────
    print("\n[PROGRAMS] Creating helper sub-programs...")

    grab_prog = RDK.AddProgram("grab_cone_bin_buffer", robot)
    grab_prog.RunInstruction(
        "# Attach cone_bin_buffer to GrabbingGripper (run manually or via script)",
        0,  # INSTRUCTION_COMMENT
    )
    print("  Created: grab_cone_bin_buffer")

    release_prog = RDK.AddProgram("release_cone_bin_buffer", robot)
    release_prog.RunInstruction(
        "# Release cone_bin_buffer back to original parent",
        0,  # INSTRUCTION_COMMENT
    )
    print("  Created: release_cone_bin_buffer")

    # ── Build main program ────────────────────────────────────────────
    print(f"\n[PROGRAM] Building '{PROGRAM_NAME}'...")
    prog = RDK.AddProgram(PROGRAM_NAME, robot)
    prog.setPoseFrame(robot_base)
    prog.setPoseTool(gripper)

    # 1. Start at home
    prog.MoveJ(home_target)
    print("  MoveJ -> bin_home")

    # 2. Approach sequence
    for fname, mtype, label in APPROACH_SEQUENCE:
        tgt = targets[fname]
        if mtype == "J":
            prog.MoveJ(tgt)
        else:
            prog.MoveL(tgt)
        print(f"  Move{mtype} -> {label} ({fname})")

    # 3. Grab
    prog.RunInstruction("grab_cone_bin_buffer", INSTRUCTION_CALL_PROGRAM)
    print("  Call -> grab_cone_bin_buffer")

    # 4. Retract sequence
    for fname, mtype, label in RETRACT_SEQUENCE:
        tgt = targets[fname]
        if mtype == "J":
            prog.MoveJ(tgt)
        else:
            prog.MoveL(tgt)
        print(f"  Move{mtype} -> {label} ({fname})")

    # 5. Return home
    prog.MoveJ(home_target)
    print("  MoveJ -> bin_home")

    n_ins = prog.InstructionCount()
    print(f"\n[DONE] Program '{PROGRAM_NAME}' created with {n_ins} instructions.")
    print("       Step through it in RoboDK: right-click -> Run step-by-step")
    print("       grab/release sub-programs are placeholders — run setParent manually or via script")


if __name__ == "__main__":
    main()
