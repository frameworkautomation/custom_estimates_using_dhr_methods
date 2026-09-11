"""
Back bin reachability demo — build a RoboDK program that proves the robot
(6-DOF, no rail, j7=0) can reach the cone bin, pick up cone_bin_buffer,
and return home.

Creates:
  - Targets from each approach/retract frame (bin + gripper slot)
  - A home (transport) joint target
  - A main program "back_bin_demo" with MoveJ/MoveL instructions
  - Helper sub-programs for attach/detach of gripper and bin object

The full sequence (matching DHR's change_tool + move_task pattern):
  1. Home -> GrabbingGripperSlot (pick up gripper visual)
  2. Home -> Cone bin approach/grab/retract with object
  3. Home -> Cone bin approach/place object/retract (return bin)
  4. Home -> GrabbingGripperSlot (drop off gripper visual)
  5. Home

Usage:
    python robert_checker_stuff/back_bin_reachability_demo.py --robodk-ip 172.23.208.1 --use-current
"""

import sys
import os
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME,
    ITEM_TYPE_OBJECT, ITEM_TYPE_TARGET, ITEM_TYPE_PROGRAM,
    ITEM_TYPE_PROGRAM_PYTHON, ITEM_TYPE_FOLDER, INSTRUCTION_CALL_PROGRAM,
)
from robodk.robomath import Pose_2_TxyzRxyz

# ── CONFIG ──────────────────────────────────────────────────────────────────

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]

TOOL_CHANGER_NAME = "ToolChanger"
GRIPPER_TOOL_NAME = "GrabbingGripper"
GRIPPER_VISUAL_NAME = "GrabbingGripperVisual"
GRAB_OBJECT = "cone_bin_buffer"

# Gripper slot frames (DHR pattern: approach MoveJ, slot MoveL)
GRIPPER_SLOT_FRAME = "GrabbingGripperSlot"
GRIPPER_APPROACH_FRAME = "ApproachGrabbingGripperSlot"

# Bin movement sequence: (frame_name, move_type, label)
BIN_APPROACH_SEQUENCE = [
    ("ApproachConeBinBuffer",      "J", "bin_approach_coarse"),
    ("ApproachConeBinBufferBelow", "L", "bin_approach_below"),
    ("Cone_Bin_Frame",             "L", "bin_at_grab"),
]
BIN_RETRACT_SEQUENCE = [
    ("ApproachConeBinBufferUp",    "L", "bin_retract_up"),
    ("ApproachConeBinBuffer",      "L", "bin_retract_clear"),
]

# DHR's transport pose (6-DOF)
TRANSPORT_JOINTS = [0, -50, 15, 0, -15, -90]

PROGRAM_NAME = "back_bin_demo"
BIN_DEMO_FOLDER_NAME = "bin_demo"


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


def to_robodk_path(path):
    """Convert WSL /mnt/c/... path to C:/... for RoboDK."""
    abs_path = os.path.abspath(path)
    if abs_path.startswith("/mnt/"):
        parts = abs_path.split("/")
        drive = parts[2].upper()
        rest = "/".join(parts[3:])
        return f"{drive}:/{rest}"
    return abs_path


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def add_python_program(RDK, name, code):
    """Write a Python script to the project dir and add it to RoboDK as a program."""
    script_path = os.path.join(SCRIPT_DIR, f"_tmp_{name}.py")
    with open(script_path, "w", encoding="utf-8") as f:
        f.write(code)
    robodk_path = to_robodk_path(script_path)
    prog = RDK.AddFile(robodk_path)
    if prog.Valid():
        prog.setName(name)
    os.unlink(script_path)
    return prog


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

    robot_base = robot.Parent()
    assert robot_base.Valid(), "Robot has no valid parent frame"
    print(f"  Base:  {robot_base.Name()}")

    tool_changer = RDK.Item(TOOL_CHANGER_NAME, ITEM_TYPE_TOOL)
    assert tool_changer.Valid(), f"Tool '{TOOL_CHANGER_NAME}' not found"
    print(f"  ToolChanger: {tool_changer.Name()}")

    gripper_tool = RDK.Item(GRIPPER_TOOL_NAME, ITEM_TYPE_TOOL)
    gripper_visual = RDK.Item(GRIPPER_VISUAL_NAME, ITEM_TYPE_OBJECT)
    if gripper_tool.Valid():
        print(f"  Gripper tool: {gripper_tool.Name()}")
    assert gripper_visual.Valid(), f"Object '{GRIPPER_VISUAL_NAME}' not found"
    gripper_home_parent = gripper_visual.Parent().Name()
    print(f"  Gripper visual: {gripper_visual.Name()} (parent: {gripper_home_parent})")

    # Gripper slot frames
    gripper_slot = RDK.Item(GRIPPER_SLOT_FRAME, ITEM_TYPE_FRAME)
    assert gripper_slot.Valid(), f"Frame '{GRIPPER_SLOT_FRAME}' not found in station"
    print(f"  Frame: {GRIPPER_SLOT_FRAME} -> {describe_pose(gripper_slot.PoseWrt(robot_base))}")

    gripper_approach = RDK.Item(GRIPPER_APPROACH_FRAME, ITEM_TYPE_FRAME)
    assert gripper_approach.Valid(), f"Frame '{GRIPPER_APPROACH_FRAME}' not found in station"
    print(f"  Frame: {GRIPPER_APPROACH_FRAME} -> {describe_pose(gripper_approach.PoseWrt(robot_base))}")

    # Bin frames
    all_bin_steps = BIN_APPROACH_SEQUENCE + BIN_RETRACT_SEQUENCE
    all_bin_frame_names = set(s[0] for s in all_bin_steps)
    frames = {}
    for fname in all_bin_frame_names:
        f = RDK.Item(fname, ITEM_TYPE_FRAME)
        assert f.Valid(), f"Frame '{fname}' not found in station"
        print(f"  Frame: {fname} -> {describe_pose(f.PoseWrt(robot_base))}")
        frames[fname] = f

    grab_obj = RDK.Item(GRAB_OBJECT, ITEM_TYPE_OBJECT)
    assert grab_obj.Valid(), f"Object '{GRAB_OBJECT}' not found"
    grab_obj_home_parent = grab_obj.Parent().Name()
    print(f"  Object: {grab_obj.Name()} (parent: {grab_obj_home_parent})")

    print("\n[OK] All items found.")

    # ── Clean up old folder/programs if re-running ──────────────────────
    old_folder = RDK.Item(BIN_DEMO_FOLDER_NAME, ITEM_TYPE_FOLDER)
    if old_folder.Valid():
        old_folder.Delete()
        print(f"[CLEAN] Deleted old '{BIN_DEMO_FOLDER_NAME}' folder")

    helper_names = ["attach_gripper", "detach_gripper",
                    "grab_cone_bin_buffer", "release_cone_bin_buffer"]
    for prog_name in [PROGRAM_NAME] + helper_names:
        for ptype in [ITEM_TYPE_PROGRAM, ITEM_TYPE_PROGRAM_PYTHON]:
            old = RDK.Item(prog_name, ptype)
            if old.Valid():
                old.Delete()
                print(f"[CLEAN] Deleted old program '{prog_name}'")

    old_folder = RDK.Item("bin_demo_targets", ITEM_TYPE_FRAME)
    if old_folder.Valid():
        old_folder.Delete()
        print("[CLEAN] Deleted old bin_demo_targets folder")

    # ── Create station folder for all bin demo items ─────────────────
    RDK.Command("AddFolder", BIN_DEMO_FOLDER_NAME)
    demo_folder = RDK.Item(BIN_DEMO_FOLDER_NAME, ITEM_TYPE_FOLDER)
    assert demo_folder.Valid(), f"Failed to create folder '{BIN_DEMO_FOLDER_NAME}'"
    print(f"\n[FOLDER] Created '{BIN_DEMO_FOLDER_NAME}'")

    # ── Create targets ────────────────────────────────────────────────
    print("\n[TARGETS] Creating targets...")
    target_folder = RDK.AddFrame("bin_demo_targets")
    target_folder.setParent(demo_folder)
    robot.setPoseFrame(robot_base)

    # Home / transport target
    home_target = RDK.AddTarget("bin_home", target_folder, robot)
    home_target.setJoints(TRANSPORT_JOINTS)
    home_target.setAsJointTarget()
    print(f"  Created: bin_home (joints: {TRANSPORT_JOINTS})")

    # Gripper slot targets (use ToolChanger as active tool for these moves)
    gripper_slot_target = RDK.AddTarget("bin_gripper_slot", target_folder, robot)
    gripper_slot_target.setPose(gripper_slot.PoseWrt(robot_base))
    print(f"  Created: bin_gripper_slot")

    gripper_approach_target = RDK.AddTarget("bin_gripper_approach", target_folder, robot)
    gripper_approach_target.setPose(gripper_approach.PoseWrt(robot_base))
    print(f"  Created: bin_gripper_approach")

    # Bin targets
    targets = {}
    for fname in all_bin_frame_names:
        tname = f"bin_{fname}"
        pose = frames[fname].PoseWrt(robot_base)
        tgt = RDK.AddTarget(tname, target_folder, robot)
        tgt.setPose(pose)
        targets[fname] = tgt
        print(f"  Created: {tname}")

    # ── Create helper sub-programs (real setParentStatic calls) ─────────
    print("\n[PROGRAMS] Creating helper sub-programs...")

    helper_scripts = {
        "attach_gripper": f'''from robodk.robolink import Robolink, ITEM_TYPE_OBJECT, ITEM_TYPE_TOOL
RDK = Robolink()
visual = RDK.Item("{GRIPPER_VISUAL_NAME}", ITEM_TYPE_OBJECT)
tool_changer = RDK.Item("{TOOL_CHANGER_NAME}", ITEM_TYPE_TOOL)
visual.setParentStatic(tool_changer)
print("Attached {GRIPPER_VISUAL_NAME} to {TOOL_CHANGER_NAME}")
''',
        "detach_gripper": f'''from robodk.robolink import Robolink, ITEM_TYPE_OBJECT
RDK = Robolink()
visual = RDK.Item("{GRIPPER_VISUAL_NAME}", ITEM_TYPE_OBJECT)
slot = RDK.Item("{gripper_home_parent}")
visual.setParentStatic(slot)
print("Detached {GRIPPER_VISUAL_NAME} to {gripper_home_parent}")
''',
        "grab_cone_bin_buffer": f'''from robodk.robolink import Robolink, ITEM_TYPE_TOOL, ITEM_TYPE_OBJECT
RDK = Robolink()
obj = RDK.Item("{GRAB_OBJECT}", ITEM_TYPE_OBJECT)
gripper = RDK.Item("{GRIPPER_TOOL_NAME}", ITEM_TYPE_TOOL)
obj.setParentStatic(gripper)
print("Grabbed {GRAB_OBJECT}")
''',
        "release_cone_bin_buffer": f'''from robodk.robolink import Robolink, ITEM_TYPE_OBJECT
RDK = Robolink()
obj = RDK.Item("{GRAB_OBJECT}", ITEM_TYPE_OBJECT)
home = RDK.Item("{grab_obj_home_parent}")
obj.setParentStatic(home)
print("Released {GRAB_OBJECT} to {grab_obj_home_parent}")
''',
    }

    for name, code in helper_scripts.items():
        prog = add_python_program(RDK, name, code)
        assert prog.Valid(), f"Failed to create helper program '{name}'"
        prog.setParent(demo_folder)
        print(f"  Created: {name}")

    # ── Build main program ────────────────────────────────────────────
    print(f"\n[PROGRAM] Building '{PROGRAM_NAME}'...")
    prog = RDK.AddProgram(PROGRAM_NAME, robot)
    prog.setPoseFrame(robot_base)

    # ── Phase 1: Pick up gripper ──────────────────────────────────────
    prog.RunInstruction("# Phase 1: Pick up GrabbingGripper", 0)
    prog.setPoseTool(tool_changer)
    prog.MoveJ(home_target)
    print("  MoveJ -> home (ToolChanger)")

    prog.MoveJ(gripper_approach_target)
    print("  MoveJ -> gripper approach")

    prog.MoveL(gripper_slot_target)
    print("  MoveL -> gripper slot")

    prog.RunInstruction("attach_gripper", INSTRUCTION_CALL_PROGRAM)
    print("  Call -> attach_gripper")

    prog.MoveL(gripper_approach_target)
    print("  MoveL -> gripper approach (retract)")

    # ── Phase 2: Go to bin, grab object ───────────────────────────────
    prog.RunInstruction("# Phase 2: Approach bin and grab", 0)
    prog.setPoseTool(gripper_tool if gripper_tool.Valid() else tool_changer)
    prog.MoveJ(home_target)
    print("  MoveJ -> home (GrabbingGripper)")

    for fname, mtype, label in BIN_APPROACH_SEQUENCE:
        tgt = targets[fname]
        if mtype == "J":
            prog.MoveJ(tgt)
        else:
            prog.MoveL(tgt)
        print(f"  Move{mtype} -> {label}")

    prog.RunInstruction("grab_cone_bin_buffer", INSTRUCTION_CALL_PROGRAM)
    print("  Call -> grab_cone_bin_buffer")

    # ── Phase 3: Retract from bin ─────────────────────────────────────
    prog.RunInstruction("# Phase 3: Retract from bin", 0)
    for fname, mtype, label in BIN_RETRACT_SEQUENCE:
        tgt = targets[fname]
        if mtype == "J":
            prog.MoveJ(tgt)
        else:
            prog.MoveL(tgt)
        print(f"  Move{mtype} -> {label}")

    prog.MoveJ(home_target)
    prog.Pause(5000)
    print("  MoveJ -> home (pause 5s with object)")

    # ── Phase 4: Return to bin, place object ──────────────────────────
    prog.RunInstruction("# Phase 4: Return to bin and place object", 0)
    for fname, mtype, label in BIN_APPROACH_SEQUENCE:
        tgt = targets[fname]
        if mtype == "J":
            prog.MoveJ(tgt)
        else:
            prog.MoveL(tgt)
        print(f"  Move{mtype} -> {label} (return)")

    prog.RunInstruction("release_cone_bin_buffer", INSTRUCTION_CALL_PROGRAM)
    print("  Call -> release_cone_bin_buffer")

    for fname, mtype, label in BIN_RETRACT_SEQUENCE:
        tgt = targets[fname]
        if mtype == "J":
            prog.MoveJ(tgt)
        else:
            prog.MoveL(tgt)
        print(f"  Move{mtype} -> {label} (return retract)")

    prog.MoveJ(home_target)
    print("  MoveJ -> home")

    # ── Phase 5: Drop off gripper ─────────────────────────────────────
    prog.RunInstruction("# Phase 5: Return GrabbingGripper to slot", 0)

    prog.setPoseTool(tool_changer)
    prog.MoveJ(gripper_approach_target)
    print("  MoveJ -> gripper approach")

    prog.MoveL(gripper_slot_target)
    print("  MoveL -> gripper slot")

    prog.RunInstruction("detach_gripper", INSTRUCTION_CALL_PROGRAM)
    print("  Call -> detach_gripper")

    prog.MoveL(gripper_approach_target)
    print("  MoveL -> gripper approach (retract)")

    prog.MoveJ(home_target)
    print("  MoveJ -> home (done)")

    prog.setParent(demo_folder)

    n_ins = prog.InstructionCount()
    print(f"\n[DONE] Program '{PROGRAM_NAME}' created with {n_ins} instructions.")
    print("       Step through it in RoboDK: right-click -> Run step-by-step")
    print("       Helper sub-programs use setParentStatic to attach/detach meshes")


if __name__ == "__main__":
    main()
