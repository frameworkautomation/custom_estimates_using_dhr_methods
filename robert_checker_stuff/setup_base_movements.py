"""
Build movement sequences for base cone pickup in the extracted station.

Expects organize_station.py to have been run first (folders, programs,
offset targets all exist under Offset_relative_to_schematic).

Phase 1 — create empty programs for each grab/string_grab pair

Phase 2 — create offset targets for each grab and string_grab:
  - offset_before_for_<name> — offset along -Z before the target (approach)
  - offset_after_for_<name>  — offset along -Z after the target (retract)

Phase 3 — solve IK for all targets with Z-axis rotation sweep:
  - Try original pose first; if it fails, sweep Z rotations per config step
  - j2/j3 locked near default configuration

Phase 4 — build targets_to_use folder + JSON

Phase 5 — create per-cone attach scripts

Phase 6 — create final pullaway targets

Phase 7 — create obstacle avoidance targets:
  - approach_pullaway, approach_pull_in, retract_pullaway, bl_retract_pullaway
  - All in created_for_obstacle_avoidance folder
  - IK seeded from offset solutions

Phase 8 — populate programs with movement instructions:
  - 5 home positions (j1=0, ±170, ±180) — picks closest for start and end
  - Creates 'main' program that runs all cone programs + replace_cone

Use --rapid-test-mode to only process one cone (first found).

Reads settings from setup_base_movements_config.json (same directory).

Usage:
    python robert_checker_stuff/setup_base_movements.py
    python robert_checker_stuff/setup_base_movements.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/setup_base_movements.py --robodk-ip 172.23.208.1 --skip 1 2
"""

import sys
import os
import re
import json
import math
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_TARGET, ITEM_TYPE_OBJECT,
    ITEM_TYPE_FOLDER, ITEM_TYPE_PROGRAM, ITEM_TYPE_PROGRAM_PYTHON,
    ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME,
    INSTRUCTION_CALL_PROGRAM,
)
from robodk.robomath import transl, rotz, Pose_2_TxyzRxyz, Mat
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(SCRIPT_DIR, "setup_base_movements_config.json")
REPORT_PATH = os.path.join(SCRIPT_DIR, "ik_failure_report.txt")
TARGETS_TO_USE_PATH = os.path.join(SCRIPT_DIR, "targets_to_use.json")
CONE_POSES_PATH = os.path.join(SCRIPT_DIR, "cone_original_poses.json")

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]

CONE_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+$")
GRAB_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+_grab$")
STRING_GRAB_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+_string_grab$")

OFFSET_FRAME_NAME = "Offset_relative_to_schematic"


# ── CONFIG ────────────────────────────────────────────────────────────────────

def load_config(path):
    assert os.path.exists(path), f"Config not found: {path}"
    with open(path, "r", encoding="utf-8") as f:
        config = json.load(f)

    step = config.get("z_rotation_step_deg")
    assert step is not None, "Config missing 'z_rotation_step_deg'"
    assert isinstance(step, (int, float)) and step > 0, \
        f"z_rotation_step_deg must be a positive number, got {step}"

    offsets = config.get("offsets_mm")
    assert offsets is not None, "Config missing 'offsets_mm'"
    for key in ("grab", "string_grab"):
        assert key in offsets, f"offsets_mm missing '{key}'"
        for direction in ("before", "after"):
            assert direction in offsets[key], f"offsets_mm.{key} missing '{direction}'"

    ee = config.get("end_effectors")
    assert ee is not None, "Config missing 'end_effectors'"
    assert "grab" in ee, "end_effectors missing 'grab'"
    assert "string_grab" in ee, "end_effectors missing 'string_grab'"

    print(f"[CONFIG] z_rotation_step_deg = {step}")
    print(f"[CONFIG] end_effectors: grab={ee['grab']}, string_grab={ee['string_grab']}")
    return config


# ── CONNECT ───────────────────────────────────────────────────────────────────

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
    raise RuntimeError(f"Robot not found. Tried: {ROBOT_NAMES}")


def find_tool(RDK, name):
    tool = RDK.Item(name, ITEM_TYPE_TOOL)
    assert tool.Valid(), f"Tool '{name}' not found in station"
    return tool


# ── FOLDERS ───────────────────────────────────────────────────────────────────

def get_or_create_folder(RDK, name, parent=None):
    """Return existing folder named `name`, or create one.

    When parent is given, checks children of parent first to handle duplicate
    folder names at different levels.
    """
    if parent is not None:
        for child in parent.Childs():
            if child.Name() == name and child.Type() == ITEM_TYPE_FOLDER:
                return child

        existing_ids = {f.item for f in RDK.ItemList(ITEM_TYPE_FOLDER)}
        RDK.Command("AddFolder", name)

        folder = None
        for f in RDK.ItemList(ITEM_TYPE_FOLDER):
            if f.item not in existing_ids and f.Name() == name:
                folder = f
                break

        assert folder is not None, f"Failed to create folder '{name}'"
        folder.setParent(parent)
        print(f"[CREATE] Created folder '{name}' under '{parent.Name()}'")
        return folder

    existing = RDK.Item(name, ITEM_TYPE_FOLDER)
    if existing.Valid():
        return existing

    RDK.Command("AddFolder", name)
    folder = RDK.Item(name, ITEM_TYPE_FOLDER)
    assert folder.Valid(), f"Failed to create folder '{name}'"
    print(f"[CREATE] Created folder '{name}'")
    return folder


# ── HELPERS ───────────────────────────────────────────────────────────────────

def discover_cone_names(RDK):
    all_targets = RDK.ItemList(ITEM_TYPE_TARGET)
    target_names = {t.Name() for t in all_targets}

    grab_names = {n for n in target_names if GRAB_PATTERN.match(n)}
    string_grab_names = {n for n in target_names if STRING_GRAB_PATTERN.match(n)}

    grab_cones = {n.rsplit("_grab", 1)[0] for n in grab_names}
    string_grab_cones = {n.rsplit("_string_grab", 1)[0] for n in string_grab_names}

    paired = sorted(grab_cones & string_grab_cones)
    return paired


def find_target_in_folder(folder, name):
    for child in folder.Childs():
        if child.Name() == name and child.Type() == ITEM_TYPE_TARGET:
            return child
    return None


def target_exists_in_folder(folder, name):
    for child in folder.Childs():
        if child.Name() == name and child.Type() == ITEM_TYPE_TARGET:
            return True
    return False


# ── PHASE 1: OFFSET TARGETS ─────────────────────────────────────────────────

def create_offset_target(RDK, robot, source_target, offset_name, offset_mm, folder):
    existing = RDK.Item(offset_name, ITEM_TYPE_TARGET)
    if existing.Valid():
        return existing, False

    source_pose = source_target.Pose()
    offset_pose = source_pose * transl(0, 0, -offset_mm)

    target = RDK.AddTarget(offset_name, folder, robot)
    target.setPose(offset_pose)
    return target, True


def create_offset_targets(RDK, robot, offsets_config, before_folder, after_folder):
    created = 0
    skipped = 0

    offset_rules = [
        (GRAB_PATTERN, offsets_config["grab"]),
        (STRING_GRAB_PATTERN, offsets_config["string_grab"]),
    ]

    all_targets = RDK.ItemList(ITEM_TYPE_TARGET)

    for target_item in all_targets:
        name = target_item.Name()
        for pattern, distances in offset_rules:
            if not pattern.match(name):
                continue

            before_name = f"offset_before_for_{name}"
            _, was_created = create_offset_target(
                RDK, robot, target_item, before_name, distances["before"], before_folder
            )
            created += was_created
            skipped += (not was_created)

            after_name = f"offset_after_for_{name}"
            _, was_created = create_offset_target(
                RDK, robot, target_item, after_name, distances["after"], after_folder
            )
            created += was_created
            skipped += (not was_created)

            break

    total = created + skipped
    print(f"[offsets] {total} offset target(s): {created} created, {skipped} already exist")


# ── PHASE 1: IK SOLVING ─────────────────────────────────────────────────────

_OPT_AXES_6DOF = {
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    # Absolute constraints to lock j2/j3 near default (forward) configuration
    "AbsJnt_2": 0, "AbsOn_2": 1, "AbsW_2": 100,
    "AbsJnt_3": 0, "AbsOn_3": 1, "AbsW_3": 100,
    # Relative weights for all joints
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1,
    "RelOn_4": 1, "RelOn_5": 1, "RelOn_6": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50,
    "RelW_4": 50, "RelW_5": 50, "RelW_6": 50,
}

HOME_SEEDS = {
    "home":      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "home_p170": [170.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "home_n170": [-170.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "home_p180": [180.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "home_n180": [-180.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}
HOME_SEED_6DOF = HOME_SEEDS["home"]


def try_ik(robot, pose, seed=None):
    if seed is None:
        seed = HOME_SEED_6DOF
    robot.setParam("OptimAxes", _OPT_AXES_6DOF)
    robot.setJoints(seed)
    try:
        robot.MoveJ(pose)
        raw = robot.Joints()
        try:
            joints = raw.list()
        except AttributeError:
            joints = list(raw)
        robot.setJoints(seed)
        if len(joints) < 6:
            return None
        if all(abs(j) < 1e-6 for j in joints):
            return None
        return joints
    except Exception:
        robot.setJoints(seed)
        return None


def solve_with_z_sweep(robot, target_pose, step_deg, seed=None):
    joints = try_ik(robot, target_pose, seed=seed)
    if joints is not None:
        return target_pose, joints, 0.0

    n_steps = int(360 / step_deg)
    for i in range(1, n_steps):
        angle_deg = step_deg * i
        angle_rad = angle_deg * math.pi / 180.0
        rotated_pose = target_pose * rotz(angle_rad)
        joints = try_ik(robot, rotated_pose, seed=seed)
        if joints is not None:
            return rotated_pose, joints, angle_deg

    return None, None, None


def solve_and_create_targets(RDK, robot, config, extracted_folder,
                             before_folder, after_folder):
    step_deg = config["z_rotation_step_deg"]
    offsets = config["offsets_mm"]
    ee_config = config["end_effectors"]

    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    if not world_frame.Valid():
        world_frame = RDK.Item("", ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)

    solve_rules = [
        (GRAB_PATTERN, ee_config["grab"], offsets["grab"]),
        (STRING_GRAB_PATTERN, ee_config["string_grab"], offsets["string_grab"]),
    ]

    all_targets = RDK.ItemList(ITEM_TYPE_TARGET)
    failures = []
    solved = 0
    cached = 0

    for target_item in all_targets:
        name = target_item.Name()

        for pattern, tool_name, offset_distances in solve_rules:
            if not pattern.match(name):
                continue

            if target_exists_in_folder(extracted_folder, name):
                cached += 1
                break

            tool = find_tool(RDK, tool_name)
            robot.setTool(tool)

            target_pose = target_item.PoseAbs()

            solved_pose, joints, angle_deg = solve_with_z_sweep(
                robot, target_pose, step_deg
            )

            if solved_pose is None:
                print(f"  [FAIL] {name} — no solution found (tool={tool_name})")
                failures.append((name, tool_name))
                break

            if angle_deg == 0.0:
                print(f"  [OK]   {name} — solved at original pose")
            else:
                print(f"  [OK]   {name} — solved at Z rotation {angle_deg:.1f} deg")

            solved_target = RDK.AddTarget(name, extracted_folder, robot)
            solved_target.setPose(solved_pose)
            solved_target.setJoints(joints)

            before_pose = solved_pose * transl(0, 0, -offset_distances["before"])
            before_solved, before_joints, _ = solve_with_z_sweep(robot, before_pose, step_deg)
            before_target = RDK.AddTarget(
                f"offset_before_for_{name}", before_folder, robot
            )
            if before_solved is not None:
                before_target.setPose(before_solved)
                before_target.setJoints(before_joints)
            else:
                before_target.setPose(before_pose)
                print(f"    [WARN] No IK for offset_before_for_{name}")

            after_pose = solved_pose * transl(0, 0, -offset_distances["after"])
            after_solved, after_joints, _ = solve_with_z_sweep(robot, after_pose, step_deg)
            after_target = RDK.AddTarget(
                f"offset_after_for_{name}", after_folder, robot
            )
            if after_solved is not None:
                after_target.setPose(after_solved)
                after_target.setJoints(after_joints)
            else:
                after_target.setPose(after_pose)
                print(f"    [WARN] No IK for offset_after_for_{name}")

            solved += 1
            break

    print(f"[solve] {solved} solved, {cached} cached, {len(failures)} failed")
    return failures


def write_failure_report(failures):
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        f.write("IK Failure Report\n")
        f.write("=" * 60 + "\n\n")
        if not failures:
            f.write("No failures — all targets solved successfully.\n")
        else:
            f.write(f"{len(failures)} target(s) could not be solved:\n\n")
            for name, tool_name in failures:
                f.write(f"  Target: {name}\n")
                f.write(f"  Tool:   {tool_name}\n")
                f.write(f"  Note:   No IK solution found at any Z-axis rotation\n\n")
    print(f"[REPORT] Written to {REPORT_PATH}")


# ── PHASE 2: targets_to_use ─────────────────────────────────────────────────

def build_targets_to_use(RDK, robot, cone_names, failures,
                         rotated_extracted, rotated_before, rotated_after):
    failed_names = {name for name, _ in failures}

    solved_children = set()
    for child in rotated_extracted.Childs():
        solved_children.add(child.Name())

    ttu_root = get_or_create_folder(RDK, "targets_to_use")
    ttu_extracted = get_or_create_folder(RDK, "extracted", parent=ttu_root)
    ttu_offsets = get_or_create_folder(RDK, "auto_generated_offsets", parent=ttu_root)
    ttu_before = get_or_create_folder(RDK, "before", parent=ttu_offsets)
    ttu_after = get_or_create_folder(RDK, "after", parent=ttu_offsets)
    ttu_root.setVisible(True)
    ttu_extracted.setVisible(True)
    ttu_offsets.setVisible(True)
    ttu_before.setVisible(True)
    ttu_after.setVisible(True)

    existing_in_extracted = {c.Name() for c in ttu_extracted.Childs() if c.Type() == ITEM_TYPE_TARGET}
    existing_in_before = {c.Name() for c in ttu_before.Childs() if c.Type() == ITEM_TYPE_TARGET}
    existing_in_after = {c.Name() for c in ttu_after.Childs() if c.Type() == ITEM_TYPE_TARGET}

    targets_to_use = {}
    created = 0
    cached = 0

    for cone in cone_names:
        grab_name = f"{cone}_grab"
        string_grab_name = f"{cone}_string_grab"

        if grab_name in failed_names or string_grab_name in failed_names:
            continue
        if grab_name not in solved_children or string_grab_name not in solved_children:
            continue

        targets_to_use[cone] = {
            "grab": {
                "target": grab_name,
                "before": f"offset_before_for_{grab_name}",
                "after": f"offset_after_for_{grab_name}",
            },
            "string_grab": {
                "target": string_grab_name,
                "before": f"offset_before_for_{string_grab_name}",
                "after": f"offset_after_for_{string_grab_name}",
            },
        }

        for target_name in (grab_name, string_grab_name):
            if target_name in existing_in_extracted:
                cached += 1
                continue
            src = find_target_in_folder(rotated_extracted, target_name)
            if src is None:
                continue
            tgt = RDK.AddTarget(target_name, ttu_extracted, robot)
            tgt.setPose(src.Pose())
            tgt.setJoints(src.Joints())
            created += 1

        for target_name in (grab_name, string_grab_name):
            before_name = f"offset_before_for_{target_name}"
            after_name = f"offset_after_for_{target_name}"

            if before_name not in existing_in_before:
                src = find_target_in_folder(rotated_before, before_name)
                if src is not None:
                    tgt = RDK.AddTarget(before_name, ttu_before, robot)
                    tgt.setPose(src.Pose())
                    tgt.setJoints(src.Joints())
                    created += 1
            else:
                cached += 1

            if after_name not in existing_in_after:
                src = find_target_in_folder(rotated_after, after_name)
                if src is not None:
                    tgt = RDK.AddTarget(after_name, ttu_after, robot)
                    tgt.setPose(src.Pose())
                    tgt.setJoints(src.Joints())
                    created += 1
            else:
                cached += 1

    print(f"[targets_to_use] {len(targets_to_use)} cone(s): {created} targets created, {cached} cached")
    return targets_to_use


def write_targets_to_use(targets_to_use):
    with open(TARGETS_TO_USE_PATH, "w", encoding="utf-8") as f:
        json.dump(targets_to_use, f, indent=2)
    print(f"[WRITE] {TARGETS_TO_USE_PATH} — {len(targets_to_use)} cone(s)")


# ── PHASE 3: create attach scripts ───────────────────────────────────────────

def to_robodk_path(path):
    abs_path = os.path.abspath(path)
    try:
        if abs_path.startswith("/mnt/"):
            parts = abs_path.split("/")
            drive = parts[2].upper()
            rest = "/".join(parts[3:])
            return f"{drive}:/{rest}"
    except (IndexError, AttributeError):
        pass
    return abs_path


REPLACE_CONE_SCRIPT = os.path.join(SCRIPT_DIR, "replace_cone.py")
ATTACH_SCRIPTS_DIR = os.path.join(SCRIPT_DIR, "attach_scripts")


def install_replace_cones_script(RDK, programs_folder):
    existing = RDK.Item("replace_cone", ITEM_TYPE_PROGRAM_PYTHON)
    if existing.Valid():
        return
    script_path = to_robodk_path(REPLACE_CONE_SCRIPT)
    item = RDK.AddFile(script_path, programs_folder)
    if item.Valid():
        print(f"[CREATE] Added replace_cone script to programs folder")


def _write_attach_script(cone_name):
    os.makedirs(ATTACH_SCRIPTS_DIR, exist_ok=True)
    script_path = os.path.join(ATTACH_SCRIPTS_DIR, f"attach_{cone_name}.py")
    if not os.path.exists(script_path):
        with open(script_path, "w", encoding="utf-8") as f:
            f.write(f'''from robodk.robolink import Robolink, ITEM_TYPE_TOOL, ITEM_TYPE_OBJECT
RDK = Robolink()
tool = RDK.Item("pickup", ITEM_TYPE_TOOL)
cone = RDK.Item("{cone_name}", ITEM_TYPE_OBJECT)
if tool.Valid() and cone.Valid():
    cone.setParentStatic(tool)
    print("Attached: {cone_name}")
else:
    print("Failed to attach {cone_name}")
''')
    return script_path


def create_attach_scripts(RDK, cone_names):
    """Create per-cone attach scripts and add them to the station."""
    programs_folder = RDK.Item("programs", ITEM_TYPE_FOLDER)
    assert programs_folder.Valid(), "programs folder not found"
    attach_folder = get_or_create_folder(RDK, "attach_scripts", parent=programs_folder)

    created = 0
    cached = 0

    for cone_name in cone_names:
        attach_name = f"attach_{cone_name}"
        existing = RDK.Item(attach_name, ITEM_TYPE_PROGRAM_PYTHON)
        if existing.Valid():
            cached += 1
            continue

        attach_script = _write_attach_script(cone_name)
        attach_path = to_robodk_path(attach_script)
        attach_prog = RDK.AddFile(attach_path)
        if attach_prog.Valid():
            attach_prog.setParent(attach_folder)
            created += 1

    # Also install replace_cone.py
    install_replace_cones_script(RDK, programs_folder)

    total = created + cached
    print(f"[attach_scripts] {total} script(s): {created} created, {cached} cached")


# ── PHASE 4: create final pullaway targets ───────────────────────────────────

# Regex to extract group base: "Base_Right_2" → "Base_Right", "alt_Base_Left_1" → "alt_Base_Left"
GROUP_PATTERN = re.compile(r"^((alt_)?Base_(Right|Left))_\d+$")


def create_final_pullaway_targets(RDK, robot, cone_names, config, after_folder):
    """Create a final pullaway target for each cone group.

    Takes the _0 cone's after-grab offset and offsets it along -X by
    final_pullaway_mm. One target per group (Base_Right, Base_Left, etc.).
    Also solves IK with Z-rotation sweep.
    """
    pullaway_mm = config.get("final_pullaway_mm", 60)
    step_deg = config["z_rotation_step_deg"]

    # Discover groups from cone names
    groups = {}
    for cone in cone_names:
        m = GROUP_PATTERN.match(cone)
        if m:
            group = m.group(1)  # e.g. "Base_Right"
            groups[group] = f"{group}_0"  # the _0 cone for this group

    created = 0
    cached = 0

    for group, zero_cone in sorted(groups.items()):
        pullaway_name = f"{group}_final_pullaway"

        # Check if already exists
        existing = find_target_in_folder(after_folder, pullaway_name)
        if existing is not None:
            cached += 1
            continue

        # Find the _0 cone's after-grab offset
        zero_after_name = f"offset_after_for_{zero_cone}_grab"
        zero_after = find_target_in_folder(after_folder, zero_after_name)
        if zero_after is None:
            print(f"  [WARN] {zero_after_name} not found — skipping {pullaway_name}")
            continue

        # Offset along -X (red axis) from the _0 after pose
        zero_pose = zero_after.Pose()
        pullaway_pose = zero_pose * transl(-pullaway_mm, 0, 0)

        # Solve IK
        tool = find_tool(RDK, config["end_effectors"]["grab"])
        robot.setTool(tool)
        world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
        if world_frame.Valid():
            robot.setPoseFrame(world_frame)

        solved_pose, joints, angle = solve_with_z_sweep(robot, pullaway_pose, step_deg)

        assert solved_pose is not None, \
            f"[ERROR] No IK solution found for {pullaway_name} at any Z rotation — " \
            f"check that the pullaway distance ({pullaway_mm}mm) is reachable"

        tgt = RDK.AddTarget(pullaway_name, after_folder, robot)
        tgt.setPose(solved_pose)
        tgt.setJoints(joints)
        if angle == 0.0:
            print(f"  [OK]   {pullaway_name} — solved at original pose")
        else:
            print(f"  [OK]   {pullaway_name} — solved at Z rotation {angle:.1f} deg")

        created += 1

    total = created + cached
    print(f"[final_pullaway] {total} target(s): {created} created, {cached} cached")


# ── PHASE 7: create obstacle avoidance targets ──────────────────────────────

def create_obstacle_avoidance_targets(RDK, robot, config, targets_to_use,
                                      ttu_before, ttu_after, avoidance_folder):
    """Create approach/retract pullaway and pull_in targets for each cone.

    All targets go in created_for_obstacle_avoidance folder.
    IK seeded from the corresponding offset solutions.
    """
    pullaway_mm = config.get("final_pullaway_mm", 600)

    created = 0
    cached = 0

    for cone_name, entries in targets_to_use.items():
        sg = entries["string_grab"]
        gr = entries["grab"]

        sg_before = find_target_in_folder(ttu_before, sg["before"])
        gr_after = find_target_in_folder(ttu_after, gr["after"])

        m = GROUP_PATTERN.match(cone_name)
        if not m:
            continue
        group = m.group(1)

        # Get seeds from offset solutions
        try:
            sg_seed = sg_before.Joints().list() if sg_before else HOME_SEED_6DOF
        except AttributeError:
            sg_seed = list(sg_before.Joints()) if sg_before else HOME_SEED_6DOF
        try:
            gr_seed = gr_after.Joints().list() if gr_after else HOME_SEED_6DOF
        except AttributeError:
            gr_seed = list(gr_after.Joints()) if gr_after else HOME_SEED_6DOF

        # Find the group's final pullaway target
        pullaway_name = f"{group}_final_pullaway"
        pullaway_tgt = find_target_in_folder(ttu_after, pullaway_name)

        # 1. approach_pullaway — same pose as group pullaway, seeded from sg_before
        if pullaway_tgt is not None:
            name = f"{cone_name}_approach_pullaway"
            existing = find_target_in_folder(avoidance_folder, name)
            if existing:
                cached += 1
            else:
                pose = pullaway_tgt.Pose()
                joints = try_ik(robot, pose, seed=sg_seed)
                if joints is None:
                    for seed in HOME_SEEDS.values():
                        joints = try_ik(robot, pose, seed=seed)
                        if joints is not None:
                            break
                tgt = RDK.AddTarget(name, avoidance_folder, robot)
                tgt.setPose(pose)
                if joints is not None:
                    tgt.setJoints(joints)
                else:
                    print(f"    [WARN] No IK for {name}")
                    tgt.setJoints(pullaway_tgt.Joints())
                created += 1

        # 2. approach_pull_in — sg_before offset by the same vector as final_pullaway
        #    (i.e. the vector from _0 after-grab to the pullaway position)
        if sg_before is not None and pullaway_tgt is not None:
            name = f"{cone_name}_approach_pull_in"
            existing = find_target_in_folder(avoidance_folder, name)
            if existing:
                cached += 1
            else:
                # Get the pullaway offset vector in world space
                zero_after_name = f"offset_after_for_{group}_0_grab"
                zero_after = find_target_in_folder(ttu_after, zero_after_name)
                if zero_after is not None:
                    pa_pos = np.array(Pose_2_TxyzRxyz(pullaway_tgt.PoseAbs())[:3])
                    za_pos = np.array(Pose_2_TxyzRxyz(zero_after.PoseAbs())[:3])
                    offset_vec = pa_pos - za_pos  # world-space offset vector

                    # Apply same vector to sg_before's world position, keep sg_before orientation
                    sg_abs = sg_before.PoseAbs()
                    sg_pos = np.array(Pose_2_TxyzRxyz(sg_abs)[:3])
                    new_pos = sg_pos + offset_vec

                    # Build pose: sg_before rotation + new position
                    sg_axes = np.array(sg_abs.rows)
                    pull_in_pose = Mat([
                        [sg_axes[0][0], sg_axes[0][1], sg_axes[0][2], new_pos[0]],
                        [sg_axes[1][0], sg_axes[1][1], sg_axes[1][2], new_pos[1]],
                        [sg_axes[2][0], sg_axes[2][1], sg_axes[2][2], new_pos[2]],
                        [0, 0, 0, 1],
                    ])
                else:
                    # Fallback: offset along sg_before's -X
                    pull_in_pose = sg_before.PoseAbs() * transl(-pullaway_mm, 0, 0)

                joints = try_ik(robot, pull_in_pose, seed=sg_seed)
                if joints is None:
                    for seed in HOME_SEEDS.values():
                        joints = try_ik(robot, pull_in_pose, seed=seed)
                        if joints is not None:
                            break
                tgt = RDK.AddTarget(name, avoidance_folder, robot)
                tgt.setPose(pull_in_pose)
                if joints is not None:
                    tgt.setJoints(joints)
                else:
                    print(f"    [WARN] No IK for {name}")
                    tgt.setJoints(sg_before.Joints())
                created += 1

        # 3. retract_pullaway — same pose as group pullaway, seeded from gr_after
        if pullaway_tgt is not None:
            name = f"{cone_name}_retract_pullaway"
            existing = find_target_in_folder(avoidance_folder, name)
            if existing:
                cached += 1
            else:
                pose = pullaway_tgt.Pose()
                joints = try_ik(robot, pose, seed=gr_seed)
                if joints is None:
                    for seed in HOME_SEEDS.values():
                        joints = try_ik(robot, pose, seed=seed)
                        if joints is not None:
                            break
                tgt = RDK.AddTarget(name, avoidance_folder, robot)
                tgt.setPose(pose)
                if joints is not None:
                    tgt.setJoints(joints)
                else:
                    print(f"    [WARN] No IK for {name}")
                    tgt.setJoints(pullaway_tgt.Joints())
                created += 1

        # 4. bl_retract_pullaway — for alt cones, route through Base_Left pullaway
        if group.startswith("alt_"):
            bl_pullaway = find_target_in_folder(ttu_after, "Base_Left_final_pullaway")
            if bl_pullaway is not None:
                name = f"{cone_name}_bl_retract_pullaway"
                existing = find_target_in_folder(avoidance_folder, name)
                if existing:
                    cached += 1
                else:
                    pose = bl_pullaway.Pose()
                    joints = try_ik(robot, pose, seed=gr_seed)
                    if joints is None:
                        for seed in HOME_SEEDS.values():
                            joints = try_ik(robot, pose, seed=seed)
                            if joints is not None:
                                break
                    tgt = RDK.AddTarget(name, avoidance_folder, robot)
                    tgt.setPose(pose)
                    if joints is not None:
                        tgt.setJoints(joints)
                    else:
                        print(f"    [WARN] No IK for {name}")
                        tgt.setJoints(bl_pullaway.Joints())
                    created += 1

        print(f"  [OK] {cone_name}")

    total = created + cached
    print(f"[avoidance] {total} target(s): {created} created, {cached} cached")


# ── PHASE 8: populate programs ───────────────────────────────────────────────

def create_home_targets(RDK, robot, extracted_folder):
    """Create all home joint targets. Returns dict of name → target item."""
    homes = {}
    for name, joints in HOME_SEEDS.items():
        existing = find_target_in_folder(extracted_folder, name)
        if existing is not None:
            homes[name] = existing
            continue
        tgt = RDK.AddTarget(name, extracted_folder, robot)
        tgt.setJoints(joints)
        tgt.setAsJointTarget()
        homes[name] = tgt
    print(f"[homes] {len(homes)} home target(s)")
    return homes


def _angular_dist(a_deg, b_deg):
    """Shortest angular distance in degrees, accounting for wrapping."""
    diff = (a_deg - b_deg) % 360
    if diff > 180:
        diff = 360 - diff
    return diff


def pick_closest_home(homes, reference_target):
    """Pick the home target with shortest joint-space distance to reference.

    Uses wrapping-aware angular distance on all joints.
    """
    try:
        ref_joints = reference_target.Joints().list()
    except AttributeError:
        ref_joints = list(reference_target.Joints())

    best_target = None
    best_name = None
    best_dist = float("inf")

    for name, target in homes.items():
        seed = HOME_SEEDS[name]
        dist = sum(_angular_dist(a, b) ** 2 for a, b in zip(seed, ref_joints))
        if dist < best_dist:
            best_dist = dist
            best_target = target
            best_name = name

    return best_target, best_name


def populate_programs(RDK, robot, targets_to_use, ee_config,
                      extracted_folder, before_folder, after_folder,
                      avoidance_folder):
    """Populate programs — references only, no target creation."""
    populated = 0
    skipped = 0

    knotting_tool = find_tool(RDK, ee_config["string_grab"])
    pickup_tool = find_tool(RDK, ee_config["grab"])
    homes = create_home_targets(RDK, robot, extracted_folder)

    # Save original cone poses
    cone_poses = {}
    if os.path.exists(CONE_POSES_PATH):
        with open(CONE_POSES_PATH, "r", encoding="utf-8") as f:
            cone_poses = json.load(f)

    all_objects = RDK.ItemList(ITEM_TYPE_OBJECT)
    for obj in all_objects:
        name = obj.Name()
        if CONE_PATTERN.match(name) and name not in cone_poses:
            cone_poses[name] = {
                "pose": Pose_2_TxyzRxyz(obj.Pose()),
                "parent": obj.Parent().Name() if obj.Parent().Valid() else "",
            }

    if cone_poses:
        with open(CONE_POSES_PATH, "w", encoding="utf-8") as f:
            json.dump(cone_poses, f, indent=2)
        print(f"[SAVE] Cone original poses saved ({len(cone_poses)} cones)")

    for cone_name, entries in targets_to_use.items():
        prog = RDK.Item(cone_name, ITEM_TYPE_PROGRAM)
        if not prog.Valid():
            print(f"  [SKIP] Program '{cone_name}' not found")
            skipped += 1
            continue

        n_ins = prog.InstructionCount()
        if n_ins > 0:
            skipped += 1
            continue

        sg = entries["string_grab"]
        gr = entries["grab"]

        sg_before = find_target_in_folder(before_folder, sg["before"])
        sg_target = find_target_in_folder(extracted_folder, sg["target"])
        sg_after = find_target_in_folder(after_folder, sg["after"])

        gr_before = find_target_in_folder(before_folder, gr["before"])
        gr_target = find_target_in_folder(extracted_folder, gr["target"])
        gr_after = find_target_in_folder(after_folder, gr["after"])

        missing = []
        for label, item in [
            (sg["before"], sg_before), (sg["target"], sg_target), (sg["after"], sg_after),
            (gr["before"], gr_before), (gr["target"], gr_target), (gr["after"], gr_after),
        ]:
            if item is None:
                missing.append(label)
        if missing:
            print(f"  [SKIP] {cone_name} — missing targets: {missing}")
            skipped += 1
            continue

        # Look up avoidance targets
        approach_pullaway = find_target_in_folder(avoidance_folder, f"{cone_name}_approach_pullaway")
        approach_pull_in = find_target_in_folder(avoidance_folder, f"{cone_name}_approach_pull_in")
        retract_pullaway = find_target_in_folder(avoidance_folder, f"{cone_name}_retract_pullaway")
        bl_retract = find_target_in_folder(avoidance_folder, f"{cone_name}_bl_retract_pullaway")

        # Set knotting tool first
        prog.setPoseTool(knotting_tool)

        # Start from closest home
        start_home, start_name = pick_closest_home(homes, sg_before)
        prog.MoveJ(start_home)

        # Approach sequence
        if approach_pullaway is not None:
            prog.MoveJ(approach_pullaway)
        if approach_pull_in is not None:
            prog.MoveJ(approach_pull_in)

        # String grab sequence
        prog.MoveJ(sg_before)
        prog.MoveL(sg_target)
        prog.MoveL(sg_after)

        # Pickup grab sequence
        prog.setPoseTool(pickup_tool)
        prog.MoveL(gr_before)
        prog.MoveL(gr_target)

        # Attach cone
        prog.RunInstruction(f"attach_{cone_name}", INSTRUCTION_CALL_PROGRAM)

        # Retract sequence
        prog.MoveL(gr_after)
        if retract_pullaway is not None:
            prog.MoveL(retract_pullaway)
        if bl_retract is not None:
            prog.MoveL(bl_retract)

        # Return to closest home
        end_home, end_name = pick_closest_home(homes, gr_after)
        prog.MoveJ(end_home)

        populated += 1
        print(f"  [OK]   {cone_name} (start={start_name}, end={end_name})")

    total = populated + skipped
    print(f"[programs] {total} program(s): {populated} populated, {skipped} skipped")


def create_main_program(RDK, robot, targets_to_use, extracted_folder):
    """Create a 'main' program that runs each cone program then replace_cone."""
    existing = RDK.Item("main", ITEM_TYPE_PROGRAM)
    if existing.Valid():
        if existing.InstructionCount() > 0:
            print("[CACHE] 'main' program already exists")
            return
        main_prog = existing
    else:
        main_prog = RDK.AddProgram("main", robot)
        programs_folder = RDK.Item("programs", ITEM_TYPE_FOLDER)
        if programs_folder.Valid():
            main_prog.setParent(programs_folder)

    for cone_name in sorted(targets_to_use.keys()):
        main_prog.RunInstruction(cone_name, INSTRUCTION_CALL_PROGRAM)
        main_prog.RunInstruction("replace_cone", INSTRUCTION_CALL_PROGRAM)

    print(f"[CREATE] 'main' program with {len(targets_to_use)} cone sequences")


# ── MAIN ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Build movement sequences for base cone pickup (run organize_station.py first)"
    )
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help=f"Config JSON (default: {os.path.basename(DEFAULT_CONFIG)})")
    ap.add_argument("--skip", nargs="*", default=[],
                    help="Phases to skip, e.g. --skip 1 2 3 4 5 6 7 8")
    ap.add_argument("--rapid-test-mode", action="store_true",
                    help="Only process one cone for quick testing")
    args = ap.parse_args()

    skip = {s.upper() for s in args.skip}
    if skip:
        print(f"[SKIP] Skipping phases: {', '.join(sorted(skip))}")

    config = load_config(args.config)
    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)

    # Verify organize_station.py has been run
    offset_frame = RDK.Item(OFFSET_FRAME_NAME, ITEM_TYPE_FRAME)
    assert offset_frame.Valid(), \
        f"'{OFFSET_FRAME_NAME}' not found — run organize_station.py first"

    cone_names = discover_cone_names(RDK)

    if args.rapid_test_mode and cone_names:
        cone_names = [cone_names[0]]
        print(f"[RAPID TEST] Only processing: {cone_names[0]}")
    else:
        print(f"[DISCOVER] {len(cone_names)} base cone(s) with grab + string_grab pairs")

    # ── Phase 1: create empty programs ────────────────────────────────
    if "1" not in skip:
        print("\n── Phase 1: Create programs ──")
        programs_folder = get_or_create_folder(RDK, "programs")
        programs_folder.setVisible(True)
        created = 0
        skipped_p = 0
        for cone_name in cone_names:
            existing = RDK.Item(cone_name, ITEM_TYPE_PROGRAM)
            if existing.Valid():
                skipped_p += 1
                continue
            prog = RDK.AddProgram(cone_name, robot)
            prog.setParent(programs_folder)
            created += 1
        print(f"[programs] {created + skipped_p} program(s): {created} created, {skipped_p} already exist")
    else:
        print("\n── Phase 1: SKIPPED ──")

    # ── Phase 2: create offset targets ────────────────────────────────
    if "2" not in skip:
        print("\n── Phase 2: Create offset targets (before/after) ──")
        offsets_parent = get_or_create_folder(RDK, "auto_generated_offsets", parent=offset_frame)
        before_folder = get_or_create_folder(RDK, "before", parent=offsets_parent)
        after_folder = get_or_create_folder(RDK, "after", parent=offsets_parent)
        offsets_parent.setVisible(True)
        before_folder.setVisible(True)
        after_folder.setVisible(True)
        create_offset_targets(RDK, robot, config["offsets_mm"], before_folder, after_folder)
    else:
        print("\n── Phase 2: SKIPPED ──")

    # ── Phase 3: solve IK with Z-rotation sweep ──────────────────────
    rotated_root = get_or_create_folder(RDK, "targets_rotated_for_solution")
    rotated_extracted = get_or_create_folder(RDK, "extracted", parent=rotated_root)
    rotated_offsets = get_or_create_folder(
        RDK, "auto_generated_offsets", parent=rotated_root
    )
    rotated_before = get_or_create_folder(RDK, "before", parent=rotated_offsets)
    rotated_after = get_or_create_folder(RDK, "after", parent=rotated_offsets)

    failures = []

    if "3" not in skip:
        print("\n── Phase 3: Solve IK for all targets ──")
        step_deg = config["z_rotation_step_deg"]
        n_steps = int(360 / step_deg)
        print(f"[INFO] Z-rotation sweep: {step_deg} deg steps = {n_steps} attempts per target")

        rotated_root.setVisible(True)
        rotated_extracted.setVisible(True)
        rotated_offsets.setVisible(True)
        rotated_before.setVisible(True)
        rotated_after.setVisible(True)

        failures = solve_and_create_targets(
            RDK, robot, config, rotated_extracted, rotated_before, rotated_after
        )

        if failures:
            write_failure_report(failures)
            print(f"\n[WARN] {len(failures)} target(s) failed — see {REPORT_PATH}")
        else:
            if os.path.exists(REPORT_PATH):
                os.remove(REPORT_PATH)
            print("\n[OK] All targets solved successfully.")
    else:
        print("\n── Phase 3: SKIPPED ──")

    # ── Phase 4: build targets_to_use folder + JSON ───────────────────
    if "4" not in skip:
        print("\n── Phase 4: Build targets_to_use folder ──")
        targets_to_use = build_targets_to_use(
            RDK, robot, cone_names, failures,
            rotated_extracted, rotated_before, rotated_after
        )
        write_targets_to_use(targets_to_use)
    else:
        print("\n── Phase 4: SKIPPED ──")

    # ── Phase 5: create attach scripts ──────────────────────────────
    if "5" not in skip:
        print("\n── Phase 5: Create attach scripts ──")
        create_attach_scripts(RDK, cone_names)
    else:
        print("\n── Phase 5: SKIPPED ──")

    # ── Phase 6: create final pullaway targets ────────────────────────
    ttu_root = RDK.Item("targets_to_use", ITEM_TYPE_FOLDER)
    assert ttu_root.Valid(), \
        "targets_to_use folder not found — run Phase 4 first"
    ttu_extracted = get_or_create_folder(RDK, "extracted", parent=ttu_root)
    ttu_offsets = get_or_create_folder(RDK, "auto_generated_offsets", parent=ttu_root)
    ttu_before = get_or_create_folder(RDK, "before", parent=ttu_offsets)
    ttu_after = get_or_create_folder(RDK, "after", parent=ttu_offsets)

    if "6" not in skip:
        print("\n── Phase 6: Create final pullaway targets ──")
        create_final_pullaway_targets(RDK, robot, cone_names, config, ttu_after)
    else:
        print("\n── Phase 6: SKIPPED ──")

    # ── Build targets_to_use dict for phases 7-8 ─────────────────────
    extracted_targets = {c.Name(): c for c in ttu_extracted.Childs()
                        if c.Type() == ITEM_TYPE_TARGET}
    targets_to_use_dict = {}
    for name in extracted_targets:
        if GRAB_PATTERN.match(name):
            cone = name.rsplit("_grab", 1)[0]
            kind = "grab"
        elif STRING_GRAB_PATTERN.match(name):
            cone = name.rsplit("_string_grab", 1)[0]
            kind = "string_grab"
        else:
            continue
        if cone not in targets_to_use_dict:
            targets_to_use_dict[cone] = {}
        targets_to_use_dict[cone][kind] = {
            "target": name,
            "before": f"offset_before_for_{name}",
            "after": f"offset_after_for_{name}",
        }
    complete = {c: v for c, v in targets_to_use_dict.items()
                if "grab" in v and "string_grab" in v}

    # ── Phase 7: create obstacle avoidance targets ────────────────────
    avoidance_folder = get_or_create_folder(RDK, "created_for_obstacle_avoidance")
    avoidance_folder.setVisible(True)

    if "7" not in skip:
        print("\n── Phase 7: Create obstacle avoidance targets ──")
        print(f"[INFO] {len(complete)} cone(s) with both grab + string_grab")
        create_obstacle_avoidance_targets(
            RDK, robot, config, complete,
            ttu_before, ttu_after, avoidance_folder
        )
    else:
        print("\n── Phase 7: SKIPPED ──")

    # ── Phase 8: populate programs ────────────────────────────────────
    if "8" not in skip:
        print("\n── Phase 8: Populate programs ──")
        assert len(extracted_targets) > 0, \
            "targets_to_use/extracted is empty — run Phase 4 first"
        print(f"[INFO] {len(complete)} cone(s)")

        ee_config = config["end_effectors"]
        populate_programs(
            RDK, robot, complete, ee_config,
            ttu_extracted, ttu_before, ttu_after,
            avoidance_folder
        )

        create_main_program(RDK, robot, complete, ttu_extracted)
    else:
        print("\n── Phase 8: SKIPPED ──")

    print("\n[DONE] Station setup complete.")


if __name__ == "__main__":
    main()
