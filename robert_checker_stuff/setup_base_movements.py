"""
Set up the extracted station for base cone movement sequence testing.

Phase 1 — organize items into visible GUI folders:
  - extracted_targets  — all Target items (grab + string_grab only)
  - cones              — Base cone objects (Base_Right_*, Base_Left_*, alt_Base_*)
  - bins               — Bin objects (bin_*)

Phase 2 — create empty programs for each grab/string_grab pair:
  - One program per base cone, named after the cone (e.g. "Base_Right_0")
  - All programs go under a "programs" folder

Phase 3 — create offset targets for each grab and string_grab:
  - offset_before_for_<name> — offset along -Z before the target (approach)
  - offset_after_for_<name>  — offset along -Z after the target (retract)
  - Distances from config (grab: 50/300mm, string_grab: 50/50mm)
  - Stored under auto_generated_offsets/before and auto_generated_offsets/after

Phase 4 — solve IK for all targets with Z-axis rotation sweep:
  - Try original pose first; if it fails, sweep Z rotations per config step
  - grab targets solved with pickup tool, string_grab with knotting tool
  - Solved targets stored in targets_rotated_for_solution/extracted
  - Solved offsets in targets_rotated_for_solution/auto_generated_offsets/before|after
  - Failures written to ik_failure_report.txt

Phase 5 — build targets_to_use folder + JSON:
  - Merges solved targets from Phase 4 into a single folder + lookup file

Phase 6 — populate programs with movement instructions:
  - Each cone's program gets MoveL instructions:
    home → knotting(before→string_grab→after) → pickup(before→grab→after)

Reads settings from setup_base_movements_config.json (same directory).

Caching: folders, item placements, programs, and solved targets are reused
if they already exist.

Usage:
    python robert_checker_stuff/setup_base_movements.py
    python robert_checker_stuff/setup_base_movements.py --robodk-ip 172.23.208.1
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
from robodk.robomath import transl, rotz, Pose_2_TxyzRxyz

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(SCRIPT_DIR, "setup_base_movements_config.json")
REPORT_PATH = os.path.join(SCRIPT_DIR, "ik_failure_report.txt")
TARGETS_TO_USE_PATH = os.path.join(SCRIPT_DIR, "targets_to_use.json")
CONE_POSES_PATH = os.path.join(SCRIPT_DIR, "cone_original_poses.json")

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]

CONE_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+$")
BIN_PATTERN = re.compile(r"^bin_\d+$")

GRAB_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+_grab$")
STRING_GRAB_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+_string_grab$")

# Matches the original extracted targets only (not offsets or solved copies)
EXTRACTED_TARGET_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+_(grab|string_grab)$")

FOLDER_DEFS = {
    "extracted_targets": {
        "item_type": ITEM_TYPE_TARGET,
        "filter": EXTRACTED_TARGET_PATTERN,
    },
    "cones": {
        "item_type": ITEM_TYPE_OBJECT,
        "filter": CONE_PATTERN,
    },
    "bins": {
        "item_type": ITEM_TYPE_OBJECT,
        "filter": BIN_PATTERN,
    },
}


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
    print(f"[CONFIG] offsets_mm.grab: before={offsets['grab']['before']}mm, after={offsets['grab']['after']}mm")
    print(f"[CONFIG] offsets_mm.string_grab: before={offsets['string_grab']['before']}mm, after={offsets['string_grab']['after']}mm")
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
    folder names at different levels (e.g. 'before' at root vs inside another folder).
    """
    if parent is not None:
        # Check children of parent for this name
        for child in parent.Childs():
            if child.Name() == name and child.Type() == ITEM_TYPE_FOLDER:
                return child

        # Snapshot root folders before creation so we can find the new one
        existing_ids = {f.item for f in RDK.ItemList(ITEM_TYPE_FOLDER)}

        RDK.Command("AddFolder", name)

        # Find the newly created folder (not in the snapshot)
        folder = None
        for f in RDK.ItemList(ITEM_TYPE_FOLDER):
            if f.item not in existing_ids and f.Name() == name:
                folder = f
                break

        assert folder is not None, f"Failed to create folder '{name}'"
        folder.setParent(parent)
        print(f"[CREATE] Created folder '{name}' under '{parent.Name()}'")
        return folder

    # No parent — global lookup
    existing = RDK.Item(name, ITEM_TYPE_FOLDER)
    if existing.Valid():
        return existing

    RDK.Command("AddFolder", name)
    folder = RDK.Item(name, ITEM_TYPE_FOLDER)
    assert folder.Valid(), f"Failed to create folder '{name}'"
    print(f"[CREATE] Created folder '{name}'")
    return folder


# ── HELPERS ───────────────────────────────────────────────────────────────────

def items_matching(RDK, item_type, pattern):
    all_items = RDK.ItemList(item_type)
    if pattern is None:
        return all_items
    return [it for it in all_items if pattern.match(it.Name())]


def move_item_to_folder(item, folder):
    parent = item.Parent()
    if parent.Valid() and parent.Name() == folder.Name():
        return False
    world_pose = item.PoseAbs()
    item.setParent(folder)
    item.setPoseAbs(world_pose)
    return True


def discover_cone_names(RDK):
    all_targets = RDK.ItemList(ITEM_TYPE_TARGET)
    target_names = {t.Name() for t in all_targets}

    grab_names = {n for n in target_names if GRAB_PATTERN.match(n)}
    string_grab_names = {n for n in target_names if STRING_GRAB_PATTERN.match(n)}

    grab_cones = {n.rsplit("_grab", 1)[0] for n in grab_names}
    string_grab_cones = {n.rsplit("_string_grab", 1)[0] for n in string_grab_names}

    paired = sorted(grab_cones & string_grab_cones)
    return paired


# ── PHASE 2 ──────────────────────────────────────────────────────────────────

def create_programs(RDK, robot, cone_names, programs_folder):
    created = 0
    skipped = 0

    for cone_name in cone_names:
        existing = RDK.Item(cone_name, ITEM_TYPE_PROGRAM)
        if existing.Valid():
            skipped += 1
            continue
        prog = RDK.AddProgram(cone_name, robot)
        prog.setParent(programs_folder)
        created += 1

    total = created + skipped
    print(f"[programs] {total} program(s): {created} created, {skipped} already exist")


# ── PHASE 3 ──────────────────────────────────────────────────────────────────

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


# ── PHASE 4: IK SOLVING ─────────────────────────────────────────────────────

# OptimAxes config for 6-DOF robot (no j7). Algorithm 3 = damped least squares.
_OPT_AXES_6DOF = {
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1,
    "RelOn_4": 1, "RelOn_5": 1, "RelOn_6": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50,
    "RelW_4": 50, "RelW_5": 50, "RelW_6": 50,
}

HOME_SEED_6DOF = [0.0] * 6


def try_ik(robot, pose):
    """Attempt IK using OptimAxes + MoveJ (same approach as robert_end_checker).

    Returns joints list or None.
    """
    robot.setParam("OptimAxes", _OPT_AXES_6DOF)
    robot.setJoints(HOME_SEED_6DOF)
    try:
        robot.MoveJ(pose)
        raw = robot.Joints()
        try:
            joints = raw.list()
        except AttributeError:
            joints = list(raw)
        robot.setJoints(HOME_SEED_6DOF)
        if len(joints) < 6:
            return None
        if all(abs(j) < 1e-6 for j in joints):
            return None
        return joints
    except Exception:
        robot.setJoints(HOME_SEED_6DOF)
        return None


def solve_with_z_sweep(robot, target_pose, step_deg):
    """Try original pose, then sweep Z rotations. Returns (solved_pose, joints, angle_deg) or (None, None, None)."""
    # Try original first
    joints = try_ik(robot, target_pose)
    if joints is not None:
        return target_pose, joints, 0.0

    # Sweep rotations
    n_steps = int(360 / step_deg)
    for i in range(1, n_steps):
        angle_deg = step_deg * i
        angle_rad = angle_deg * math.pi / 180.0
        rotated_pose = target_pose * rotz(angle_rad)
        joints = try_ik(robot, rotated_pose)
        if joints is not None:
            return rotated_pose, joints, angle_deg

    return None, None, None


def target_exists_in_folder(folder, name):
    """Check if a target with this name exists as a child of folder."""
    children = folder.Childs()
    for child in children:
        if child.Name() == name and child.Type() == ITEM_TYPE_TARGET:
            return True
    return False


def solve_and_create_targets(RDK, robot, config, extracted_folder,
                             before_folder, after_folder):
    """Solve IK for all grab and string_grab targets. Create solved targets + offsets.

    Returns list of (target_name, tool_name) for failures.
    """
    step_deg = config["z_rotation_step_deg"]
    offsets = config["offsets_mm"]
    ee_config = config["end_effectors"]

    # Set robot frame to world
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

            # Cache check
            if target_exists_in_folder(extracted_folder, name):
                cached += 1
                break

            # Set tool
            tool = find_tool(RDK, tool_name)
            robot.setTool(tool)

            # Get world pose
            target_pose = target_item.PoseAbs()

            # Solve
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

            # Create solved target in extracted folder
            solved_target = RDK.AddTarget(name, extracted_folder, robot)
            solved_target.setPose(solved_pose)
            solved_target.setJoints(joints)

            # Create before/after offsets for the solved pose, with IK solutions
            before_pose = solved_pose * transl(0, 0, -offset_distances["before"])
            before_joints = try_ik(robot, before_pose)
            before_target = RDK.AddTarget(
                f"offset_before_for_{name}", before_folder, robot
            )
            before_target.setPose(before_pose)
            if before_joints is not None:
                before_target.setJoints(before_joints)
            else:
                print(f"    [WARN] No IK for offset_before_for_{name}")

            after_pose = solved_pose * transl(0, 0, -offset_distances["after"])
            after_joints = try_ik(robot, after_pose)
            after_target = RDK.AddTarget(
                f"offset_after_for_{name}", after_folder, robot
            )
            after_target.setPose(after_pose)
            if after_joints is not None:
                after_target.setJoints(after_joints)
            else:
                print(f"    [WARN] No IK for offset_after_for_{name}")

            solved += 1
            break

    print(f"[solve] {solved} solved, {cached} cached, {len(failures)} failed")
    return failures


def write_failure_report(failures):
    """Write failure report to ik_failure_report.txt."""
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


# ── PHASE 5: targets_to_use.json + program population ───────────────────────

def build_targets_to_use(RDK, robot, cone_names, failures, offsets_config,
                         rotated_extracted, rotated_before, rotated_after):
    """Build the targets_to_use folder in RoboDK and a JSON summary.

    For each cone where both grab and string_grab solved, copies the solved
    targets and their offsets into the targets_to_use folder so everything
    needed for programs is in one place.

    Folder structure:
      targets_to_use/
        extracted/
          <cone>_grab
          <cone>_string_grab
        auto_generated_offsets/
          before/
            offset_before_for_<cone>_grab
            offset_before_for_<cone>_string_grab
          after/
            offset_after_for_<cone>_grab
            offset_after_for_<cone>_string_grab
    """
    failed_names = {name for name, _ in failures}

    solved_children = set()
    for child in rotated_extracted.Childs():
        solved_children.add(child.Name())

    # Create folder structure
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

    # Track what's already in each folder for caching
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

        # Copy targets into targets_to_use/extracted
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

        # Copy before/after offsets
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


def find_target_in_folder(folder, name):
    """Find a target by name within a folder's children."""
    for child in folder.Childs():
        if child.Name() == name and child.Type() == ITEM_TYPE_TARGET:
            return child
    return None


def get_or_create_home_target(RDK, robot, rotated_extracted):
    """Get or create a 'home' target at all-zeros joints in the extracted folder."""
    home = find_target_in_folder(rotated_extracted, "home")
    if home is not None:
        return home
    home = RDK.AddTarget("home", rotated_extracted, robot)
    home.setJoints(HOME_SEED_6DOF)
    home.setAsJointTarget()
    return home


def to_robodk_path(path):
    """Convert path for RoboDK. Handles WSL /mnt/c/... -> C:/... conversion."""
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


def install_replace_cones_script(RDK, programs_folder):
    """Add replace_cone.py as a Python program in the RoboDK programs folder."""
    existing = RDK.Item("replace_cone", ITEM_TYPE_PROGRAM_PYTHON)
    if existing.Valid():
        print("[CACHE] replace_cone script already in station")
        return

    script_path = to_robodk_path(REPLACE_CONE_SCRIPT)
    item = RDK.AddFile(script_path, programs_folder)
    if item.Valid():
        print(f"[CREATE] Added replace_cone script to programs folder")
    else:
        print(f"[WARN] Failed to add replace_cone.py to station")


ATTACH_SCRIPTS_DIR = os.path.join(SCRIPT_DIR, "attach_scripts")


def _write_attach_script(cone_name):
    """Write a tiny Python script that attaches a specific cone to the pickup tool."""
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


def find_cone_object(RDK, cone_name):
    """Find the cone mesh object in the station by cone name."""
    obj = RDK.Item(cone_name, ITEM_TYPE_OBJECT)
    if obj.Valid():
        return obj
    return None


def populate_programs(RDK, robot, targets_to_use, ee_config,
                      extracted_folder, before_folder, after_folder):
    """Populate each cone's program with MoveL instructions.

    Sequence per program:
    1. MoveJ to home
    2. Set knotting tool → MoveJ before string_grab → MoveL string_grab → MoveL after string_grab
    3. Set pickup tool → MoveL before grab → MoveL grab → [attach cone] → MoveL after grab
    4. MoveJ to home

    After the grab target is reached, the cone object is parented to the pickup
    tool so it moves with the robot for the remainder of the sequence.

    Targets are looked up from the targets_to_use folder subfolders.
    """
    populated = 0
    skipped = 0

    knotting_tool = find_tool(RDK, ee_config["string_grab"])
    pickup_tool = find_tool(RDK, ee_config["grab"])
    home_target = get_or_create_home_target(RDK, robot, extracted_folder)

    # Save original cone poses (before any attaching happens)
    cone_poses = {}
    if os.path.exists(CONE_POSES_PATH):
        with open(CONE_POSES_PATH, "r", encoding="utf-8") as f:
            cone_poses = json.load(f)

    all_objects = RDK.ItemList(ITEM_TYPE_OBJECT)
    for obj in all_objects:
        name = obj.Name()
        if CONE_PATTERN.match(name) and name not in cone_poses:
            cone_poses[name] = Pose_2_TxyzRxyz(obj.PoseAbs())

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

        # Check if program already has instructions (caching)
        n_ins = prog.InstructionCount()
        if n_ins > 0:
            skipped += 1
            continue

        # Look up all 6 targets from the targets_to_use folders
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

        # Build the program — MoveJ to home, then tool changes + MoveL sequences
        prog.MoveJ(home_target)

        # String grab sequence with knotting tool
        prog.setPoseTool(knotting_tool)
        prog.MoveJ(sg_before)
        prog.MoveL(sg_target)
        prog.MoveL(sg_after)

        # Pickup grab sequence with pickup tool
        prog.setPoseTool(pickup_tool)
        prog.MoveL(gr_before)
        prog.MoveL(gr_target)

        # Create per-cone attach script and call it during playback
        attach_name = f"attach_{cone_name}"
        attach_prog = RDK.Item(attach_name, ITEM_TYPE_PROGRAM_PYTHON)
        if not attach_prog.Valid():
            attach_script = _write_attach_script(cone_name)
            attach_path = to_robodk_path(attach_script)
            attach_prog = RDK.AddFile(attach_path)
            if attach_prog.Valid():
                progs = RDK.Item("programs", ITEM_TYPE_FOLDER)
                attach_folder = get_or_create_folder(RDK, "attach_scripts", parent=progs)
                attach_prog.setParent(attach_folder)

        prog.RunInstruction(attach_name, INSTRUCTION_CALL_PROGRAM)

        prog.MoveL(gr_after)

        # Return home
        prog.MoveJ(home_target)

        populated += 1
        print(f"  [OK]   {cone_name} — 10 instructions added")

    total = populated + skipped
    print(f"[programs] {total} program(s): {populated} populated, {skipped} skipped")


# ── MAIN ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Set up extracted station for base cone movement testing")
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help=f"Config JSON (default: {os.path.basename(DEFAULT_CONFIG)})")
    ap.add_argument("--skip", nargs="*", default=[],
                    help="Phases to skip, e.g. --skip 1 2 3 4 5 6")
    args = ap.parse_args()

    skip = {s.upper() for s in args.skip}
    if skip:
        print(f"[SKIP] Skipping phases: {', '.join(sorted(skip))}")

    config = load_config(args.config)
    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)

    # ── Phase 1: organize items into folders ──────────────────────────────
    if "1" not in skip:
        print("\n── Phase 1: Organize items into folders ──")
        for folder_name, spec in FOLDER_DEFS.items():
            folder = get_or_create_folder(RDK, folder_name)
            matched = items_matching(RDK, spec["item_type"], spec["filter"])

            moved = 0
            skipped_count = 0
            for item in matched:
                if move_item_to_folder(item, folder):
                    moved += 1
                else:
                    skipped_count += 1

            total = moved + skipped_count
            print(f"[{folder_name}] {total} item(s): {moved} moved, {skipped_count} already in place")

        for folder_name in FOLDER_DEFS:
            folder = RDK.Item(folder_name, ITEM_TYPE_FOLDER)
            if folder.Valid():
                folder.setVisible(True)
    else:
        print("\n── Phase 1: SKIPPED ──")

    # ── Phase 2: create programs for grab/string_grab pairs ───────────────
    # Always discover cone names (needed by later phases)
    cone_names = discover_cone_names(RDK)

    if "2" not in skip:
        print("\n── Phase 2: Create programs for base cone pairs ──")
        print(f"[DISCOVER] {len(cone_names)} base cone(s) with grab + string_grab pairs")
        for name in cone_names:
            print(f"  - {name}")

        programs_folder = get_or_create_folder(RDK, "programs")
        programs_folder.setVisible(True)
        create_programs(RDK, robot, cone_names, programs_folder)
    else:
        print("\n── Phase 2: SKIPPED ──")

    # ── Phase 3: create offset targets ────────────────────────────────────
    if "3" not in skip:
        print("\n── Phase 3: Create offset targets (before/after) ──")
        offsets_parent = get_or_create_folder(RDK, "auto_generated_offsets")
        before_folder = get_or_create_folder(RDK, "before", parent=offsets_parent)
        after_folder = get_or_create_folder(RDK, "after", parent=offsets_parent)
        offsets_parent.setVisible(True)
        before_folder.setVisible(True)
        after_folder.setVisible(True)

        create_offset_targets(RDK, robot, config["offsets_mm"], before_folder, after_folder)
    else:
        print("\n── Phase 3: SKIPPED ──")

    # ── Phase 4: solve IK with Z-rotation sweep ──────────────────────────
    # Always set up folder refs (needed by Phase 5A)
    rotated_root = get_or_create_folder(RDK, "targets_rotated_for_solution")
    rotated_extracted = get_or_create_folder(RDK, "extracted", parent=rotated_root)
    rotated_offsets = get_or_create_folder(
        RDK, "auto_generated_offsets", parent=rotated_root
    )
    rotated_before = get_or_create_folder(RDK, "before", parent=rotated_offsets)
    rotated_after = get_or_create_folder(RDK, "after", parent=rotated_offsets)

    failures = []

    if "4" not in skip:
        print("\n── Phase 4: Solve IK for all targets ──")
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
        print("\n── Phase 4: SKIPPED ──")

    # ── Phase 5: build targets_to_use folder + JSON ─────────────────────
    if "5" not in skip:
        print("\n── Phase 5: Build targets_to_use folder ──")
        targets_to_use = build_targets_to_use(
            RDK, robot, cone_names, failures, config["offsets_mm"],
            rotated_extracted, rotated_before, rotated_after
        )
        write_targets_to_use(targets_to_use)
    else:
        print("\n── Phase 5: SKIPPED ──")

    # ── Phase 6: populate programs ────────────────────────────────────
    if "6" not in skip:
        print("\n── Phase 6: Populate programs ──")

        # Get folder refs from the RoboDK station — these must already exist
        ttu_root = RDK.Item("targets_to_use", ITEM_TYPE_FOLDER)
        assert ttu_root.Valid(), \
            "targets_to_use folder not found — run Phase 5 first"

        ttu_extracted = get_or_create_folder(RDK, "extracted", parent=ttu_root)
        ttu_offsets = get_or_create_folder(RDK, "auto_generated_offsets", parent=ttu_root)
        ttu_before = get_or_create_folder(RDK, "before", parent=ttu_offsets)
        ttu_after = get_or_create_folder(RDK, "after", parent=ttu_offsets)

        # Build targets_to_use dict from the folder contents
        extracted_targets = {c.Name(): c for c in ttu_extracted.Childs()
                            if c.Type() == ITEM_TYPE_TARGET}
        assert len(extracted_targets) > 0, \
            "targets_to_use/extracted is empty — run Phase 5 first"
        print(f"[INFO] Found {len(extracted_targets)} target(s) in targets_to_use/extracted")

        # Reconstruct cone → target mapping from folder contents
        targets_to_use = {}
        for name in extracted_targets:
            if GRAB_PATTERN.match(name):
                cone = name.rsplit("_grab", 1)[0]
                kind = "grab"
            elif STRING_GRAB_PATTERN.match(name):
                cone = name.rsplit("_string_grab", 1)[0]
                kind = "string_grab"
            else:
                continue

            if cone not in targets_to_use:
                targets_to_use[cone] = {}
            targets_to_use[cone][kind] = {
                "target": name,
                "before": f"offset_before_for_{name}",
                "after": f"offset_after_for_{name}",
            }

        # Only populate cones that have both grab and string_grab
        complete = {c: v for c, v in targets_to_use.items()
                    if "grab" in v and "string_grab" in v}

        print(f"[INFO] {len(complete)} cone(s) with both grab + string_grab")

        ee_config = config["end_effectors"]
        populate_programs(
            RDK, robot, complete, ee_config,
            ttu_extracted, ttu_before, ttu_after
        )

        # Add replace_cone.py to the station so user can restore cones from GUI
        programs_folder = RDK.Item("programs", ITEM_TYPE_FOLDER)
        if programs_folder.Valid():
            install_replace_cones_script(RDK, programs_folder)
    else:
        print("\n── Phase 6: SKIPPED ──")

    print("\n[DONE] Station setup complete.")


if __name__ == "__main__":
    main()
