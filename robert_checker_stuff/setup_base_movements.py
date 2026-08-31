"""
Build movement sequences for base cone pickup in the extracted station.

Expects organize_station.py to have been run first (folders, programs,
offset targets all exist under Offset_relative_to_schematic).

Phase 1 — solve IK for all targets with Z-axis rotation sweep:
  - Try original pose first; if it fails, sweep Z rotations per config step
  - grab targets solved with pickup tool, string_grab with knotting tool
  - Solved targets + offsets stored in targets_rotated_for_solution/
  - Failures written to ik_failure_report.txt

Phase 2 — build targets_to_use folder + JSON:
  - Copies solved targets from Phase 1 into a single folder for programs

Phase 3 — create per-cone attach scripts:
  - One script per cone in attach_scripts/ folder
  - Each script parents its specific cone to the pickup tool during playback

Phase 4 — populate programs with movement instructions:
  - Each cone's program gets MoveL instructions:
    home → knotting(before→string_grab→after) → pickup(before→grab→after)

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
from robodk.robomath import transl, rotz, Pose_2_TxyzRxyz

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


# ── PHASE 1: IK SOLVING ─────────────────────────────────────────────────────

_OPT_AXES_6DOF = {
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1,
    "RelOn_4": 1, "RelOn_5": 1, "RelOn_6": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50,
    "RelW_4": 50, "RelW_5": 50, "RelW_6": 50,
}

HOME_SEED_6DOF = [0.0] * 6


def try_ik(robot, pose):
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
    joints = try_ik(robot, target_pose)
    if joints is not None:
        return target_pose, joints, 0.0

    n_steps = int(360 / step_deg)
    for i in range(1, n_steps):
        angle_deg = step_deg * i
        angle_rad = angle_deg * math.pi / 180.0
        rotated_pose = target_pose * rotz(angle_rad)
        joints = try_ik(robot, rotated_pose)
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


# ── PHASE 4: populate programs ───────────────────────────────────────────────

def get_or_create_home_target(RDK, robot, extracted_folder):
    home = find_target_in_folder(extracted_folder, "home")
    if home is not None:
        return home
    home = RDK.AddTarget("home", extracted_folder, robot)
    home.setJoints(HOME_SEED_6DOF)
    home.setAsJointTarget()
    return home


def populate_programs(RDK, robot, targets_to_use, ee_config,
                      extracted_folder, before_folder, after_folder):
    populated = 0
    skipped = 0

    knotting_tool = find_tool(RDK, ee_config["string_grab"])
    pickup_tool = find_tool(RDK, ee_config["grab"])
    home_target = get_or_create_home_target(RDK, robot, extracted_folder)

    # Save original cone poses relative to parent (before any attaching happens)
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

        prog.MoveJ(home_target)

        prog.setPoseTool(knotting_tool)
        prog.MoveJ(sg_before)
        prog.MoveL(sg_target)
        prog.MoveL(sg_after)

        prog.setPoseTool(pickup_tool)
        prog.MoveL(gr_before)
        prog.MoveL(gr_target)

        # Call the per-cone attach script (created in Phase 3)
        prog.RunInstruction(f"attach_{cone_name}", INSTRUCTION_CALL_PROGRAM)

        prog.MoveL(gr_after)
        prog.MoveJ(home_target)

        populated += 1
        print(f"  [OK]   {cone_name} — 10 instructions added")

    total = populated + skipped
    print(f"[programs] {total} program(s): {populated} populated, {skipped} skipped")


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
                    help="Phases to skip, e.g. --skip 1 2 3 4")
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
    print(f"[DISCOVER] {len(cone_names)} base cone(s) with grab + string_grab pairs")

    # ── Phase 1: solve IK with Z-rotation sweep ──────────────────────
    rotated_root = get_or_create_folder(RDK, "targets_rotated_for_solution")
    rotated_extracted = get_or_create_folder(RDK, "extracted", parent=rotated_root)
    rotated_offsets = get_or_create_folder(
        RDK, "auto_generated_offsets", parent=rotated_root
    )
    rotated_before = get_or_create_folder(RDK, "before", parent=rotated_offsets)
    rotated_after = get_or_create_folder(RDK, "after", parent=rotated_offsets)

    failures = []

    if "1" not in skip:
        print("\n── Phase 1: Solve IK for all targets ──")
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
        print("\n── Phase 1: SKIPPED ──")

    # ── Phase 2: build targets_to_use folder + JSON ───────────────────
    if "2" not in skip:
        print("\n── Phase 2: Build targets_to_use folder ──")
        targets_to_use = build_targets_to_use(
            RDK, robot, cone_names, failures,
            rotated_extracted, rotated_before, rotated_after
        )
        write_targets_to_use(targets_to_use)
    else:
        print("\n── Phase 2: SKIPPED ──")

    # ── Phase 3: create attach scripts ──────────────────────────────
    if "3" not in skip:
        print("\n── Phase 3: Create attach scripts ──")
        create_attach_scripts(RDK, cone_names)
    else:
        print("\n── Phase 3: SKIPPED ──")

    # ── Phase 4: populate programs ────────────────────────────────────
    if "4" not in skip:
        print("\n── Phase 4: Populate programs ──")

        ttu_root = RDK.Item("targets_to_use", ITEM_TYPE_FOLDER)
        assert ttu_root.Valid(), \
            "targets_to_use folder not found — run Phase 2 first"

        ttu_extracted = get_or_create_folder(RDK, "extracted", parent=ttu_root)
        ttu_offsets = get_or_create_folder(RDK, "auto_generated_offsets", parent=ttu_root)
        ttu_before = get_or_create_folder(RDK, "before", parent=ttu_offsets)
        ttu_after = get_or_create_folder(RDK, "after", parent=ttu_offsets)

        extracted_targets = {c.Name(): c for c in ttu_extracted.Childs()
                            if c.Type() == ITEM_TYPE_TARGET}
        assert len(extracted_targets) > 0, \
            "targets_to_use/extracted is empty — run Phase 2 first"
        print(f"[INFO] Found {len(extracted_targets)} target(s) in targets_to_use/extracted")

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

        complete = {c: v for c, v in targets_to_use.items()
                    if "grab" in v and "string_grab" in v}
        print(f"[INFO] {len(complete)} cone(s) with both grab + string_grab")

        ee_config = config["end_effectors"]
        populate_programs(
            RDK, robot, complete, ee_config,
            ttu_extracted, ttu_before, ttu_after
        )
    else:
        print("\n── Phase 4: SKIPPED ──")

    print("\n[DONE] Station setup complete.")


if __name__ == "__main__":
    main()
