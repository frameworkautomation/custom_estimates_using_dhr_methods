"""
Build remove_cone / add_cone programs for machine cones in RoboDK.

Discovers cone frames under Machine{N}Base/top_plate_frame, solves IK for each
child frame (cut, grip, suck targets), then creates RoboDK programs with the
full movement sequences including cone attach/detach.

Usage:
    python robert_checker_stuff/setup_machine_cone_programs.py
    python robert_checker_stuff/setup_machine_cone_programs.py --robodk-ip 172.23.208.1
"""

import sys
import os
import json
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_TARGET, ITEM_TYPE_OBJECT,
    ITEM_TYPE_FOLDER, ITEM_TYPE_PROGRAM, ITEM_TYPE_PROGRAM_PYTHON,
    ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME,
    INSTRUCTION_CALL_PROGRAM,
)
from robodk.robomath import Pose_2_TxyzRxyz

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(SCRIPT_DIR, "setup_machine_cone_programs_config.json")
CONE_POSES_PATH = os.path.join(SCRIPT_DIR, "machine_cone_original_poses.json")

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]
HOME_SEED = [0.0] * 7
NUM_JOINTS = 7
J7_TOL_MM = 10.0

# OptimAxes dict for locked j7 (from robert_end_checker.py)
_OPT_AXES_LOCKED = {
    "AbsOn_7": 1, "AbsW_7": 100,
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
    "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
    "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
}


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
            print(f"[INFO] Found robot: '{name}'")
            return r
    raise RuntimeError(f"Robot not found. Tried: {ROBOT_NAMES}")


def find_tool(RDK, name):
    tool = RDK.Item(name, ITEM_TYPE_TOOL)
    assert tool.Valid(), f"Tool '{name}' not found in station"
    return tool


# ── FOLDERS ───────────────────────────────────────────────────────────────────

def get_or_create_folder(RDK, name, parent=None):
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


# ── IK SOLVING ────────────────────────────────────────────────────────────────

def _solve_ik_locked_j7(robot, RDK, pose, j7_target):
    """Solve IK with j7 constrained using OptimAxes + MoveJ (7-DOF)."""
    props = dict(_OPT_AXES_LOCKED)
    props["AbsJnt_7"] = j7_target
    robot.setParam("OptimAxes", props)

    robot.setJoints(HOME_SEED)
    try:
        robot.MoveJ(pose)
        raw = robot.Joints()
        try:
            joints = raw.list()
        except AttributeError:
            joints = list(raw)
        robot.setJoints(HOME_SEED)
        if len(joints) >= 7 and abs(joints[6] - j7_target) > J7_TOL_MM:
            return [], False
        return joints, True
    except Exception:
        robot.setJoints(HOME_SEED)
        return [], False


# ── TOOL MAPPING ──────────────────────────────────────────────────────────────

def tool_for_child(child_name, tools_config):
    """Return the tool name for a given child frame name."""
    if child_name.startswith("cut"):
        return tools_config["cutting"]
    elif child_name.startswith("grip"):
        return tools_config["pickup"]
    elif child_name.startswith("suck"):
        return tools_config["knotting"]
    raise ValueError(f"Unknown child frame prefix: '{child_name}'")


# ── WSL PATH CONVERSION ──────────────────────────────────────────────────────

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


# ── PHASE 1: DISCOVER & VALIDATE ─────────────────────────────────────────────

def discover_cone_frames(RDK, config):
    """Find cone frames under Machine{N}Base/top_plate_frame.

    Returns dict: cone_name -> {child_name -> frame_item}
    """
    machine_num = config["machine_number"]
    top_plate_name = config["top_plate_frame"]
    cone_frame_names = config["cone_frames"]

    # Collect all unique child frame names from config
    all_child_names = set()
    for phase_children in config["child_frames"].values():
        all_child_names.update(phase_children)

    # Find top_plate_frame in station
    top_plate = RDK.Item(top_plate_name, ITEM_TYPE_FRAME)
    assert top_plate.Valid(), \
        f"'{top_plate_name}' not found in station"
    print(f"[INFO] Found '{top_plate_name}'")

    cones = {}
    for cone_name in cone_frame_names:
        cone_frame = RDK.Item(cone_name, ITEM_TYPE_FRAME)
        assert cone_frame.Valid(), \
            f"Cone frame '{cone_name}' not found in station"

        children = {}
        for child_name in all_child_names:
            # Search children of cone_frame for this child
            child = None
            for c in cone_frame.Childs():
                if c.Name() == child_name and c.Type() == ITEM_TYPE_FRAME:
                    child = c
                    break
            assert child is not None, \
                f"Child frame '{child_name}' not found under '{cone_name}'"
            children[child_name] = child

        cones[cone_name] = children
        print(f"  [OK] {cone_name}: {len(children)} child frames found")

    return cones


# ── PHASE 2: SOLVE IK & CREATE TARGETS ───────────────────────────────────────

def solve_and_create_targets(RDK, robot, cones, config):
    """Solve IK for each unique child frame, create targets under them.

    Returns dict: cone_name -> {child_name -> target_item_or_None}
    Also returns list of (cone_name, child_name) failures.
    """
    j7_value = config["j7_value"]
    tools_config = config["tools"]

    # Set robot frame to world
    world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
    if not world_frame.Valid():
        from robodk.robomath import eye
        station = RDK.ActiveStation()
        world_frame = RDK.AddFrame("WorldFrame", station)
        world_frame.setPose(eye(4))
    robot.setPoseFrame(world_frame)

    targets = {}
    failures = []
    solved = 0
    cached = 0

    for cone_name, children in cones.items():
        targets[cone_name] = {}

        for child_name, child_frame in children.items():
            target_name = f"target_{child_name}"

            # Check if target already exists under this child frame
            existing = None
            for c in child_frame.Childs():
                if c.Name() == target_name and c.Type() == ITEM_TYPE_TARGET:
                    existing = c
                    break

            if existing is not None:
                targets[cone_name][child_name] = existing
                cached += 1
                continue

            # Set the correct tool
            tool_name = tool_for_child(child_name, tools_config)
            tool = find_tool(RDK, tool_name)
            robot.setTool(tool)

            # Get world pose of the child frame
            pose = child_frame.PoseAbs()

            # Solve IK with j7 locked
            joints, ok = _solve_ik_locked_j7(robot, RDK, pose, j7_value)

            if not ok:
                print(f"  [FAIL] {cone_name}/{child_name} — no IK (tool={tool_name})")
                failures.append((cone_name, child_name))
                targets[cone_name][child_name] = None
                continue

            # Create target under the child frame
            tgt = RDK.AddTarget(target_name, child_frame, robot)
            tgt.setPose(child_frame.Pose())  # relative to parent frame
            tgt.setJoints(joints)
            targets[cone_name][child_name] = tgt
            solved += 1
            print(f"  [OK]   {cone_name}/{child_name} (tool={tool_name})")

    total = solved + cached + len(failures)
    print(f"[solve] {total} target(s): {solved} solved, {cached} cached, {len(failures)} failed")
    return targets, failures


# ── PHASE 3: HOME & RAIL TARGETS ─────────────────────────────────────────────

def create_home_target(RDK, robot, folder, config):
    """Create the home joint target. Returns target item."""
    name = "home"
    for child in folder.Childs():
        if child.Name() == name and child.Type() == ITEM_TYPE_TARGET:
            return child

    home_joints = config["home_joints"]
    tgt = RDK.AddTarget(name, folder, robot)
    tgt.setJoints(home_joints)
    tgt.setAsJointTarget()
    print(f"[CREATE] home target")
    return tgt


def create_home_on_rail_target(RDK, robot, folder, config):
    """Create home-on-rail target (j7 at configured value, arm at home)."""
    name = "home_on_rail"
    for child in folder.Childs():
        if child.Name() == name and child.Type() == ITEM_TYPE_TARGET:
            return child

    joints = list(config["home_joints"])
    joints[6] = config["j7_value"]
    tgt = RDK.AddTarget(name, folder, robot)
    tgt.setJoints(joints)
    tgt.setAsJointTarget()
    print(f"[CREATE] home_on_rail target (j7={config['j7_value']})")
    return tgt


# ── PHASE 4: ATTACH/DETACH SCRIPTS ───────────────────────────────────────────

ATTACH_SCRIPTS_DIR = os.path.join(SCRIPT_DIR, "machine_cone_attach_scripts")


def _write_attach_script(cone_name, pickup_tool_name):
    os.makedirs(ATTACH_SCRIPTS_DIR, exist_ok=True)
    script_path = os.path.join(ATTACH_SCRIPTS_DIR, f"attach_{cone_name}.py")
    if not os.path.exists(script_path):
        with open(script_path, "w", encoding="utf-8") as f:
            f.write(f'''from robodk.robolink import Robolink, ITEM_TYPE_TOOL, ITEM_TYPE_OBJECT
RDK = Robolink()
tool = RDK.Item("{pickup_tool_name}", ITEM_TYPE_TOOL)
cone = RDK.Item("{cone_name}", ITEM_TYPE_OBJECT)
if tool.Valid() and cone.Valid():
    cone.setParentStatic(tool)
    print("Attached: {cone_name}")
else:
    print("Failed to attach {cone_name}")
''')
    return script_path


def _write_detach_script(cone_name):
    os.makedirs(ATTACH_SCRIPTS_DIR, exist_ok=True)
    script_path = os.path.join(ATTACH_SCRIPTS_DIR, f"detach_{cone_name}.py")
    poses_path_robodk = to_robodk_path(CONE_POSES_PATH)
    if not os.path.exists(script_path):
        with open(script_path, "w", encoding="utf-8") as f:
            f.write(f'''import json
from robodk.robolink import Robolink, ITEM_TYPE_OBJECT, ITEM_TYPE_FRAME
from robodk.robomath import TxyzRxyz_2_Pose
RDK = Robolink()
cone = RDK.Item("{cone_name}", ITEM_TYPE_OBJECT)
if not cone.Valid():
    print("Cone not found: {cone_name}")
else:
    with open(r"{poses_path_robodk}", "r") as f:
        poses = json.load(f)
    info = poses["{cone_name}"]
    parent = RDK.Item(info["parent"], ITEM_TYPE_FRAME)
    if not parent.Valid():
        parent = RDK.Item(info["parent"])
    cone.setParentStatic(parent)
    cone.setPose(TxyzRxyz_2_Pose(info["pose"]))
    print("Detached: {cone_name}")
''')
    return script_path


def save_cone_original_poses(RDK, cones):
    """Save original parent + pose for each cone object (for detach scripts)."""
    cone_poses = {}
    if os.path.exists(CONE_POSES_PATH):
        with open(CONE_POSES_PATH, "r", encoding="utf-8") as f:
            cone_poses = json.load(f)

    updated = 0
    for cone_name in cones:
        if cone_name in cone_poses:
            continue
        obj = RDK.Item(cone_name, ITEM_TYPE_OBJECT)
        if not obj.Valid():
            print(f"  [WARN] Object '{cone_name}' not found — can't save pose")
            continue
        cone_poses[cone_name] = {
            "pose": Pose_2_TxyzRxyz(obj.Pose()),
            "parent": obj.Parent().Name() if obj.Parent().Valid() else "",
        }
        updated += 1

    if cone_poses:
        with open(CONE_POSES_PATH, "w", encoding="utf-8") as f:
            json.dump(cone_poses, f, indent=2)
    print(f"[SAVE] Cone poses: {len(cone_poses)} total, {updated} new")


def create_helper_scripts(RDK, cones, config, programs_folder):
    """Create attach/detach Python script programs per cone in RoboDK."""
    pickup_tool_name = config["tools"]["pickup"]
    created = 0
    cached = 0

    for cone_name in cones:
        # Attach script
        attach_prog_name = f"attach_{cone_name}"
        existing = RDK.Item(attach_prog_name, ITEM_TYPE_PROGRAM_PYTHON)
        if existing.Valid():
            cached += 1
        else:
            script_path = _write_attach_script(cone_name, pickup_tool_name)
            prog = RDK.AddFile(to_robodk_path(script_path))
            if prog.Valid():
                prog.setParent(programs_folder)
                created += 1

        # Detach script
        detach_prog_name = f"detach_{cone_name}"
        existing = RDK.Item(detach_prog_name, ITEM_TYPE_PROGRAM_PYTHON)
        if existing.Valid():
            cached += 1
        else:
            script_path = _write_detach_script(cone_name)
            prog = RDK.AddFile(to_robodk_path(script_path))
            if prog.Valid():
                prog.setParent(programs_folder)
                created += 1

    print(f"[helper_scripts] {created + cached} script(s): {created} created, {cached} cached")


# ── PHASE 5: POPULATE PROGRAMS ────────────────────────────────────────────────

def _get_target(targets, cone_name, child_name):
    """Get a solved target, or None if it failed."""
    return targets.get(cone_name, {}).get(child_name)


def populate_remove_cone(RDK, robot, prog, cone_name, targets, config,
                         home_target, home_on_rail_target):
    """Populate a remove_cone program with movement instructions."""
    seq = config["remove_cone_sequence"]
    tools_config = config["tools"]

    # 1. MoveJ to home
    prog.MoveJ(home_target)

    # 2. MoveJ to home_on_rail (j7 positioning)
    prog.MoveJ(home_on_rail_target)

    # 3. Cut phase (cutting tool)
    cutting_tool = find_tool(RDK, tools_config["cutting"])
    prog.setPoseTool(cutting_tool)
    for child_name in seq["cut"]:
        tgt = _get_target(targets, cone_name, child_name)
        assert tgt is not None, f"Missing target for {cone_name}/{child_name}"
        prog.MoveL(tgt)

    # 4. Grip phase (pickup tool)
    pickup_tool = find_tool(RDK, tools_config["pickup"])
    prog.setPoseTool(pickup_tool)
    grip_seq = seq["grip"]
    for i, child_name in enumerate(grip_seq):
        tgt = _get_target(targets, cone_name, child_name)
        assert tgt is not None, f"Missing target for {cone_name}/{child_name}"
        prog.MoveL(tgt)
        # Attach cone after reaching grip (3rd in sequence: index 2)
        if i == 2 and child_name == "grip":
            prog.RunInstruction(f"attach_{cone_name}", INSTRUCTION_CALL_PROGRAM)

    # 5. MoveJ to home on rail
    prog.MoveJ(home_on_rail_target)

    # 6. MoveJ to home
    prog.MoveJ(home_target)


def populate_add_cone(RDK, robot, prog, cone_name, targets, config,
                      home_target, home_on_rail_target):
    """Populate an add_cone program with movement instructions."""
    seq = config["add_cone_sequence"]
    tools_config = config["tools"]

    # 1. MoveJ to home
    prog.MoveJ(home_target)

    # 2. MoveJ to home_on_rail (j7 positioning)
    prog.MoveJ(home_on_rail_target)

    # 3. Grip phase (pickup tool — robot carrying cone)
    pickup_tool = find_tool(RDK, tools_config["pickup"])
    prog.setPoseTool(pickup_tool)
    grip_seq = seq["grip"]
    for i, child_name in enumerate(grip_seq):
        tgt = _get_target(targets, cone_name, child_name)
        assert tgt is not None, f"Missing target for {cone_name}/{child_name}"
        prog.MoveL(tgt)
        # Detach cone after reaching grip (3rd in sequence: index 2)
        if i == 2 and child_name == "grip":
            prog.RunInstruction(f"detach_{cone_name}", INSTRUCTION_CALL_PROGRAM)

    # 4. Suck phase (knotting tool)
    knotting_tool = find_tool(RDK, tools_config["knotting"])
    prog.setPoseTool(knotting_tool)
    for child_name in seq["suck"]:
        tgt = _get_target(targets, cone_name, child_name)
        assert tgt is not None, f"Missing target for {cone_name}/{child_name}"
        prog.MoveL(tgt)

    # 5. MoveJ to home on rail
    prog.MoveJ(home_on_rail_target)

    # 6. MoveJ to home
    prog.MoveJ(home_target)


def create_and_populate_programs(RDK, robot, cones, targets, failures, config):
    """Create program hierarchy and populate with movement instructions."""
    failed_set = {(c, ch) for c, ch in failures}
    tools_config = config["tools"]

    # Determine which cones have all targets solved
    all_child_names = set()
    for phase_children in config["child_frames"].values():
        all_child_names.update(phase_children)

    viable_cones = []
    for cone_name in config["cone_frames"]:
        cone_ok = True
        for child_name in all_child_names:
            if (cone_name, child_name) in failed_set:
                cone_ok = False
                break
            if _get_target(targets, cone_name, child_name) is None:
                cone_ok = False
                break
        if cone_ok:
            viable_cones.append(cone_name)
        else:
            print(f"  [SKIP] {cone_name} — has IK failures, skipping program creation")

    if not viable_cones:
        print("[WARN] No cones with all targets solved — no programs created")
        return

    # Create folder hierarchy
    root_folder = get_or_create_folder(RDK, "machine_cone_programs")
    root_folder.setVisible(True)

    # Create home targets in root folder
    home_target = create_home_target(RDK, robot, root_folder, config)
    home_on_rail_target = create_home_on_rail_target(RDK, robot, root_folder, config)

    # Save cone poses for detach scripts
    save_cone_original_poses(RDK, config["cone_frames"])

    # Create helper scripts
    create_helper_scripts(RDK, config["cone_frames"], config, root_folder)

    populated = 0

    for cone_name in viable_cones:
        cone_folder = get_or_create_folder(RDK, cone_name, parent=root_folder)

        # ── remove_cone program ──
        remove_name = f"remove_cone_{cone_name}"
        remove_prog = RDK.Item(remove_name, ITEM_TYPE_PROGRAM)
        if remove_prog.Valid() and remove_prog.InstructionCount() > 0:
            print(f"  [CACHE] {remove_name}")
        else:
            if not remove_prog.Valid():
                remove_prog = RDK.AddProgram(remove_name, robot)
                remove_prog.setParent(cone_folder)
            populate_remove_cone(RDK, robot, remove_prog, cone_name, targets,
                                 config, home_target, home_on_rail_target)
            print(f"  [OK]   {remove_name}")
            populated += 1

        # ── add_cone program ──
        add_name = f"add_cone_{cone_name}"
        add_prog = RDK.Item(add_name, ITEM_TYPE_PROGRAM)
        if add_prog.Valid() and add_prog.InstructionCount() > 0:
            print(f"  [CACHE] {add_name}")
        else:
            if not add_prog.Valid():
                add_prog = RDK.AddProgram(add_name, robot)
                add_prog.setParent(cone_folder)
            populate_add_cone(RDK, robot, add_prog, cone_name, targets,
                              config, home_target, home_on_rail_target)
            print(f"  [OK]   {add_name}")
            populated += 1

        # ── remove_add combo program ──
        combo_name = f"remove_add_{cone_name}"
        combo_prog = RDK.Item(combo_name, ITEM_TYPE_PROGRAM)
        if combo_prog.Valid() and combo_prog.InstructionCount() > 0:
            print(f"  [CACHE] {combo_name}")
        else:
            if not combo_prog.Valid():
                combo_prog = RDK.AddProgram(combo_name, robot)
                combo_prog.setParent(cone_folder)
            combo_prog.RunInstruction(remove_name, INSTRUCTION_CALL_PROGRAM)
            combo_prog.RunInstruction(add_name, INSTRUCTION_CALL_PROGRAM)
            print(f"  [OK]   {combo_name}")
            populated += 1

    # ── run_all program ──
    run_all_name = "run_all"
    run_all = RDK.Item(run_all_name, ITEM_TYPE_PROGRAM)
    if run_all.Valid() and run_all.InstructionCount() > 0:
        print(f"  [CACHE] {run_all_name}")
    else:
        if not run_all.Valid():
            run_all = RDK.AddProgram(run_all_name, robot)
            run_all.setParent(root_folder)
        for cone_name in viable_cones:
            run_all.RunInstruction(f"remove_add_{cone_name}", INSTRUCTION_CALL_PROGRAM)
        print(f"  [OK]   {run_all_name} ({len(viable_cones)} cones)")
        populated += 1

    print(f"[programs] {populated} program(s) populated")


# ── MAIN ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Build remove/add cone programs for machine cones"
    )
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help=f"Config JSON (default: {os.path.basename(DEFAULT_CONFIG)})")
    args = ap.parse_args()

    # Load config
    assert os.path.exists(args.config), f"Config not found: {args.config}"
    with open(args.config, "r", encoding="utf-8") as f:
        config = json.load(f)
    print(f"[CONFIG] Machine {config['machine_number']}, j7={config['j7_value']}")
    print(f"[CONFIG] Cones: {config['cone_frames']}")

    # Connect
    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)

    # Verify 7 DOF
    try:
        n_dof = len(robot.Joints().list())
    except AttributeError:
        n_dof = len(list(robot.Joints()))
    assert n_dof == 7, f"Expected 7 DOF robot, got {n_dof}"
    print(f"[INFO] Robot has {n_dof} DOF")

    # Phase 1: Discover & validate cone frames
    print("\n── Phase 1: Discover cone frames ──")
    cones = discover_cone_frames(RDK, config)

    # Phase 2: Solve IK & create targets
    print("\n── Phase 2: Solve IK & create targets ──")
    targets, failures = solve_and_create_targets(RDK, robot, cones, config)

    if failures:
        print(f"\n[WARN] {len(failures)} IK failure(s):")
        for cone_name, child_name in failures:
            print(f"  - {cone_name}/{child_name}")

    # Phase 3: Create & populate programs
    print("\n── Phase 3: Create & populate programs ──")
    create_and_populate_programs(RDK, robot, cones, targets, failures, config)

    print("\n[DONE] Machine cone programs setup complete.")


if __name__ == "__main__":
    main()
