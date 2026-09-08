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

def _solve_ik_locked_j7(robot, RDK, pose, j7_target, j7_weight=100, seed=None,
                        check_j7_tol=True):
    """Solve IK with j7 constrained using OptimAxes + MoveJ (7-DOF)."""
    props = dict(_OPT_AXES_LOCKED)
    props["AbsJnt_7"] = j7_target
    props["AbsW_7"] = j7_weight
    robot.setParam("OptimAxes", props)

    if seed is None:
        seed = HOME_SEED
    robot.setJoints(seed)
    try:
        robot.MoveJ(pose)
        raw = robot.Joints()
        try:
            joints = raw.list()
        except AttributeError:
            joints = list(raw)
        robot.setJoints(HOME_SEED)
        if check_j7_tol and len(joints) >= 7 and abs(joints[6] - j7_target) > J7_TOL_MM:
            return [], False
        return joints, True
    except Exception:
        robot.setJoints(HOME_SEED)
        return [], False


# Seeds with j1 at different positions to explore arm configurations
_ALT_SEEDS = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [170.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-170.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
]


def solve_ik_with_fallback(robot, RDK, pose, j7_target):
    """Try IK with decreasing j7 constraint strength and multiple seeds.

    Strategy:
    1. Hard lock (weight=100) with default seed
    2. Hard lock with alternative seeds
    3. Softer j7 weights (50, 20, 5) with FK verification
    4. Each softer weight also tries alternative seeds
    Returns (joints, ok, j7_weight_used).
    """
    import math

    # 1. Hard lock, default seed
    joints, ok = _solve_ik_locked_j7(robot, RDK, pose, j7_target)
    if ok:
        return joints, True, 100

    # 2. Softer j7 weights with FK verification
    for w in [50, 20, 5]:
        for seed in _ALT_SEEDS:
            s = list(seed)
            s[6] = j7_target
            joints, ok = _solve_ik_locked_j7(
                robot, RDK, pose, j7_target, j7_weight=w, seed=s,
                check_j7_tol=False
            )
            if not ok or len(joints) < 7:
                continue
            # FK verify — move to joints, check TCP error
            robot.MoveJ(joints)
            achieved = robot.Pose()
            t = Pose_2_TxyzRxyz(pose)
            a = Pose_2_TxyzRxyz(achieved)
            fk_err = math.sqrt(sum((t[i] - a[i]) ** 2 for i in range(3)))
            robot.setJoints(HOME_SEED)
            if fk_err <= 50.0:
                return joints, True, w

    return [], False, 0


# ── TOOL MAPPING ──────────────────────────────────────────────────────────────

def tool_for_child(child_name, tools_config):
    """Return the tool name for a given child frame name."""
    lower = child_name.lower()
    if lower.startswith("cut"):
        return tools_config["cutting"]
    elif lower.startswith("grip"):
        return tools_config["pickup"]
    elif lower.startswith("suck"):
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

def _find_frame_recursive(parent, name):
    """Search recursively through children for a frame with the given name."""
    try:
        children = parent.Childs()
    except Exception:
        return None
    for child in children:
        try:
            if child.Name() == name and child.Type() == ITEM_TYPE_FRAME:
                return child
            found = _find_frame_recursive(child, name)
            if found is not None:
                return found
        except Exception:
            continue
    return None


def discover_cone_frames(RDK, config):
    """Find cone frames under Machine{N}Base/top_plate_frame.

    Returns dict: cone_name -> {child_name -> frame_item}
    Searches recursively — child frames may be nested under other frames.
    """
    machine_num = config["machine_number"]
    top_plate_name = config["top_plate_frame"]
    cone_frame_names = config["cone_frames"]

    # Collect all unique child frame names from config
    all_child_names = set()
    for phase_children in config["child_frames"].values():
        all_child_names.update(phase_children)

    # Navigate from Machine{N}Base to its top_plate_frame to avoid
    # hitting duplicate names from other machines
    machine_base_name = f"Machine{machine_num}Base"
    machine_base = RDK.Item(machine_base_name, ITEM_TYPE_FRAME)
    assert machine_base.Valid(), \
        f"'{machine_base_name}' not found in station"
    print(f"[INFO] Found '{machine_base_name}'")

    top_plate = _find_frame_recursive(machine_base, top_plate_name)
    assert top_plate is not None, \
        f"'{top_plate_name}' not found under '{machine_base_name}'"
    print(f"[INFO] Found '{top_plate_name}' under '{machine_base_name}'")

    cones = {}
    for cone_name in cone_frame_names:
        cone_frame = _find_frame_recursive(top_plate, cone_name)
        assert cone_frame is not None, \
            f"Cone frame '{cone_name}' not found under '{machine_base_name}/{top_plate_name}'"

        children = {}
        for child_name in all_child_names:
            child = _find_frame_recursive(cone_frame, child_name)
            assert child is not None, \
                f"Child frame '{child_name}' not found under '{cone_name}' (searched recursively)"
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
            target_name = f"target_m{config['machine_number']}_{cone_name}_{child_name}"

            # Check if target already exists
            existing = RDK.Item(target_name, ITEM_TYPE_TARGET)
            if existing.Valid():
                targets[cone_name][child_name] = existing
                cached += 1
                continue

            # Set the correct tool
            tool_name = tool_for_child(child_name, tools_config)
            tool = find_tool(RDK, tool_name)
            robot.setTool(tool)

            # Get world pose of the child frame
            pose = child_frame.PoseAbs()

            # Solve IK with j7 locked, falling back to softer constraints
            joints, ok, w_used = solve_ik_with_fallback(robot, RDK, pose, j7_value)

            if not ok:
                print(f"  [FAIL] {cone_name}/{child_name} — no IK (tool={tool_name})")
                failures.append((cone_name, child_name))
                targets[cone_name][child_name] = None
                continue

            j7_actual = joints[6] if len(joints) >= 7 else 0

            # Create cartesian target under WorldFrame with world pose
            # This way MoveL interprets the pose in world coordinates
            # (matching what we solved IK against)
            tgt = RDK.AddTarget(target_name, world_frame, robot)
            tgt.setAsCartesianTarget()
            tgt.setPose(pose)  # world pose
            tgt.setJoints(joints)  # preferred configuration / IK seed
            targets[cone_name][child_name] = tgt
            solved += 1
            w_info = f" w={w_used}" if w_used < 100 else ""
            j7_info = f" j7={j7_actual:.0f}" if w_used < 100 else ""
            print(f"  [OK]   {cone_name}/{child_name} (tool={tool_name}{w_info}{j7_info})")

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


def _write_set_optim_axes_script(j7_value):
    """Write a Python script that sets OptimAxes on the robot with soft j7 constraint."""
    os.makedirs(ATTACH_SCRIPTS_DIR, exist_ok=True)
    script_path = os.path.join(ATTACH_SCRIPTS_DIR, "set_optim_axes.py")
    if not os.path.exists(script_path):
        with open(script_path, "w", encoding="utf-8") as f:
            f.write(f'''from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
RDK = Robolink()
robot = RDK.Item("Fanuc R2000iC 125L", ITEM_TYPE_ROBOT)
if not robot.Valid():
    robot = RDK.Item("Fanuc R-2000iC/125L", ITEM_TYPE_ROBOT)
props = {{
    "AbsOn_7": 1, "AbsJnt_7": {j7_value}, "AbsW_7": 20,
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
    "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
    "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
}}
robot.setParam("OptimAxes", props)
print("OptimAxes set: j7 soft constraint at {j7_value}")
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

    # OptimAxes script
    optim_name = "set_optim_axes"
    existing = RDK.Item(optim_name, ITEM_TYPE_PROGRAM_PYTHON)
    if existing.Valid():
        cached += 1
    else:
        script_path = _write_set_optim_axes_script(config["j7_value"])
        prog = RDK.AddFile(to_robodk_path(script_path))
        if prog.Valid():
            prog.setParent(programs_folder)
            created += 1

    print(f"[helper_scripts] {created + cached} script(s): {created} created, {cached} cached")


# ── PHASE 5: GENERATE MOVEMENT SCRIPTS ────────────────────────────────────────

MOVEMENT_SCRIPTS_DIR = os.path.join(SCRIPT_DIR, "machine_cone_movement_scripts")


def _is_approach(child_name):
    """Return True if this is an approach frame (MoveJ appropriate)."""
    return "approach" in child_name.lower()


def _move_call(child_name, prev_child):
    """Return 'MoveJ' or 'MoveL' string based on movement context.

    MoveJ when: moving TO an approach and previous was NOT an action target.
    MoveL when: moving from action to approach (retract), or any precise move.
    """
    if _is_approach(child_name) and (prev_child is None or _is_approach(prev_child)):
        return "MoveJ"
    return "MoveL"


def _get_target(targets, cone_name, child_name):
    """Get a solved target, or None if it failed."""
    return targets.get(cone_name, {}).get(child_name)


def _generate_movement_script(cone_name, config, script_type):
    """Generate a Python script that executes movements via the RoboDK API.

    DHR-style: setPoseFrame(WorldFrame), setParam(OptimAxes), then
    robot.MoveJ/MoveL(frame.PoseAbs()) directly.
    """
    os.makedirs(MOVEMENT_SCRIPTS_DIR, exist_ok=True)
    machine_num = config["machine_number"]
    script_name = f"{script_type}_m{machine_num}_{cone_name}"
    script_path = os.path.join(MOVEMENT_SCRIPTS_DIR, f"{script_name}.py")

    j7_value = config["j7_value"]
    tools_config = config["tools"]
    home_joints = config["home_joints"]
    home_on_rail = list(home_joints)
    home_on_rail[6] = j7_value

    if script_type == "remove_cone":
        seq = config["remove_cone_sequence"]
    else:
        seq = config["add_cone_sequence"]

    # Build the movement lines
    lines = []
    lines.append(f'''import sys
sys.path.append("C:/RoboDK/Python")
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME
from robodk.robomath import eye

RDK = Robolink()

# Find robot
robot = RDK.Item("Fanuc R2000iC 125L", ITEM_TYPE_ROBOT)
if not robot.Valid():
    robot = RDK.Item("Fanuc R-2000iC/125L", ITEM_TYPE_ROBOT)

# Set world frame
world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
if not world_frame.Valid():
    station = RDK.ActiveStation()
    world_frame = RDK.AddFrame("WorldFrame", station)
    world_frame.setPose(eye(4))
robot.setPoseFrame(world_frame)

from robodk.robomath import Pose_2_TxyzRxyz

def find_child(parent, name):
    """Find a frame by name recursively under parent."""
    try:
        for child in parent.Childs():
            try:
                if child.Name() == name and child.Type() == 3:  # ITEM_TYPE_FRAME
                    return child
                found = find_child(child, name)
                if found is not None:
                    return found
            except:
                continue
    except:
        pass
    return None

# Find the cone frame for this script
cone_frame = find_child(RDK.Item("Machine{config['machine_number']}Base", ITEM_TYPE_FRAME), "{cone_name}")
assert cone_frame is not None, "Cone frame '{cone_name}' not found"

# Rail joint limits
joint_limits = robot.JointLimits()
try:
    j7_min = joint_limits[0].list()[6] + 10
    j7_max = joint_limits[1].list()[6] - 10
except:
    j7_min = 0
    j7_max = 9000

def set_optim_for_pose(pose):
    """Set OptimAxes with j7 locked to the target's X position (rail axis).
    DHR pattern: extract j7 from frame position along rail axis."""
    coords = Pose_2_TxyzRxyz(pose)
    j7_target = max(j7_min, min(coords[0], j7_max))  # X axis = rail
    optim = {{
        "AbsOn_7": 1, "AbsJnt_7": j7_target, "AbsW_7": 100,
        "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
        "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
        "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
        "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
        "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
    }}
    robot.setParam("OptimAxes", optim)
    # Nudge j7 away from 0.0 (RoboDK solver bug)
    curr = robot.Joints().list()
    if len(curr) >= 7 and curr[6] == 0.0:
        curr[6] = 0.001
        robot.setJoints(curr)

def get_pose(child_name, set_optim=True):
    """Get PoseAbs of a child frame under this cone.
    set_optim=True: set OptimAxes for MoveJ (locks j7 to frame X position).
    set_optim=False: skip OptimAxes for MoveL (use current robot config)."""
    f = find_child(cone_frame, child_name)
    assert f is not None, f"Frame '{{child_name}}' not found under '{cone_name}'"
    pose = f.PoseAbs()
    if set_optim:
        set_optim_for_pose(pose)
    return pose

print("[START] {script_name}")

# Home
robot.MoveJ({home_joints})

# Move to rail position
robot.MoveJ({home_on_rail})
''')

    if script_type == "remove_cone":
        # Cut phase
        lines.append(f'# Cut phase')
        lines.append(f'robot.setPoseTool(RDK.Item("{tools_config["cutting"]}", ITEM_TYPE_TOOL))')
        prev = None
        for child_name in seq["cut"]:
            move = _move_call(child_name, prev)
            lines.append(f'robot.{move}(get_pose("{child_name}", set_optim={"True" if move == "MoveJ" else "False"}))')
            prev = child_name

        # Grip phase
        lines.append(f'\n# Grip phase')
        lines.append(f'robot.setPoseTool(RDK.Item("{tools_config["pickup"]}", ITEM_TYPE_TOOL))')
        grip_seq = seq["grip"]
        prev = None
        for i, child_name in enumerate(grip_seq):
            move = _move_call(child_name, prev)
            lines.append(f'robot.{move}(get_pose("{child_name}", set_optim={"True" if move == "MoveJ" else "False"}))')
            if child_name == "grip" and (i + 1 < len(grip_seq)):
                lines.append(f'''
# Attach cone
from robodk.robolink import ITEM_TYPE_OBJECT
cone = RDK.Item("{cone_name}", ITEM_TYPE_OBJECT)
tool = RDK.Item("{tools_config["pickup"]}", ITEM_TYPE_TOOL)
if cone.Valid() and tool.Valid():
    cone.setParentStatic(tool)
    print("Attached: {cone_name}")
''')
            prev = child_name

    else:  # add_cone
        # Grip phase
        lines.append(f'# Grip phase')
        lines.append(f'robot.setPoseTool(RDK.Item("{tools_config["pickup"]}", ITEM_TYPE_TOOL))')
        grip_seq = seq["grip"]
        prev = None
        for i, child_name in enumerate(grip_seq):
            move = _move_call(child_name, prev)
            lines.append(f'robot.{move}(get_pose("{child_name}", set_optim={"True" if move == "MoveJ" else "False"}))')
            if child_name == "grip" and (i + 1 < len(grip_seq)):
                lines.append(f'''
# Detach cone
import json
from robodk.robolink import ITEM_TYPE_OBJECT
from robodk.robomath import TxyzRxyz_2_Pose
cone = RDK.Item("{cone_name}", ITEM_TYPE_OBJECT)
if cone.Valid():
    try:
        with open(r"{to_robodk_path(CONE_POSES_PATH)}", "r") as f:
            poses = json.load(f)
        info = poses["{cone_name}"]
        parent = RDK.Item(info["parent"], ITEM_TYPE_FRAME)
        if not parent.Valid():
            parent = RDK.Item(info["parent"])
        cone.setParentStatic(parent)
        cone.setPose(TxyzRxyz_2_Pose(info["pose"]))
        print("Detached: {cone_name}")
    except Exception as e:
        print(f"Detach failed: {{e}}")
''')
            prev = child_name

        # Suck phase
        lines.append(f'\n# Suck phase')
        lines.append(f'robot.setPoseTool(RDK.Item("{tools_config["knotting"]}", ITEM_TYPE_TOOL))')
        prev = None
        for child_name in seq["suck"]:
            move = _move_call(child_name, prev)
            lines.append(f'robot.{move}(get_pose("{child_name}", set_optim={"True" if move == "MoveJ" else "False"}))')
            prev = child_name

    # Return home
    lines.append(f'''
# Return home
robot.MoveJ({home_on_rail})
robot.MoveJ({home_joints})
print("[DONE] {script_name}")
''')

    with open(script_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    return script_path, script_name


def create_and_populate_programs(RDK, robot, cones, targets, failures, config):
    """Create Python script programs that execute movements via RoboDK API.

    DHR-style: each program is a Python script that calls robot.MoveJ/MoveL
    with frame.PoseAbs() directly, after setting OptimAxes and WorldFrame.
    """
    failed_set = {(c, ch) for c, ch in failures}

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
    machine_num = config["machine_number"]
    root_folder = get_or_create_folder(RDK, f"machine_{machine_num}_cone_programs")
    root_folder.setVisible(True)

    # Save cone poses for detach scripts
    save_cone_original_poses(RDK, config["cone_frames"])

    populated = 0

    for cone_name in viable_cones:
        cone_folder = get_or_create_folder(RDK, cone_name, parent=root_folder)

        for script_type in ("remove_cone", "add_cone"):
            prog_name = f"{script_type}_m{machine_num}_{cone_name}"
            existing = RDK.Item(prog_name, ITEM_TYPE_PROGRAM_PYTHON)
            if existing.Valid():
                print(f"  [CACHE] {prog_name}")
                continue

            script_path, script_name = _generate_movement_script(
                cone_name, config, script_type
            )
            prog = RDK.AddFile(to_robodk_path(script_path))
            if prog.Valid():
                prog.setParent(cone_folder)
                print(f"  [OK]   {prog_name}")
                populated += 1
            else:
                print(f"  [FAIL] Could not add {prog_name}")

    # Generate check_ programs (RoboDK program instructions, MoveJ only)
    # These just MoveJ to each target to verify reachability in the GUI
    for cone_name in viable_cones:
        cone_folder = get_or_create_folder(RDK, cone_name, parent=root_folder)
        check_name = f"check_m{machine_num}_{cone_name}"
        check_prog = RDK.Item(check_name, ITEM_TYPE_PROGRAM)
        if check_prog.Valid() and check_prog.InstructionCount() > 0:
            print(f"  [CACHE] {check_name}")
        else:
            if not check_prog.Valid():
                check_prog = RDK.AddProgram(check_name, robot)
                check_prog.setParent(cone_folder)

            # Set world frame
            world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
            if world_frame.Valid():
                check_prog.setPoseFrame(world_frame)

            # MoveJ to each target in order: cut, grip, suck
            for phase_name, tool_name in [
                ("cut", config["tools"]["cutting"]),
                ("grip", config["tools"]["pickup"]),
                ("suck", config["tools"]["knotting"]),
            ]:
                tool = find_tool(RDK, tool_name)
                check_prog.setPoseTool(tool)
                for child_name in config["child_frames"][phase_name]:
                    tgt = _get_target(targets, cone_name, child_name)
                    if tgt is not None:
                        check_prog.MoveJ(tgt)

            print(f"  [OK]   {check_name}")
            populated += 1

    # Generate run_all as a RoboDK program with CallProgram instructions
    run_all_name = f"run_all_m{machine_num}"
    existing = RDK.Item(run_all_name, ITEM_TYPE_PROGRAM)
    if existing.Valid() and existing.InstructionCount() > 0:
        print(f"  [CACHE] {run_all_name}")
    else:
        if not existing.Valid():
            run_all = RDK.AddProgram(run_all_name, robot)
        else:
            run_all = existing
        for cone_name in viable_cones:
            for script_type in ("remove_cone", "add_cone"):
                sname = f"{script_type}_m{machine_num}_{cone_name}"
                run_all.RunInstruction(sname, INSTRUCTION_CALL_PROGRAM)
        print(f"  [OK]   {run_all_name} ({len(viable_cones)} cones)")
        populated += 1

    print(f"[programs] {populated} script(s) created")


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
