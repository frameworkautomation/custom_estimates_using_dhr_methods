"""
Build DHR-style movement scripts for machine cones using optimization frames.

Expects:
1. setup_machine_cone_programs.py to have been run (check_ targets exist)
2. Optimization frames placed in station under a folder:
   Machine{N}Base / optim_frames /
     closest_m{N}   — frame at j7 position for closest cones
     furthest_m{N}  — frame at j7 position for furthest cones

Each cone maps to an optimization frame. The frame's X position determines
where j7 locks during MoveL sequences. MoveJ handles rail transitions between
different optimization frame positions.

Usage:
    python robert_checker_stuff/build_machine_cone_movements.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/build_machine_cone_movements.py --robodk-ip 172.23.208.1 \
        --config robert_checker_stuff/setup_machine_cone_programs_config_alternate_side.json
"""

import sys
import os
import json
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_PROGRAM, ITEM_TYPE_PROGRAM_PYTHON,
    ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME, ITEM_TYPE_FOLDER,
    INSTRUCTION_CALL_PROGRAM,
)
from robodk.robomath import Pose_2_TxyzRxyz

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(SCRIPT_DIR, "setup_machine_cone_programs_config.json")
CONE_POSES_PATH = os.path.join(SCRIPT_DIR, "machine_cone_original_poses.json")
MOVEMENT_SCRIPTS_DIR = os.path.join(SCRIPT_DIR, "machine_cone_movement_scripts")

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]


# ── CONNECT ───────────────────────────────────────────────────────────────────

def connect(ip=None):
    if ip:
        return Robolink(robodk_ip=ip)
    try:
        rdk = Robolink()
        rdk.Item("")
        return rdk
    except Exception:
        return Robolink(robodk_ip="172.23.208.1")


def find_robot(RDK):
    for name in ROBOT_NAMES:
        r = RDK.Item(name, ITEM_TYPE_ROBOT)
        if r.Valid():
            return r
    raise RuntimeError(f"Robot not found. Tried: {ROBOT_NAMES}")


def find_tool(RDK, name):
    tool = RDK.Item(name, ITEM_TYPE_TOOL)
    assert tool.Valid(), f"Tool '{name}' not found"
    return tool


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


# ── FRAME SEARCH ──────────────────────────────────────────────────────────────

def _find_frame_recursive(parent, name):
    try:
        for child in parent.Childs():
            try:
                if child.Name() == name and child.Type() == 3:
                    return child
                found = _find_frame_recursive(child, name)
                if found is not None:
                    return found
            except:
                continue
    except:
        pass
    return None


# ── OPTIMIZATION FRAMES ──────────────────────────────────────────────────────

def find_optim_frames(RDK, config):
    """Find optimization frames under Machine{N}Base/optim_frames/.

    Expects: closest_m{N} and furthest_m{N} frames.
    Returns dict: frame_name -> (frame_item, j7_value)
    """
    machine_num = config["machine_number"]
    machine_base = RDK.Item(f"Machine{machine_num}Base", ITEM_TYPE_FRAME)
    assert machine_base.Valid(), f"Machine{machine_num}Base not found"

    optim_folder = _find_frame_recursive(machine_base, "optim_frames")
    assert optim_folder is not None, \
        f"'optim_frames' folder not found under Machine{machine_num}Base"

    closest_name = f"closest_m{machine_num}"
    furthest_name = f"furthest_m{machine_num}"

    closest = _find_frame_recursive(optim_folder, closest_name)
    furthest = _find_frame_recursive(optim_folder, furthest_name)

    assert closest is not None, f"'{closest_name}' not found under optim_frames"
    assert furthest is not None, f"'{furthest_name}' not found under optim_frames"

    closest_j7 = Pose_2_TxyzRxyz(closest.PoseAbs())[0]
    furthest_j7 = Pose_2_TxyzRxyz(furthest.PoseAbs())[0]

    print(f"[OPTIM] {closest_name}: j7={closest_j7:.0f}")
    print(f"[OPTIM] {furthest_name}: j7={furthest_j7:.0f}")

    return {
        "closest": (closest, closest_j7),
        "furthest": (furthest, furthest_j7),
    }


def cone_to_optim(cone_name):
    """Map cone name to optimization frame key."""
    if "closest" in cone_name:
        return "closest"
    elif "furthest" in cone_name:
        return "furthest"
    # Default: closest for front, furthest for back? Configurable later
    if "front" in cone_name:
        return "closest"
    return "furthest"


# ── MOVEMENT SCRIPT GENERATION ────────────────────────────────────────────────

def _is_approach(child_name):
    return "approach" in child_name.lower()


def _move_call(child_name, prev_child):
    """MoveJ to first approach, MoveL for everything else within a phase."""
    if _is_approach(child_name) and (prev_child is None or _is_approach(prev_child)):
        return "MoveJ"
    return "MoveL"


def generate_movement_script(cone_name, config, script_type, optim_j7):
    """Generate a DHR-style Python movement script.

    optim_j7: the j7 value from the optimization frame for this cone.
    All MoveL within a phase lock j7 to this value via OptimAxes.
    MoveJ transitions set OptimAxes before moving.
    """
    os.makedirs(MOVEMENT_SCRIPTS_DIR, exist_ok=True)
    machine_num = config["machine_number"]
    script_name = f"{script_type}_m{machine_num}_{cone_name}"
    script_path = os.path.join(MOVEMENT_SCRIPTS_DIR, f"{script_name}.py")

    tools_config = config["tools"]
    home_joints = config["home_joints"]
    home_on_rail = list(home_joints)
    home_on_rail[6] = config["j7_value"]

    if script_type == "remove_cone":
        seq = config["remove_cone_sequence"]
    else:
        seq = config["add_cone_sequence"]

    lines = []
    lines.append(f'''import sys
sys.path.append("C:/RoboDK/Python")
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME
from robodk.robomath import eye, Pose_2_TxyzRxyz

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

def find_child(parent, name):
    try:
        for child in parent.Childs():
            try:
                if child.Name() == name and child.Type() == 3:
                    return child
                found = find_child(child, name)
                if found is not None:
                    return found
            except:
                continue
    except:
        pass
    return None

# Find the cone frame
cone_frame = find_child(RDK.Item("Machine{machine_num}Base", ITEM_TYPE_FRAME), "{cone_name}")
assert cone_frame is not None, "Cone frame '{cone_name}' not found"

# Optimization frame j7 value for this cone
OPTIM_J7 = {optim_j7:.1f}

def set_optim(j7_val):
    """Set OptimAxes with j7 hard-locked to the given value."""
    optim = {{
        "AbsOn_7": 1, "AbsJnt_7": j7_val, "AbsW_7": 100,
        "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
        "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
        "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
        "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
        "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
    }}
    robot.setParam("OptimAxes", optim)
    curr = robot.Joints().list()
    if len(curr) >= 7 and curr[6] == 0.0:
        curr[6] = 0.001
        robot.setJoints(curr)

def get_pose(child_name):
    """Get PoseAbs of a child frame under this cone."""
    f = find_child(cone_frame, child_name)
    assert f is not None, f"Frame '{{child_name}}' not found under '{cone_name}'"
    return f.PoseAbs()

print("[START] {script_name}")

# Home
robot.MoveJ({home_joints})

# Move to rail position near optimization frame
robot.MoveJ({home_on_rail})

# Lock j7 to optimization frame position
set_optim(OPTIM_J7)
''')

    if script_type == "remove_cone":
        # Cut phase
        lines.append(f'# Cut phase')
        lines.append(f'robot.setPoseTool(RDK.Item("{tools_config["cutting"]}", ITEM_TYPE_TOOL))')
        lines.append(f'set_optim(OPTIM_J7)')
        prev = None
        for child_name in seq["cut"]:
            move = _move_call(child_name, prev)
            lines.append(f'robot.{move}(get_pose("{child_name}"))')
            prev = child_name

        # Grip phase
        lines.append(f'\n# Grip phase')
        lines.append(f'robot.setPoseTool(RDK.Item("{tools_config["pickup"]}", ITEM_TYPE_TOOL))')
        lines.append(f'set_optim(OPTIM_J7)')
        grip_seq = seq["grip"]
        prev = None
        for i, child_name in enumerate(grip_seq):
            move = _move_call(child_name, prev)
            lines.append(f'robot.{move}(get_pose("{child_name}"))')
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
        lines.append(f'set_optim(OPTIM_J7)')
        grip_seq = seq["grip"]
        prev = None
        for i, child_name in enumerate(grip_seq):
            move = _move_call(child_name, prev)
            lines.append(f'robot.{move}(get_pose("{child_name}"))')
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
        lines.append(f'set_optim(OPTIM_J7)')
        prev = None
        for child_name in seq["suck"]:
            move = _move_call(child_name, prev)
            lines.append(f'robot.{move}(get_pose("{child_name}"))')
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


# ── MAIN ──────────────────────────────────────────────────────────────────────

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
        return folder

    existing = RDK.Item(name, ITEM_TYPE_FOLDER)
    if existing.Valid():
        return existing
    RDK.Command("AddFolder", name)
    folder = RDK.Item(name, ITEM_TYPE_FOLDER)
    assert folder.Valid(), f"Failed to create folder '{name}'"
    return folder


def main():
    ap = argparse.ArgumentParser(
        description="Build movement scripts using optimization frames"
    )
    ap.add_argument("--robodk-ip", default=None)
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    args = ap.parse_args()

    assert os.path.exists(args.config), f"Config not found: {args.config}"
    with open(args.config, "r", encoding="utf-8") as f:
        config = json.load(f)

    machine_num = config["machine_number"]
    print(f"[CONFIG] Machine {machine_num}, j7={config['j7_value']}")

    RDK = connect(args.robodk_ip)
    robot = find_robot(RDK)
    print(f"[INFO] Robot: {robot.Name()}")

    # Find optimization frames
    print("\n── Find optimization frames ──")
    optim_frames = find_optim_frames(RDK, config)

    # Generate movement scripts
    print("\n── Generate movement scripts ──")
    root_folder = get_or_create_folder(RDK, f"machine_{machine_num}_cone_programs")
    root_folder.setVisible(True)

    populated = 0
    for cone_name in config["cone_frames"]:
        optim_key = cone_to_optim(cone_name)
        _, optim_j7 = optim_frames[optim_key]

        cone_folder = get_or_create_folder(RDK, cone_name, parent=root_folder)

        for script_type in ("remove_cone", "add_cone"):
            prog_name = f"{script_type}_m{machine_num}_{cone_name}"
            existing = RDK.Item(prog_name, ITEM_TYPE_PROGRAM_PYTHON)
            if existing.Valid():
                print(f"  [CACHE] {prog_name}")
                continue

            script_path, script_name = generate_movement_script(
                cone_name, config, script_type, optim_j7
            )
            prog = RDK.AddFile(to_robodk_path(script_path))
            if prog.Valid():
                prog.setParent(cone_folder)
                print(f"  [OK]   {prog_name} (optim={optim_key}, j7={optim_j7:.0f})")
                populated += 1

    # run_all program
    run_all_name = f"run_all_m{machine_num}"
    existing = RDK.Item(run_all_name, ITEM_TYPE_PROGRAM)
    if existing.Valid() and existing.InstructionCount() > 0:
        print(f"  [CACHE] {run_all_name}")
    else:
        if not existing.Valid():
            run_all = RDK.AddProgram(run_all_name, robot)
        else:
            run_all = existing
        for cone_name in config["cone_frames"]:
            for st in ("remove_cone", "add_cone"):
                run_all.RunInstruction(
                    f"{st}_m{machine_num}_{cone_name}", INSTRUCTION_CALL_PROGRAM
                )
        print(f"  [OK]   {run_all_name}")
        populated += 1

    print(f"\n[DONE] {populated} scripts created")


if __name__ == "__main__":
    main()
