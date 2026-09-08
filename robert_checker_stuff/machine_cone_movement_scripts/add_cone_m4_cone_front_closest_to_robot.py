import sys
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
cone_frame = find_child(RDK.Item("Machine4Base", ITEM_TYPE_FRAME), "cone_front_closest_to_robot")
assert cone_frame is not None, "Cone frame 'cone_front_closest_to_robot' not found"

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
    optim = {
        "AbsOn_7": 1, "AbsJnt_7": j7_target, "AbsW_7": 100,
        "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
        "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
        "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
        "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
        "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
    }
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
    assert f is not None, f"Frame '{child_name}' not found under 'cone_front_closest_to_robot'"
    pose = f.PoseAbs()
    if set_optim:
        set_optim_for_pose(pose)
    return pose

print("[START] add_cone_m4_cone_front_closest_to_robot")

# Home
robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Move to rail position
robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3450])

# Grip phase
robot.setPoseTool(RDK.Item("pickup", ITEM_TYPE_TOOL))
robot.MoveJ(get_pose("grip_approach", set_optim=True))
robot.MoveL(get_pose("grip", set_optim=False))

# Detach cone
import json
from robodk.robolink import ITEM_TYPE_OBJECT
from robodk.robomath import TxyzRxyz_2_Pose
cone = RDK.Item("cone_front_closest_to_robot", ITEM_TYPE_OBJECT)
if cone.Valid():
    try:
        with open(r"C:/Users/samst/Framework/clones/custom_estimates_2/robert_checker_stuff/machine_cone_original_poses.json", "r") as f:
            poses = json.load(f)
        info = poses["cone_front_closest_to_robot"]
        parent = RDK.Item(info["parent"], ITEM_TYPE_FRAME)
        if not parent.Valid():
            parent = RDK.Item(info["parent"])
        cone.setParentStatic(parent)
        cone.setPose(TxyzRxyz_2_Pose(info["pose"]))
        print("Detached: cone_front_closest_to_robot")
    except Exception as e:
        print(f"Detach failed: {e}")

robot.MoveL(get_pose("grip_approach", set_optim=False))

# Suck phase
robot.setPoseTool(RDK.Item("knotting", ITEM_TYPE_TOOL))
robot.MoveJ(get_pose("suck_approach", set_optim=True))
robot.MoveL(get_pose("suck", set_optim=False))
robot.MoveL(get_pose("suck_approach", set_optim=False))

# Return home
robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3450])
robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
print("[DONE] add_cone_m4_cone_front_closest_to_robot")
