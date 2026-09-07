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

# OptimAxes — soft j7 constraint
optim = {
    "AbsOn_7": 1, "AbsJnt_7": 1700, "AbsW_7": 20,
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
    "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
    "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
}
robot.setParam("OptimAxes", optim)

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
cone_frame = find_child(RDK.Item("Machine3Base", ITEM_TYPE_FRAME), "cone_front_furthest")
assert cone_frame is not None, "Cone frame 'cone_front_furthest' not found"

def get_pose(child_name):
    """Get PoseAbs of a child frame under this cone."""
    f = find_child(cone_frame, child_name)
    assert f is not None, f"Frame '{child_name}' not found under 'cone_front_furthest'"
    return f.PoseAbs()

print("[START] add_cone_cone_front_furthest")

# Home
robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Move to rail position
robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1700])

# Grip phase
robot.setPoseTool(RDK.Item("pickup", ITEM_TYPE_TOOL))
robot.MoveJ(get_pose("grip_approach"))
robot.MoveL(get_pose("grip"))

# Detach cone
import json
from robodk.robolink import ITEM_TYPE_OBJECT
from robodk.robomath import TxyzRxyz_2_Pose
cone = RDK.Item("cone_front_furthest", ITEM_TYPE_OBJECT)
if cone.Valid():
    try:
        with open(r"C:/Users/samst/Framework/clones/custom_estimates_2/robert_checker_stuff/machine_cone_original_poses.json", "r") as f:
            poses = json.load(f)
        info = poses["cone_front_furthest"]
        parent = RDK.Item(info["parent"], ITEM_TYPE_FRAME)
        if not parent.Valid():
            parent = RDK.Item(info["parent"])
        cone.setParentStatic(parent)
        cone.setPose(TxyzRxyz_2_Pose(info["pose"]))
        print("Detached: cone_front_furthest")
    except Exception as e:
        print(f"Detach failed: {e}")

robot.MoveL(get_pose("grip_approach"))

# Suck phase
robot.setPoseTool(RDK.Item("knotting", ITEM_TYPE_TOOL))
robot.MoveJ(get_pose("suck_approach"))
robot.MoveL(get_pose("suck"))
robot.MoveL(get_pose("suck_approach"))

# Return home
robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1700])
robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
print("[DONE] add_cone_cone_front_furthest")
