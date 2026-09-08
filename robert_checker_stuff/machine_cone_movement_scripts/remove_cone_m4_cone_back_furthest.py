import sys
sys.path.append("C:/RoboDK/Python")
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME
from robodk.robomath import eye, Pose_2_TxyzRxyz

RDK = Robolink()

# Find robot
robot = RDK.Item("Fanuc R2000iC 125L", ITEM_TYPE_ROBOT)
if not robot.Valid():
    robot = RDK.Item("Fanuc R-2000iC/125L", ITEM_TYPE_ROBOT)

# Set pose frame to robot's parent (DHR method)
# robot.Parent() = robot base frame, moves with the rail
robot.setPoseFrame(robot.Parent())

# Rail base for PoseWrt calculations
rail_base = robot.Parent().Parent().Parent()

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
cone_frame = find_child(RDK.Item("Machine4Base", ITEM_TYPE_FRAME), "cone_back_furthest")
assert cone_frame is not None, "Cone frame 'cone_back_furthest' not found"

# Optimization frame j7 value for this cone
OPTIM_J7 = 3450.0

def set_optim(j7_val):
    """Set OptimAxes with j7 hard-locked to the given value."""
    optim = {
        "AbsOn_7": 1, "AbsJnt_7": j7_val, "AbsW_7": 100,
        "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
        "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
        "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
        "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
        "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
    }
    robot.setParam("OptimAxes", optim)
    curr = robot.Joints().list()
    if len(curr) >= 7 and curr[6] == 0.0:
        curr[6] = 0.001
        robot.setJoints(curr)

def get_pose(child_name):
    """Get pose of a child frame relative to rail base (DHR method)."""
    f = find_child(cone_frame, child_name)
    assert f is not None, f"Frame '{child_name}' not found under 'cone_back_furthest'"
    return f.PoseWrt(rail_base)

print("[START] remove_cone_m4_cone_back_furthest")

# Home
robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# Move to rail position near optimization frame
robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3450])

# Lock j7 to optimization frame position
set_optim(OPTIM_J7)

# Cut phase
robot.setPoseTool(RDK.Item("cutting", ITEM_TYPE_TOOL))
set_optim(OPTIM_J7)
robot.MoveJ(get_pose("Cut_approach"))
robot.MoveL(get_pose("Cut_Catch"))
robot.MoveL(get_pose("Cut"))
robot.MoveL(get_pose("Cut_approach"))

# Grip phase
robot.setPoseTool(RDK.Item("pickup", ITEM_TYPE_TOOL))
set_optim(OPTIM_J7)
robot.MoveJ(get_pose("grip_approach"))
robot.MoveL(get_pose("grip"))

# Attach cone
from robodk.robolink import ITEM_TYPE_OBJECT
cone = RDK.Item("cone_back_furthest", ITEM_TYPE_OBJECT)
tool = RDK.Item("pickup", ITEM_TYPE_TOOL)
if cone.Valid() and tool.Valid():
    cone.setParentStatic(tool)
    print("Attached: cone_back_furthest")

robot.MoveL(get_pose("grip_approach"))

# Return home
robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3450])
robot.MoveJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
print("[DONE] remove_cone_m4_cone_back_furthest")
