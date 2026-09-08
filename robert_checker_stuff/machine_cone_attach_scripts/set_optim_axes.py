from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
RDK = Robolink()
robot = RDK.Item("Fanuc R2000iC 125L", ITEM_TYPE_ROBOT)
if not robot.Valid():
    robot = RDK.Item("Fanuc R-2000iC/125L", ITEM_TYPE_ROBOT)
props = {
    "AbsOn_7": 1, "AbsJnt_7": 1700, "AbsW_7": 20,
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
    "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
    "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
}
robot.setParam("OptimAxes", props)
print("OptimAxes set: j7 soft constraint at 1700")
