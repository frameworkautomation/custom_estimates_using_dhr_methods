import json
from robodk.robolink import Robolink, ITEM_TYPE_OBJECT, ITEM_TYPE_FRAME
from robodk.robomath import TxyzRxyz_2_Pose
RDK = Robolink()
cone = RDK.Item("cone_back_furthest", ITEM_TYPE_OBJECT)
if not cone.Valid():
    print("Cone not found: cone_back_furthest")
else:
    with open(r"C:/Users/samst/Framework/clones/custom_estimates_2/robert_checker_stuff/machine_cone_original_poses.json", "r") as f:
        poses = json.load(f)
    info = poses["cone_back_furthest"]
    parent = RDK.Item(info["parent"], ITEM_TYPE_FRAME)
    if not parent.Valid():
        parent = RDK.Item(info["parent"])
    cone.setParentStatic(parent)
    cone.setPose(TxyzRxyz_2_Pose(info["pose"]))
    print("Detached: cone_back_furthest")
