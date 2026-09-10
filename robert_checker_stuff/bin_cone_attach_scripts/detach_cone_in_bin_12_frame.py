import json
from robodk.robolink import Robolink, ITEM_TYPE_OBJECT, ITEM_TYPE_FRAME
from robodk.robomath import TxyzRxyz_2_Pose
RDK = Robolink()
cone = RDK.Item("sams_simple_cone", ITEM_TYPE_OBJECT)
if cone.Valid():
    try:
        with open(r"C:/Users/samst/Framework/clones/custom_estimates_2/robert_checker_stuff/bin_cone_original_poses.json", "r") as f:
            poses = json.load(f)
        info = poses["sams_simple_cone"]
        parent = RDK.Item(info["parent"], ITEM_TYPE_FRAME)
        if not parent.Valid():
            parent = RDK.Item(info["parent"])
        cone.setParentStatic(parent)
        cone.setPose(TxyzRxyz_2_Pose(info["pose"]))
        print("Detached: sams_simple_cone")
    except Exception as e:
        print(f"Detach failed: {e}")
else:
    print("Cone not found: sams_simple_cone")
