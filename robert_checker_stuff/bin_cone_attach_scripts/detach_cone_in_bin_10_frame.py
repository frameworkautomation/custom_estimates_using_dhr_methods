import json
from robodk.robolink import Robolink, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT
from robodk.robomath import TxyzRxyz_2_Pose
RDK = Robolink()
try:
    with open(r"C:/Users/samst/Framework/clones/custom_estimates_2/robert_checker_stuff/bin_cone_original_poses.json", "r") as f:
        poses = json.load(f)
    info = poses["cone_in_bin_10_frame"]
    parent = RDK.Item(info["parent_frame"], ITEM_TYPE_FRAME)
    tool = RDK.Item("pickup", ITEM_TYPE_TOOL)
    # Find the object currently under the tool
    if tool.Valid():
        for child in tool.Childs():
            if child.Type() == ITEM_TYPE_OBJECT and child.Name() == info["object_name"]:
                child.setParentStatic(parent)
                child.setPose(TxyzRxyz_2_Pose(info["pose"]))
                print("Detached cone back to cone_in_bin_10_frame")
                break
        else:
            print("No matching object found under pickup tool")
    else:
        print("Pickup tool not found")
except Exception as e:
    print(f"Detach failed: {e}")
