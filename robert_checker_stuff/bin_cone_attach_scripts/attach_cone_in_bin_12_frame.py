from robodk.robolink import Robolink, ITEM_TYPE_TOOL, ITEM_TYPE_OBJECT
RDK = Robolink()
tool = RDK.Item("pickup", ITEM_TYPE_TOOL)
cone = RDK.Item("sams_simple_cone", ITEM_TYPE_OBJECT)
if tool.Valid() and cone.Valid():
    cone.setParentStatic(tool)
    print("Attached: sams_simple_cone")
else:
    print("Failed to attach sams_simple_cone")
