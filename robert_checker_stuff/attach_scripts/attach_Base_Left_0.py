from robodk.robolink import Robolink, ITEM_TYPE_TOOL, ITEM_TYPE_OBJECT
RDK = Robolink()
tool = RDK.Item("pickup", ITEM_TYPE_TOOL)
cone = RDK.Item("Base_Left_0", ITEM_TYPE_OBJECT)
if tool.Valid() and cone.Valid():
    cone.setParentStatic(tool)
    print("Attached: Base_Left_0")
else:
    print("Failed to attach Base_Left_0")
