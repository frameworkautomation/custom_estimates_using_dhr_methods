from robodk.robolink import Robolink, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT
RDK = Robolink()
tool = RDK.Item("pickup", ITEM_TYPE_TOOL)
cone_frame = RDK.Item("cone_in_bin_01_frame", ITEM_TYPE_FRAME)
if tool.Valid() and cone_frame.Valid():
    for child in cone_frame.Childs():
        if child.Type() == ITEM_TYPE_OBJECT:
            child.setParentStatic(tool)
            print("Attached cone from cone_in_bin_01_frame")
            break
    else:
        print("No object child under cone_in_bin_01_frame")
else:
    print("Failed: tool or frame not found")
