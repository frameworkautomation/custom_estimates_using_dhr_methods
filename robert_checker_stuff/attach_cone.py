"""
Attach the cone closest to the robot's current TCP position.

Called as a sub-program during RoboDK program playback. Finds which grab
target (in targets_to_use/extracted) the TCP is nearest to, extracts the
cone name from that target, and parents the cone to the pickup tool.
"""

import re
from robodk.robolink import Robolink, ITEM_TYPE_TOOL, ITEM_TYPE_TARGET, ITEM_TYPE_OBJECT, ITEM_TYPE_FOLDER
from robodk.robomath import Pose_2_TxyzRxyz
import math

RDK = Robolink()

GRAB_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+_grab$")

tool = RDK.Item("pickup", ITEM_TYPE_TOOL)
if not tool.Valid():
    print("pickup tool not found")
    quit()

tcp_xyz = Pose_2_TxyzRxyz(tool.PoseAbs())[:3]

# Only search grab targets inside targets_to_use/extracted
ttu_root = RDK.Item("targets_to_use", ITEM_TYPE_FOLDER)
if not ttu_root.Valid():
    print("targets_to_use folder not found")
    quit()

ttu_extracted = None
for child in ttu_root.Childs():
    if child.Name() == "extracted" and child.Type() == ITEM_TYPE_FOLDER:
        ttu_extracted = child
        break

if ttu_extracted is None:
    print("targets_to_use/extracted folder not found")
    quit()

best_dist = float("inf")
best_cone_name = None

for t in ttu_extracted.Childs():
    if t.Type() != ITEM_TYPE_TARGET:
        continue
    name = t.Name()
    if not GRAB_PATTERN.match(name):
        continue
    pos = Pose_2_TxyzRxyz(t.PoseAbs())[:3]
    dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(tcp_xyz, pos)))
    if dist < best_dist:
        best_dist = dist
        best_cone_name = name.rsplit("_grab", 1)[0]

if best_cone_name is None:
    print("No grab targets found in targets_to_use/extracted")
    quit()

print(f"Closest grab target: {best_cone_name}_grab ({best_dist:.1f}mm)")

cone = RDK.Item(best_cone_name, ITEM_TYPE_OBJECT)
if not cone.Valid():
    print(f"Cone object '{best_cone_name}' not found")
    quit()

cone.setParentStatic(tool)
print(f"Attached: {best_cone_name}")
