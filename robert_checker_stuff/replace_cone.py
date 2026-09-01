"""
Detach any cone from the end effector and restore it to its original position.

Reads original cone poses from cone_original_poses.json (saved by Phase 6
of setup_base_movements.py before attaching). Finds cones attached to tools,
reparents them back to their cone_frame, and restores their original pose.

Can be run standalone or from inside RoboDK (added to station by Phase 4).

Usage (standalone):
    python robert_checker_stuff/replace_cone.py
    python robert_checker_stuff/replace_cone.py --robodk-ip 172.23.208.1

Usage (inside RoboDK):
    Run from the station tree — double-click "replace_cone" program.
"""

import sys
import os
import re
import json

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_OBJECT, ITEM_TYPE_FRAME
from robodk.robomath import TxyzRxyz_2_Pose

# Hardcoded path — __file__ is unreliable when RoboDK copies scripts to temp
POSES_PATH = r"C:\Users\samst\Framework\clones\custom_estimates_using_dhr_methods\robert_checker_stuff\cone_original_poses.json"
CONE_PATTERN = re.compile(r"^(alt_)?Base_(Right|Left)_\d+$")


def run(RDK=None):
    if RDK is None:
        RDK = Robolink()

    assert os.path.exists(POSES_PATH), \
        f"Original poses file not found: {POSES_PATH}\n" \
        f"Run setup_base_movements.py first to save cone poses."

    with open(POSES_PATH, "r", encoding="utf-8") as f:
        original_poses = json.load(f)

    all_objects = RDK.ItemList(ITEM_TYPE_OBJECT)
    cones = [obj for obj in all_objects if CONE_PATTERN.match(obj.Name())]

    restored = 0
    already_ok = 0

    for cone in cones:
        name = cone.Name()
        parent = cone.Parent()

        # Check if cone is already in its _frame
        expected_parent = f"{name}_frame"
        if parent.Valid() and parent.Name() == expected_parent:
            already_ok += 1
            continue

        if name not in original_poses:
            print(f"  [WARN] No saved pose for '{name}' — skipping")
            continue

        saved = original_poses[name]
        if isinstance(saved, dict):
            pose_data = saved["pose"]
            parent_name = saved.get("parent", expected_parent)
        else:
            pose_data = saved
            parent_name = expected_parent

        original_pose = TxyzRxyz_2_Pose(pose_data)

        # Find the parent frame to restore to
        target_parent = RDK.Item(parent_name, ITEM_TYPE_FRAME)
        if not target_parent.Valid():
            print(f"  [WARN] Parent '{parent_name}' not found for '{name}' — skipping")
            continue

        cone.setParent(target_parent)
        cone.setPose(original_pose)

        restored += 1
        print(f"  [RESTORE] {name} → {parent_name}")

    print(f"\n[DONE] {restored} cone(s) restored, {already_ok} already in place")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Detach cones from tool and restore to original position")
    ap.add_argument("--robodk-ip", default=None)
    args = ap.parse_args()

    if args.robodk_ip:
        RDK = Robolink(robodk_ip=args.robodk_ip)
    else:
        RDK = Robolink()

    run(RDK)
else:
    # Running inside RoboDK
    run()
