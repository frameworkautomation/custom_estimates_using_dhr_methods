"""
Place bright red cones at all cone_ frames in the station.

Searches under Cone_Bin_Frame (bin cones) and top_plate_frame (machine cones)
for frames starting with 'cone_' and places a cone mesh in each.

Usage:
    python robert_checker_stuff/place_all_cones.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/place_all_cones.py --robodk-ip 172.23.208.1 --remove

AI-generated code (Claude Opus 4.6) — human-reviewed before use.
"""

import sys
import os
import argparse

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from place_cones import place_cones, connect, wsl_to_win

REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DEFAULT_CONE_STEP = os.path.join(REPO_ROOT, "my_assets", "sams_simple_cone.stp")
DEFAULT_COLOR = "#FF0000"

# Parents to search for cone_ frames
CONE_PARENTS = ["Cone_Bin_Frame", "top_plate_frame"]


def collect_cone_frames(parent):
    """Recursively find all cone_ frames under a parent."""
    results = []
    if parent.Name().startswith("cone_") and parent.Type() == ITEM_TYPE_FRAME:
        results.append(parent)
    for child in parent.Childs():
        results.extend(collect_cone_frames(child))
    return results


def main():
    parser = argparse.ArgumentParser(description="Place red cones at all cone_ frames")
    parser.add_argument("--robodk-ip", default=None)
    parser.add_argument("--step-file", default=DEFAULT_CONE_STEP)
    parser.add_argument("--color", default=DEFAULT_COLOR, help="Hex color (default: #FF0000)")
    parser.add_argument("--remove", action="store_true", help="Remove placed cones instead")
    args = parser.parse_args()

    RDK = connect(args.robodk_ip)

    # Collect cone_ frames from known parents
    cone_frames = []
    for parent_name in CONE_PARENTS:
        p = RDK.Item(parent_name)
        if not p.Valid():
            print(f"[WARN] {parent_name} not found, skipping")
            continue
        found = collect_cone_frames(p)
        cone_frames.extend(found)
        if found:
            print(f"[INFO] {len(found)} cone frames under {parent_name}")

    print(f"\nFound {len(cone_frames)} cone frames total")

    if args.remove:
        removed = 0
        for frame in cone_frames:
            for child in frame.Childs():
                if child.Type() == ITEM_TYPE_OBJECT:
                    print(f"[REMOVE] {child.Name()} from {frame.Name()}")
                    child.Delete()
                    removed += 1
        print(f"[DONE] Removed {removed} cone(s)")
        return

    if not cone_frames:
        print("[WARN] No cone_ frames found")
        return

    cones = place_cones(RDK, cone_frames, args.step_file, colors=[args.color])
    print(f"[DONE] Placed {len(cones)} cones")


if __name__ == "__main__":
    main()
