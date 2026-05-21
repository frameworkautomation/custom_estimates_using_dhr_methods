"""
scrape_robodk_frames.py

Connects to a running RoboDK instance and extracts frames by pattern.
Saves full 6D poses (position + orientation as Euler angles AND rotation
matrix columns) to JSON so Grasshopper / Rhino can display them as planes.

Two default groups are scraped:

  --rail-pattern   (default: OptimizationApproach)
      One frame per machine / cart / rack zone along the linear rail.
      DHR names these: OptimizationApproachMachine1, OptimizationApproachCart2, etc.
      Each frame's X position in world space = where the rail needs to be
      to service that zone.

  --approach-pattern   (default: CurtainSafe)
      Curtain-safe approach frames placed in front of each machine door.
      DHR names these: ApproachMachine1CurtainSafe, etc.

Output: robo_dk_output/scraped_frames.json
        Two top-level keys: "rail_points" and "approach_frames".
        Each entry has: name, x, y, z, rx, ry, rz, xaxis, yaxis, zaxis.

Usage:
    python misc_extraction_utils/scrape_robodk_frames.py
    python misc_extraction_utils/scrape_robodk_frames.py --rail-pattern "OptimizationApproach" --approach-pattern "CurtainSafe"
    python misc_extraction_utils/scrape_robodk_frames.py --all-frames
"""

import sys
import os
import json
import math
import argparse
import re

sys.path.append("C:/RoboDK/Python")

OUTPUT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output", "scraped_frames.json"
)


def parse_args():
    p = argparse.ArgumentParser(description="Scrape named frames from RoboDK station.")
    p.add_argument(
        "--rail-pattern",
        default="OptimizationApproach",
        help="Case-insensitive regex for rail-position frames (default: 'OptimizationApproach')",
    )
    p.add_argument(
        "--approach-pattern",
        default="CurtainSafe",
        help="Case-insensitive regex for machine approach frames (default: 'CurtainSafe')",
    )
    p.add_argument(
        "--all-frames",
        action="store_true",
        help="Dump every frame in the station into a single 'all_frames' key (ignores other patterns)",
    )
    return p.parse_args()


def connect():
    from robodk.robolink import Robolink, ITEM_TYPE_FRAME
    rdk = Robolink()
    try:
        rdk.Item("")
    except Exception as e:
        print(f"[ERROR] Cannot connect to RoboDK: {e}")
        sys.exit(1)
    print("[INFO] Connected to RoboDK")
    return rdk, ITEM_TYPE_FRAME


def frame_to_dict(frame):
    """Extract name, 6D pose (Euler ZYX), and rotation column vectors from a frame."""
    from robodk.robomath import Pose_2_TxyzRxyz

    name = frame.Name()
    pose = frame.PoseAbs()
    vals = Pose_2_TxyzRxyz(pose)
    m = pose.rows

    return {
        "name": name,
        # Position (mm)
        "x":  round(vals[0], 4),
        "y":  round(vals[1], 4),
        "z":  round(vals[2], 4),
        # Euler ZYX angles (degrees) — for reference
        "rx": round(math.degrees(vals[3]), 6),
        "ry": round(math.degrees(vals[4]), 6),
        "rz": round(math.degrees(vals[5]), 6),
        # Rotation matrix columns — use these to build Rhino Planes directly
        "xaxis": [round(m[0][0], 6), round(m[1][0], 6), round(m[2][0], 6)],
        "yaxis": [round(m[0][1], 6), round(m[1][1], 6), round(m[2][1], 6)],
        "zaxis": [round(m[0][2], 6), round(m[1][2], 6), round(m[2][2], 6)],
    }


def scrape(rdk, ITEM_TYPE_FRAME, pattern):
    """Return list of frame dicts whose names match the regex pattern."""
    regex = re.compile(pattern, re.IGNORECASE)
    results = []
    for frame in rdk.ItemList(ITEM_TYPE_FRAME):
        if regex.search(frame.Name()):
            results.append(frame_to_dict(frame))
    return results


def print_table(entries, label):
    if not entries:
        print(f"  (none)")
        return
    col_w = max(len(e["name"]) for e in entries) + 2
    print(f"  {'Name':<{col_w}} {'x':>10} {'y':>10} {'z':>10}")
    print(f"  {'-' * (col_w + 33)}")
    for e in entries:
        print(f"  {e['name']:<{col_w}} {e['x']:>10.1f} {e['y']:>10.1f} {e['z']:>10.1f}")


def main():
    args = parse_args()
    rdk, ITEM_TYPE_FRAME = connect()

    all_frames = rdk.ItemList(ITEM_TYPE_FRAME)
    print(f"[INFO] Total frames in station: {len(all_frames)}")

    output = {}

    if args.all_frames:
        output["all_frames"] = [frame_to_dict(f) for f in all_frames]
        print(f"\nall_frames ({len(output['all_frames'])} entries):")
        print_table(output["all_frames"], "all_frames")
    else:
        output["rail_points"] = scrape(rdk, ITEM_TYPE_FRAME, args.rail_pattern)
        output["approach_frames"] = scrape(rdk, ITEM_TYPE_FRAME, args.approach_pattern)

        print(f"\nrail_points  (pattern: '{args.rail_pattern}')  — {len(output['rail_points'])} matches")
        print_table(output["rail_points"], "rail_points")

        print(f"\napproach_frames  (pattern: '{args.approach_pattern}')  — {len(output['approach_frames'])} matches")
        print_table(output["approach_frames"], "approach_frames")

        if not output["rail_points"] and not output["approach_frames"]:
            print("\n[WARN] Nothing matched. Try --all-frames to see everything in the station.")

    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    with open(OUTPUT_PATH, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\n[INFO] Saved to {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
