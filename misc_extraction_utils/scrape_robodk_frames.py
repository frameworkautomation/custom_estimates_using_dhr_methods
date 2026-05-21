"""
scrape_robodk_frames.py

Reads DHR's robodk.yaml (no RoboDK connection needed) and extracts frame
poses by name pattern. Composes the full parent-chain transforms so output
positions are in world space, matching what RoboDK would show.

Two default groups:

  --rail-pattern   (default: ^OptimizationApproachMachine\d+$)
      One OptimizationApproach frame per machine (primary, no Shifted/Oil
      variants). These mark each machine's position along the rail axis.

  --approach-pattern   (default: CurtainSafe$)
      All CurtainSafe frames -- one per machine door approach position.

Output: robo_dk_output/scraped_frames.json
        Keys: "rail_points" and "approach_frames" (or "all_frames" with --all-frames).
        Each entry: name, x, y, z, rx, ry, rz, xaxis, yaxis, zaxis.

Usage:
    python misc_extraction_utils/scrape_robodk_frames.py
    python misc_extraction_utils/scrape_robodk_frames.py --rail-pattern "OptimizationApproachMachine"
    python misc_extraction_utils/scrape_robodk_frames.py --all-frames
    python misc_extraction_utils/scrape_robodk_frames.py --yaml path/to/other.yaml
"""

import argparse
import json
import os
import sys

# Allow running as: python misc_extraction_utils/scrape_robodk_frames.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robodk_yaml_reader import DEFAULT_YAML_PATH, load_all_frames, filter_frames

OUTPUT_PATH = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output", "scraped_frames.json"
))


def parse_args():
    p = argparse.ArgumentParser(
        description="Extract frame world poses from robodk.yaml (no RoboDK needed)."
    )
    p.add_argument(
        "--yaml",
        default=DEFAULT_YAML_PATH,
        help="Path to robodk.yaml",
    )
    p.add_argument(
        "--rail-pattern",
        default=r"^OptimizationApproachMachine\d+$",
        help="Regex for rail-point frames (default: '^OptimizationApproachMachine\\d+$')",
    )
    p.add_argument(
        "--approach-pattern",
        default="CurtainSafe$",
        help="Regex for approach frames (default: 'CurtainSafe$')",
    )
    p.add_argument(
        "--all-frames",
        action="store_true",
        help="Dump every frame into a single 'all_frames' key",
    )
    return p.parse_args()


def print_table(entries):
    if not entries:
        print("  (none)")
        return
    col_w = max(len(e["name"]) for e in entries) + 2
    print(f"  {'Name':<{col_w}} {'x':>10} {'y':>10} {'z':>10}")
    print(f"  {'-' * (col_w + 33)}")
    for e in entries:
        print(f"  {e['name']:<{col_w}} {e['x']:>10.1f} {e['y']:>10.1f} {e['z']:>10.1f}")


def main():
    args = parse_args()

    if not os.path.isfile(args.yaml):
        print(f"[ERROR] YAML not found: {args.yaml}")
        print("        Run cloning_stuff/make_clones.sh first.")
        sys.exit(1)

    print(f"[INFO] Reading {args.yaml}")
    all_frames = load_all_frames(args.yaml)
    print(f"[INFO] Total named items: {len(all_frames)}")

    output = {}

    if args.all_frames:
        output["all_frames"] = all_frames
        print(f"\nall_frames: {len(all_frames)} entries")
        print_table(all_frames)
    else:
        output["rail_points"]    = filter_frames(all_frames, args.rail_pattern)
        output["approach_frames"] = filter_frames(all_frames, args.approach_pattern)

        print(f"\nrail_points  (pattern: '{args.rail_pattern}')  - {len(output['rail_points'])} matches")
        print_table(output["rail_points"])

        print(f"\napproach_frames  (pattern: '{args.approach_pattern}')  - {len(output['approach_frames'])} matches")
        print_table(output["approach_frames"])

        if not output["rail_points"] and not output["approach_frames"]:
            print("\n[WARN] Nothing matched. Try --all-frames to dump everything.")

    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    with open(OUTPUT_PATH, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)
    print(f"\n[INFO] Saved to {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
