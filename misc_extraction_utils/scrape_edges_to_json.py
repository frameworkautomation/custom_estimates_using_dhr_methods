"""
scrape_edges_to_json.py

Reads DHR's robodk.yaml and extracts two types of edges:

  1. rail_edges   -- connections between consecutive OptimizationApproachMachine
                     frames sorted by X position (the rail corridor).
  2. approach_edges -- connections from each rail point to its corresponding
                       CurtainSafe approach frame (machine door approaches).

Each edge entry:
  {
    "name":   "Machine1_rail -> Machine2_rail",
    "from":   {"name": "...", "x": ..., "y": ..., "z": ...},
    "to":     {"name": "...", "x": ..., "y": ..., "z": ...}
  }

Output: robo_dk_output/scraped_edges.json
        Keys: "rail_edges", "approach_edges"

Usage:
    python misc_extraction_utils/scrape_edges_to_json.py
    python misc_extraction_utils/scrape_edges_to_json.py --yaml path/to/other.yaml
"""

import argparse
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robodk_yaml_reader import DEFAULT_YAML_PATH, load_all_frames, filter_frames, frames_by_name

OUTPUT_PATH = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output", "scraped_edges.json"
))

# Patterns
RAIL_PATTERN     = r"^OptimizationApproachMachine\d+$"
APPROACH_PATTERN = r"CurtainSafe$"


def _point(entry):
    return {"name": entry["name"], "x": entry["x"], "y": entry["y"], "z": entry["z"]}


def build_rail_edges(rail_frames):
    """
    Connect consecutive rail frames sorted by X position.
    Returns a list of edge dicts.
    """
    sorted_frames = sorted(rail_frames, key=lambda e: e["x"])
    edges = []
    for i in range(len(sorted_frames) - 1):
        a = sorted_frames[i]
        b = sorted_frames[i + 1]
        edges.append({
            "name": f"{a['name']} -> {b['name']}",
            "from": _point(a),
            "to":   _point(b),
        })
    return edges


def _machine_number(name):
    """Extract the leading integer from a frame name, e.g. 'ApproachMachine3CurtainSafe' -> 3."""
    m = re.search(r"\d+", name)
    return int(m.group()) if m else -1


def build_approach_edges(rail_frames, approach_frames):
    """
    For each rail point, find the matching CurtainSafe frame by machine number
    and emit an edge between them.
    Returns a list of edge dicts.
    """
    approach_by_num = {}
    for e in approach_frames:
        n = _machine_number(e["name"])
        if n >= 0:
            approach_by_num[n] = e

    edges = []
    for rail_entry in sorted(rail_frames, key=lambda e: e["x"]):
        n = _machine_number(rail_entry["name"])
        if n in approach_by_num:
            ap = approach_by_num[n]
            edges.append({
                "name": f"{rail_entry['name']} -> {ap['name']}",
                "from": _point(rail_entry),
                "to":   _point(ap),
            })
    return edges


def parse_args():
    p = argparse.ArgumentParser(
        description="Extract rail and approach edges from robodk.yaml (no RoboDK needed)."
    )
    p.add_argument("--yaml", default=DEFAULT_YAML_PATH, help="Path to robodk.yaml")
    return p.parse_args()


def print_edges(edges):
    if not edges:
        print("  (none)")
        return
    for e in edges:
        f, t = e["from"], e["to"]
        print(f"  {e['name']}")
        print(f"    from ({f['x']:8.1f}, {f['y']:8.1f}, {f['z']:8.1f})"
              f"  to ({t['x']:8.1f}, {t['y']:8.1f}, {t['z']:8.1f})")


def main():
    args = parse_args()

    if not os.path.isfile(args.yaml):
        print(f"[ERROR] YAML not found: {args.yaml}")
        print("        Run cloning_stuff/make_clones.sh first.")
        sys.exit(1)

    print(f"[INFO] Reading {args.yaml}")
    all_frames = load_all_frames(args.yaml)
    print(f"[INFO] Total named items: {len(all_frames)}")

    rail_frames     = filter_frames(all_frames, RAIL_PATTERN)
    approach_frames = filter_frames(all_frames, APPROACH_PATTERN)

    print(f"[INFO] Rail points matched: {len(rail_frames)}")
    print(f"[INFO] Approach frames matched: {len(approach_frames)}")

    rail_edges     = build_rail_edges(rail_frames)
    approach_edges = build_approach_edges(rail_frames, approach_frames)

    output = {
        "rail_edges":     rail_edges,
        "approach_edges": approach_edges,
    }

    print(f"\nrail_edges ({len(rail_edges)} edges):")
    print_edges(rail_edges)

    print(f"\napproach_edges ({len(approach_edges)} edges):")
    print_edges(approach_edges)

    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    with open(OUTPUT_PATH, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)
    print(f"\n[INFO] Saved to {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
