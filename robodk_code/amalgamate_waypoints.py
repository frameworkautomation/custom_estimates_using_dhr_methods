"""
amalgamate_waypoints.py

Merges multiple waypoint YAML files into a single all_waypoints.yaml.
Source files and output path are read from robo_dk_output/waypoint_sources.json.

Usage:
    python robodk_code/amalgamate_waypoints.py

To add a new YAML source, edit waypoint_sources.json:
    "sources": [
        "robo_dk_output/base_cone_waypoints.yaml",
        "robo_dk_output/machine_cone_waypoints.yaml",
        "robo_dk_output/my_new_waypoints.yaml"
    ]
"""

import os
import json
import sys

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
CONFIG_PATH = os.path.join(REPO_ROOT, "robo_dk_output", "waypoint_sources.json")

try:
    import yaml
except ImportError:
    print("PyYAML not found. Install with: pip install pyyaml")
    sys.exit(1)


def write_yaml(waypoints, edges, path):
    lines = ["waypoints:"]
    for w in waypoints:
        lines.append(f"  - name: {w['name']}")
        lines.append(f"    x: {w.get('x', 0.0)}")
        lines.append(f"    y: {w.get('y', 0.0)}")
        lines.append(f"    z: {w.get('z', 0.0)}")
        lines.append(f"    rx: {w.get('rx', 0.0)}")
        lines.append(f"    ry: {w.get('ry', 0.0)}")
        lines.append(f"    rz: {w.get('rz', 0.0)}")
        lines.append(f"    frame: {w.get('frame', 'robot_local')}")
        lines.append(f"    move_type: {w.get('move_type', 'MoveJ')}")
        j7 = w.get('j7')
        lines.append(f"    j7: {j7 if j7 is not None else 'null'}")
        if w.get('note'):
            lines.append(f"    note: \"{w['note']}\"")
    lines.append("")
    lines.append("edges:")
    for e in edges:
        lines.append(f"  - from: {e['from']}")
        lines.append(f"    to:   {e['to']}")
        tested = e.get('tested')
        lines.append(f"    tested: {tested if tested is not None else 'null'}")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    if not os.path.exists(CONFIG_PATH):
        print(f"[ERROR] Config not found: {CONFIG_PATH}")
        sys.exit(1)

    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        config = json.load(f)

    sources = config.get("sources", [])
    output_rel = config.get("output", "robo_dk_output/all_waypoints.yaml")
    output_path = os.path.join(REPO_ROOT, output_rel.replace("/", os.sep))

    all_waypoints = []
    all_edges = []
    seen_names = set()
    seen_edges = set()

    for src_rel in sources:
        src_path = os.path.join(REPO_ROOT, src_rel.replace("/", os.sep))
        if not os.path.exists(src_path):
            print(f"[WARN] Source not found, skipping: {src_path}")
            continue
        with open(src_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if not data:
            print(f"[WARN] Empty YAML, skipping: {src_path}")
            continue

        wps = data.get("waypoints") or []
        edges = data.get("edges") or []

        added_wp = 0
        for wp in wps:
            name = wp.get("name", "")
            if name in seen_names:
                print(f"[WARN] Duplicate waypoint '{name}' from {src_rel}, skipping")
                continue
            seen_names.add(name)
            all_waypoints.append(wp)
            added_wp += 1

        added_edge = 0
        for e in edges:
            key = (e.get("from", ""), e.get("to", ""))
            if key in seen_edges:
                continue
            seen_edges.add(key)
            all_edges.append(e)
            added_edge += 1

        print(f"[OK] {src_rel}: {added_wp} waypoints, {added_edge} edges")

    write_yaml(all_waypoints, all_edges, output_path)
    print(f"\n[OK] {len(all_waypoints)} total waypoints, {len(all_edges)} total edges -> {output_path}")


if __name__ == "__main__":
    main()
