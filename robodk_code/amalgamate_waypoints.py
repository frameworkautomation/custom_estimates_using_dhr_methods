"""
amalgamate_waypoints.py

Merges multiple waypoint YAML files into a single all_waypoints.yaml.
Source files and output path are read from robo_dk_output/waypoint_sources.json.

Also updates path_config.yaml: any Cartesian waypoint (one with x/y/z) that is
not already present in path_config.yaml's `waypoints:` section is appended there.
This lets check_collision_free_paths.py see the GH-generated cone positions, and
lets visualize_waypoints.py show them by reading a single file.

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
PATH_CONFIG_PATH = os.path.join(REPO_ROOT, "robo_dk_output", "path_config.yaml")

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
        lines.append(f"    source: {w.get('source', 'unknown')}")
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

    # Pull any Cartesian waypoints the user manually added to path_config.yaml
    # (joints-only entries like home/transport are skipped — no pose to visualise)
    if os.path.exists(PATH_CONFIG_PATH):
        with open(PATH_CONFIG_PATH, "r", encoding="utf-8") as f:
            pc = yaml.safe_load(f)
        pc_wps = (pc or {}).get("waypoints") or {}
        added_from_pc = 0
        for name, attrs in pc_wps.items():
            if name in seen_names or not isinstance(attrs, dict):
                continue
            if "x" not in attrs and "y" not in attrs and "z" not in attrs:
                continue
            wp = {"name": name}
            for k in ("x", "y", "z", "rx", "ry", "rz", "frame", "move_type", "j7", "source"):
                if k in attrs:
                    wp[k] = attrs[k]
            wp.setdefault("source", "human")
            seen_names.add(name)
            all_waypoints.append(wp)
            added_from_pc += 1
        if added_from_pc:
            print(f"[OK] path_config.yaml: {added_from_pc} human Cartesian waypoints")

    write_yaml(all_waypoints, all_edges, output_path)
    print(f"\n[OK] {len(all_waypoints)} total waypoints, {len(all_edges)} total edges -> {output_path}")

    _update_path_config(all_waypoints)


def _update_path_config(waypoints):
    """Append Cartesian waypoints from all_waypoints into path_config.yaml.

    Only waypoints that have x/y/z are written (joints-only routing candidates
    already live in path_config.yaml and are not touched). Existing names are
    skipped. New entries are inserted just before the `routing_candidates:` line
    so the human-edited structure and all comments are preserved.
    """
    if not os.path.exists(PATH_CONFIG_PATH):
        print(f"[WARN] path_config.yaml not found, skipping: {PATH_CONFIG_PATH}")
        return

    with open(PATH_CONFIG_PATH, "r", encoding="utf-8") as f:
        text = f.read()
    existing = yaml.safe_load(text) or {}
    existing_names = set((existing.get("waypoints") or {}).keys())

    to_add = [
        wp for wp in waypoints
        if "x" in wp and wp["name"] not in existing_names
    ]
    if not to_add:
        print("[OK] path_config.yaml: no new Cartesian waypoints to add")
        return

    # Build the YAML lines for each new waypoint (indented under `waypoints:`)
    new_lines = []
    for wp in to_add:
        new_lines.append(f"  {wp['name']}:")
        for key in ("x", "y", "z", "rx", "ry", "rz"):
            new_lines.append(f"    {key}: {wp.get(key, 0.0)}")
        new_lines.append(f"    frame: {wp.get('frame', 'robot_local')}")
        new_lines.append(f"    move_type: {wp.get('move_type', 'MoveJ')}")
        j7 = wp.get("j7")
        new_lines.append(f"    j7: {'null' if j7 is None else j7}")
        new_lines.append(f"    source: {wp.get('source', 'grasshopper')}")
        if wp.get("z_axis_free"):
            new_lines.append(f"    z_axis_free: true")

    insert_block = "\n".join(new_lines) + "\n"

    # Insert just before `routing_candidates:` to stay inside the waypoints block
    marker = "routing_candidates:"
    idx = text.find(marker)
    if idx == -1:
        # Fallback: append at end
        updated = text.rstrip("\n") + "\n" + insert_block
    else:
        updated = text[:idx] + insert_block + text[idx:]

    with open(PATH_CONFIG_PATH, "w", encoding="utf-8") as f:
        f.write(updated)
    print(f"[OK] path_config.yaml: added {len(to_add)} Cartesian waypoints")


if __name__ == "__main__":
    main()
