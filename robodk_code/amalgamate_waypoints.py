"""
amalgamate_waypoints.py

Reads waypoint YAML source files and merges everything into path_config.yaml —
the single source of truth for all waypoints and edges.

Source files are listed in robo_dk_output/waypoint_sources.json ("sources" key).

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
import re
import sys

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
SOURCES_CONFIG_PATH = os.path.join(REPO_ROOT, "robo_dk_output", "waypoint_sources.json")
PATH_CONFIG_PATH = os.path.join(REPO_ROOT, "robo_dk_output", "path_config.yaml")

try:
    import yaml
except ImportError:
    print("PyYAML not found. Install with: pip install pyyaml")
    sys.exit(1)


def main():
    if not os.path.exists(SOURCES_CONFIG_PATH):
        print(f"[ERROR] Config not found: {SOURCES_CONFIG_PATH}")
        sys.exit(1)

    with open(SOURCES_CONFIG_PATH, "r", encoding="utf-8") as f:
        config = json.load(f)

    sources = config.get("sources", [])

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

    print(f"\n[OK] Collected {len(all_waypoints)} waypoints, {len(all_edges)} edges")
    _update_path_config(all_waypoints, all_edges)


def _update_path_config(waypoints, edges):
    """Write Cartesian waypoints and edges into path_config.yaml.

    Waypoints: only those with x/y/z. Inserted before `routing_candidates:`.
    Edges: entire edges block is replaced at the end of the file under a marker
    comment so it is easy to find and update on subsequent runs.
    """
    if not os.path.exists(PATH_CONFIG_PATH):
        print(f"[INFO] path_config.yaml not found, creating: {PATH_CONFIG_PATH}")
        os.makedirs(os.path.dirname(PATH_CONFIG_PATH), exist_ok=True)
        text = "waypoints:\n\nedges: []\n"
        with open(PATH_CONFIG_PATH, "w", encoding="utf-8") as f:
            f.write(text)
    else:
        with open(PATH_CONFIG_PATH, "r", encoding="utf-8") as f:
            text = f.read()
    existing = yaml.safe_load(text) or {}

    # ── Rebuild waypoints and edges from scratch ──────────────────────────────
    # Keep non-grasshopper (human) waypoints, replace all grasshopper ones
    existing_wps = existing.get("waypoints") or {}
    human_wps = {}
    removed = 0
    for name, attrs in existing_wps.items():
        if not isinstance(attrs, dict):
            continue
        if attrs.get("source") == "grasshopper":
            removed += 1
        else:
            human_wps[name] = attrs
    if removed:
        print(f"[OK] Removed {removed} stale grasshopper waypoints")

    # Add all current grasshopper waypoints
    gh_wps = {}
    for wp in waypoints:
        if "x" not in wp:
            continue
        name = wp["name"]
        entry = {}
        for k in ("x", "y", "z", "rx", "ry", "rz", "frame", "move_type", "j7", "source", "z_axis_free", "note"):
            if k in wp:
                entry[k] = wp[k]
        entry.setdefault("source", "grasshopper")
        gh_wps[name] = entry

    all_wps = {**human_wps, **gh_wps}
    print(f"[OK] {len(human_wps)} human + {len(gh_wps)} grasshopper = {len(all_wps)} waypoints")

    # Build output
    out = {"waypoints": all_wps}

    # Edges — write current set (no merging)
    out_edges = []
    for e in edges:
        tested = e.get("tested")
        out_edges.append({
            "from": e["from"],
            "to": e["to"],
            "tested": tested,
        })
    out["edges"] = out_edges
    print(f"[OK] {len(out_edges)} edges written")

    with open(PATH_CONFIG_PATH, "w", encoding="utf-8") as f:
        yaml.dump(out, f, default_flow_style=False, sort_keys=False, allow_unicode=True)


if __name__ == "__main__":
    main()
