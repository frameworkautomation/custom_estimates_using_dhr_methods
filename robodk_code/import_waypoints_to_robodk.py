"""
import_waypoints_to_robodk.py

Reads path_config.yaml (the single source of truth, populated by amalgamate_waypoints.py)
and imports every Cartesian waypoint as a RoboDK Target into the live RoboDK station.
Run with RoboDK open.

Usage:
    python robodk_code/import_waypoints_to_robodk.py
    python robodk_code/import_waypoints_to_robodk.py --yaml robo_dk_output/base_cone_waypoints.yaml
    python robodk_code/import_waypoints_to_robodk.py --robodk-ip 172.23.208.1

Default: reads robo_dk_output/path_config.yaml.
Run amalgamate_waypoints.py first to populate it from the GH source files.
"""

import sys
import os
import json
import re
import math
import argparse
import hashlib

sys.path.append("C:/RoboDK/Python")  # silently ignored in WSL; works on Windows

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

# ── RoboDK imports ─────────────────────────────────────────────────────────────
from robodk.robolink import Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_TARGET, ITEM_TYPE_ROBOT
from robodk.robomath import Mat

# ── CONFIG ─────────────────────────────────────────────────────────────────────
ROBOT_NAME     = "Fanuc R2000iC 125L"
DEFAULT_PARENT = "WaypointTargets"
DEFAULT_IP     = "172.23.208.1"

DEFAULT_YAML  = os.path.join(REPO_ROOT, "robo_dk_output", "path_config.yaml")
CACHE_PATH    = os.path.join(REPO_ROOT, "robo_dk_output", "import_waypoints_cache.json")

# Target colours (R, G, B) — RoboDK uses 0-255 per channel packed as 0xRRGGBB
COLOR_MOVEJ = 0x4444FF   # blue
COLOR_MOVEL = 0x44BB44   # green
COLOR_OTHER = 0xAAAAAA   # grey


# ── YAML PARSER ────────────────────────────────────────────────────────────────

def parse_waypoints_yaml(path):
    """
    Parse the waypoints YAML using PyYAML.

    Returns:
        waypoints : list of dicts with keys:
                      name, x, y, z, rx, ry, rz, frame, move_type, j7, note
        edges     : list of dicts with keys: from_name, to_name, tested
    """
    try:
        import yaml
    except ImportError:
        raise ImportError("PyYAML not found. Install with: pip install pyyaml")

    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)

    # path_config.yaml stores waypoints as a dict (name → attrs); flatten to list
    raw_wps = data.get("waypoints") or {}
    if isinstance(raw_wps, dict):
        wp_iter = [{"name": k, **v} for k, v in raw_wps.items() if isinstance(v, dict)]
    else:
        wp_iter = raw_wps  # source YAML files use list format

    waypoints = []
    for wp in wp_iter:
        if not isinstance(wp, dict):
            continue
        # Skip joints-only entries (home, transport) — no Cartesian pose to import
        if "x" not in wp and "y" not in wp and "z" not in wp:
            continue
        j7_raw = wp.get("j7")
        waypoints.append({
            "name":      str(wp.get("name", "")),
            "x":         float(wp.get("x", 0.0)),
            "y":         float(wp.get("y", 0.0)),
            "z":         float(wp.get("z", 0.0)),
            "rx":        float(wp.get("rx", 0.0)),
            "ry":        float(wp.get("ry", 0.0)),
            "rz":        float(wp.get("rz", 0.0)),
            "frame":     str(wp.get("frame", "world")),
            "move_type": str(wp.get("move_type", "MoveJ")),
            "j7":        float(j7_raw) if j7_raw is not None else None,
            "note":      str(wp.get("note", "")),
        })

    edges = []
    for e in (data.get("edges") or []):
        if not isinstance(e, dict):
            continue
        tested = e.get("tested")
        if isinstance(tested, str):
            tested = True if tested.lower() == "true" else (False if tested.lower() == "false" else None)
        edges.append({
            "from_name": str(e.get("from", "")),
            "to_name":   str(e.get("to", "")),
            "tested":    tested,
        })

    return waypoints, edges


# ── POSE BUILDER ───────────────────────────────────────────────────────────────

def build_pose(wp):
    """Return a RoboDK Mat for the waypoint (x/y/z in mm, rx/ry/rz in degrees).

    The GH scripts extract ZYX Euler angles: R = Rz * Ry * Rx.
    TxyzRxyz_2_Pose uses the opposite order (Rx * Ry * Rz), so we build
    the matrix directly instead.
    """
    x, y, z = wp["x"], wp["y"], wp["z"]
    rx = math.radians(wp["rx"])
    ry = math.radians(wp["ry"])
    rz = math.radians(wp["rz"])
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # R = Rz * Ry * Rx
    return Mat([
        [cy*cz,  cz*sx*sy - cx*sz,  cx*cz*sy + sx*sz,  x],
        [cy*sz,  cx*cz + sx*sy*sz,  cx*sy*sz - cz*sx,  y],
        [-sy,    cy*sx,             cx*cy,              z],
        [0,      0,                 0,                  1],
    ])


# ── TARGET COLOR ───────────────────────────────────────────────────────────────

def color_for_move_type(move_type):
    mt = (move_type or "").strip().upper()
    if mt.startswith("MOVEJ"):
        return COLOR_MOVEJ
    if mt.startswith("MOVEL"):
        return COLOR_MOVEL
    return COLOR_OTHER


# ── CACHE HELPERS ──────────────────────────────────────────────────────────────

def waypoint_hash(wp):
    """Stable hash of the fields that define a target's pose and type."""
    key = json.dumps({k: wp[k] for k in ("x","y","z","rx","ry","rz","frame","move_type","j7")}, sort_keys=True)
    return hashlib.md5(key.encode()).hexdigest()

def load_cache():
    if os.path.isfile(CACHE_PATH):
        with open(CACHE_PATH) as f:
            return json.load(f)
    return {}

def save_cache(cache):
    os.makedirs(os.path.dirname(CACHE_PATH), exist_ok=True)
    with open(CACHE_PATH, "w") as f:
        json.dump(cache, f, indent=2)


# ── MAIN ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Import waypoints YAML as RoboDK Targets into the live station."
    )
    parser.add_argument(
        "--yaml", default=DEFAULT_YAML,
        help=f"Path to waypoints YAML (default: {DEFAULT_YAML})"
    )
    parser.add_argument(
        "--robodk-ip", default=DEFAULT_IP,
        help=f"RoboDK API host (default: {DEFAULT_IP})"
    )
    parser.add_argument(
        "--parent-name", default=DEFAULT_PARENT,
        help=f"Name of the parent frame group in RoboDK (default: {DEFAULT_PARENT})"
    )
    parser.add_argument(
        "--force", action="store_true",
        help="Re-import all targets even if unchanged."
    )
    args = parser.parse_args()

    yaml_path = args.yaml if os.path.isabs(args.yaml) else os.path.join(REPO_ROOT, args.yaml)

    if not os.path.isfile(yaml_path):
        print(f"ERROR: YAML file not found: {yaml_path}")
        sys.exit(1)

    # ── Parse YAML ────────────────────────────────────────────────────────────
    print(f"Parsing: {yaml_path}")
    waypoints, edges = parse_waypoints_yaml(yaml_path)
    print(f"  Found {len(waypoints)} waypoint(s), {len(edges)} edge(s)")

    if not waypoints:
        print("No waypoints found — nothing to import.")
        sys.exit(0)

    # ── Connect to RoboDK ─────────────────────────────────────────────────────
    print(f"Connecting to RoboDK at {args.robodk_ip}:20500 ...")
    from robodk.robolink import Robolink
    RDK = Robolink(args.robodk_ip)

    robot = RDK.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        print(f"ERROR: Robot '{ROBOT_NAME}' not found in the station.")
        sys.exit(1)

    station = RDK.ActiveStation()


    # ── Resolve / create parent frame ─────────────────────────────────────────
    parent_frame = RDK.Item(args.parent_name, ITEM_TYPE_FRAME)
    if not parent_frame.Valid():
        print(f"  Creating parent frame '{args.parent_name}' under station root ...")
        parent_frame = RDK.AddFrame(args.parent_name, station)
        from robodk.robomath import eye
        parent_frame.setPose(eye(4))

    # ── Import targets ────────────────────────────────────────────────────────
    cache = {} if args.force else load_cache()
    created = 0
    skipped = 0
    replaced = 0
    failed = 0

    for wp in waypoints:
        name = wp["name"]
        h = waypoint_hash(wp)

        # Skip if unchanged and already in station
        if not args.force and cache.get(name) == h:
            existing = RDK.Item(name, ITEM_TYPE_TARGET)
            if existing.Valid():
                skipped += 1
                continue

        try:
            world_pose = build_pose(wp)

            existing = RDK.Item(name, ITEM_TYPE_TARGET)
            if existing.Valid():
                existing.Delete()
                replaced += 1

            target = RDK.AddTarget(name, parent_frame, robot)
            target.setAsCartesianTarget()
            target.setPose(world_pose)

            try:
                color = color_for_move_type(wp["move_type"])
                target.setColor([
                    ((color >> 16) & 0xFF) / 255.0,
                    ((color >> 8)  & 0xFF) / 255.0,
                    ((color)       & 0xFF) / 255.0,
                    1.0,
                ])
            except Exception:
                pass

            note_str = f"  [{wp['move_type']}  j7={wp['j7']}  frame={wp['frame']}]"
            if wp.get("note"):
                note_str += f"  # {wp['note']}"
            print(f"  + {name}{note_str}")
            cache[name] = h
            created += 1

        except Exception as exc:
            print(f"  FAILED {name}: {exc}")
            failed += 1

    save_cache(cache)

    # ── Summary ───────────────────────────────────────────────────────────────
    print()
    print(f"Done.  Created: {created}  Skipped (unchanged): {skipped}  (replaced existing: {replaced})  Failed: {failed}")
    if edges:
        print(f"Note: {len(edges)} edge(s) defined in the YAML — "
              "edges are not imported as RoboDK items (use check_collision_free_paths.py to test them).")


if __name__ == "__main__":
    main()
