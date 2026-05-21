"""
import_waypoints_to_robodk.py

Reads a waypoints YAML file and imports every waypoint as a RoboDK Target
into the live RoboDK station.  Run with RoboDK open.

Usage:
    python robodk_code/import_waypoints_to_robodk.py
    python robodk_code/import_waypoints_to_robodk.py --yaml robo_dk_output/base_cone_waypoints.yaml
    python robodk_code/import_waypoints_to_robodk.py --yaml robo_dk_output/machine_cone_waypoints.yaml
    python robodk_code/import_waypoints_to_robodk.py --robodk-ip 172.23.208.1

YAML format note:
    The waypoints YAML uses a non-standard inline format where multiple keys
    appear on a single line (e.g. "x: 0.0  y: 0.0  z: 500.0").  PyYAML
    treats that as a plain string, not a mapping.  This script parses the
    file as raw text instead of via yaml.safe_load.
"""

import sys
import os
import re
import math
import argparse

sys.path.append("C:/RoboDK/Python")  # silently ignored in WSL; works on Windows

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

# ── RoboDK imports ─────────────────────────────────────────────────────────────
from robodk.robolink import Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_TARGET, ITEM_TYPE_ROBOT
from robodk.robomath import TxyzRxyz_2_Pose

# ── CONFIG ─────────────────────────────────────────────────────────────────────
ROBOT_NAME         = "Fanuc R2000iC 125L"
DEFAULT_YAML       = os.path.join(REPO_ROOT, "robo_dk_output", "base_cone_waypoints.yaml")
DEFAULT_PARENT     = "WaypointTargets"
DEFAULT_IP         = "localhost"

# Target colours (R, G, B) — RoboDK uses 0-255 per channel packed as 0xRRGGBB
COLOR_MOVEJ = 0x4444FF   # blue
COLOR_MOVEL = 0x44BB44   # green
COLOR_OTHER = 0xAAAAAA   # grey


# ── RAW-TEXT YAML PARSER ───────────────────────────────────────────────────────

def _kv_pairs(line):
    """Return dict of all key: value tokens found on a single line."""
    return {k: v for k, v in re.findall(r'(\w+):\s*([-\d.eE+]+)', line)}


def _str_value(line, key):
    """Return the string value after 'key:' on a line, stripped of quotes/whitespace."""
    m = re.search(rf'{key}:\s*([^\s#][^\n]*)', line)
    if not m:
        return None
    return m.group(1).strip().strip('"').strip("'")


def parse_waypoints_yaml(path):
    """
    Parse the non-standard waypoints YAML by reading raw text lines.

    Returns:
        waypoints : list of dicts with keys:
                      name, x, y, z, rx, ry, rz, frame, move_type, j7, note
        edges     : list of dicts with keys: from_name, to_name, tested
    """
    with open(path, "r", encoding="utf-8") as fh:
        lines = fh.readlines()

    waypoints = []
    edges = []

    # ── state machine ──────────────────────────────────────────────────────────
    in_waypoints = False
    in_edges     = False
    current_wp   = None
    current_edge = None

    def flush_wp():
        if current_wp and current_wp.get("name"):
            waypoints.append(current_wp)

    def flush_edge():
        if current_edge and current_edge.get("from_name") and current_edge.get("to_name"):
            edges.append(current_edge)

    for raw in lines:
        line = raw.rstrip("\n")
        stripped = line.strip()

        # Skip blank lines and comments
        if not stripped or stripped.startswith("#"):
            continue

        # Section headers
        if re.match(r'^waypoints\s*:', line):
            in_waypoints = True
            in_edges     = False
            continue

        if re.match(r'^edges\s*:', line):
            flush_wp()
            current_wp = None
            in_waypoints = False
            in_edges     = True
            continue

        # ── Waypoints section ──────────────────────────────────────────────────
        if in_waypoints:
            # Start of a new waypoint entry
            m = re.match(r'\s*-\s*name:\s*(.+)', line)
            if m:
                flush_wp()
                current_wp = {
                    "name": m.group(1).strip().strip('"').strip("'"),
                    "x": 0.0, "y": 0.0, "z": 0.0,
                    "rx": 0.0, "ry": 0.0, "rz": 0.0,
                    "frame": "world",
                    "move_type": "MoveJ",
                    "j7": 0.0,
                    "note": "",
                }
                continue

            if current_wp is None:
                continue

            # Position line: x: ... y: ... z: ...
            if re.search(r'\bx\s*:', line) and re.search(r'\by\s*:', line) and re.search(r'\bz\s*:', line):
                kv = _kv_pairs(line)
                if "x" in kv:
                    current_wp["x"] = float(kv["x"])
                if "y" in kv:
                    current_wp["y"] = float(kv["y"])
                if "z" in kv:
                    current_wp["z"] = float(kv["z"])
                continue

            # Rotation line: rx: ... ry: ... rz: ...
            if re.search(r'\brx\s*:', line) and re.search(r'\bry\s*:', line) and re.search(r'\brz\s*:', line):
                kv = _kv_pairs(line)
                if "rx" in kv:
                    current_wp["rx"] = float(kv["rx"])
                if "ry" in kv:
                    current_wp["ry"] = float(kv["ry"])
                if "rz" in kv:
                    current_wp["rz"] = float(kv["rz"])
                continue

            # Scalar fields on their own lines
            for field in ("frame", "move_type", "note"):
                m = re.match(rf'\s+{field}:\s*(.+)', line)
                if m:
                    current_wp[field] = m.group(1).strip().strip('"').strip("'")
                    break

            m = re.match(r'\s+j7:\s*([-\d.eE+]+)', line)
            if m:
                current_wp["j7"] = float(m.group(1))

        # ── Edges section ─────────────────────────────────────────────────────
        elif in_edges:
            # Start of a new edge entry
            m = re.match(r'\s*-\s*from:\s*(.+)', line)
            if m:
                flush_edge()
                current_edge = {
                    "from_name": m.group(1).strip().strip('"').strip("'"),
                    "to_name":   None,
                    "tested":    None,
                }
                continue

            if current_edge is None:
                continue

            m = re.match(r'\s+to:\s*(.+)', line)
            if m:
                current_edge["to_name"] = m.group(1).strip().strip('"').strip("'")
                continue

            m = re.match(r'\s+tested:\s*(.+)', line)
            if m:
                val = m.group(1).strip()
                if val.lower() in ("true", "yes"):
                    current_edge["tested"] = True
                elif val.lower() in ("false", "no"):
                    current_edge["tested"] = False
                else:
                    current_edge["tested"] = None
                continue

    # Flush final entries
    flush_wp()
    flush_edge()

    return waypoints, edges


# ── POSE BUILDER ───────────────────────────────────────────────────────────────

def build_pose(wp):
    """Return a RoboDK Mat for the waypoint (x/y/z in mm, rx/ry/rz in degrees)."""
    return TxyzRxyz_2_Pose([
        wp["x"], wp["y"], wp["z"],
        math.radians(wp["rx"]),
        math.radians(wp["ry"]),
        math.radians(wp["rz"]),
    ])


# ── TARGET COLOR ───────────────────────────────────────────────────────────────

def color_for_move_type(move_type):
    mt = (move_type or "").strip().upper()
    if mt.startswith("MOVEJ"):
        return COLOR_MOVEJ
    if mt.startswith("MOVEL"):
        return COLOR_MOVEL
    return COLOR_OTHER


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

    # ── Robot base pose (needed for robot_local frames) ───────────────────────
    robot_base_pose = robot.PoseFrame()

    # ── Resolve / create parent frame ─────────────────────────────────────────
    parent_frame = RDK.Item(args.parent_name, ITEM_TYPE_FRAME)
    if not parent_frame.Valid():
        print(f"  Creating parent frame '{args.parent_name}' under station root ...")
        parent_frame = RDK.AddFrame(args.parent_name, station)
        from robodk.robomath import eye
        parent_frame.setPose(eye(4))

    # ── Import targets ────────────────────────────────────────────────────────
    created = 0
    replaced = 0
    failed = 0

    for wp in waypoints:
        name = wp["name"]
        try:
            local_pose = build_pose(wp)

            if wp["frame"] == "robot_local":
                world_pose = robot_base_pose * local_pose
            else:
                # "world" or unrecognised — use as-is
                world_pose = local_pose

            # Delete existing target with the same name if present
            existing = RDK.Item(name, ITEM_TYPE_TARGET)
            if existing.Valid():
                existing.Delete()
                replaced += 1

            target = RDK.AddTarget(name, parent_frame, robot)
            target.setPose(world_pose)

            # Colour by move type (best-effort — not all RoboDK versions expose setColor on targets)
            try:
                color = color_for_move_type(wp["move_type"])
                target.setColor([
                    ((color >> 16) & 0xFF) / 255.0,
                    ((color >> 8)  & 0xFF) / 255.0,
                    ((color)       & 0xFF) / 255.0,
                    1.0,
                ])
            except Exception:
                pass  # color is cosmetic — don't fail the import

            note_str = f"  [{wp['move_type']}  j7={wp['j7']}  frame={wp['frame']}]"
            if wp.get("note"):
                note_str += f"  # {wp['note']}"
            print(f"  + {name}{note_str}")
            created += 1

        except Exception as exc:
            print(f"  FAILED {name}: {exc}")
            failed += 1

    # ── Summary ───────────────────────────────────────────────────────────────
    print()
    print(f"Done.  Created: {created}  (replaced existing: {replaced})  Failed: {failed}")
    if edges:
        print(f"Note: {len(edges)} edge(s) defined in the YAML — "
              "edges are not imported as RoboDK items (use check_collision_free_paths.py to test them).")


if __name__ == "__main__":
    main()
