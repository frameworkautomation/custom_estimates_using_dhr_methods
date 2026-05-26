"""
export_machine_cone_waypoints.py

Reads all cone_grab_* targets from the live RoboDK station and writes their
world-space poses + approach offsets to machine_cone_waypoints.yaml.

Run with RoboDK open (station already loaded):
    python robodk_code/export_machine_cone_waypoints.py

Optional args:
    --approach-mm FLOAT   approach offset along grab Z-axis (default 200.0)
    --output PATH         output YAML path (default: robo_dk_output/machine_cone_waypoints.yaml)
    --robodk-ip IP        RoboDK host (default: 172.23.208.1)

Output YAML schema (world frame, j7 free):
    waypoints:
      - name: cone_grab_N_approach
        x: ...  y: ...  z: ...
        rx: ...  ry: ...  rz: ...
        frame: world
        move_type: MoveJ
        j7: null
        note: "machine cone approach"
      - name: cone_grab_N
        ...
        move_type: MoveL
        j7: null
        note: "machine cone place"
    edges:
      - from: cone_grab_N_approach
        to:   cone_grab_N
        tested: null
      - from: cone_grab_N
        to:   cone_grab_N_approach
        tested: null
"""

import sys
import os
import argparse
import math

sys.path.append("C:/RoboDK/Python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robodk.robolink import Robolink, ITEM_TYPE_TARGET
from robodk.robomath import transl, Pose_2_TxyzRxyz

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
DEFAULT_OUTPUT = os.path.join(REPO_ROOT, "robo_dk_output", "machine_cone_waypoints.yaml")
DEFAULT_APPROACH_MM = 200.0


# ── helpers ───────────────────────────────────────────────────────────────────

def connect(ip="localhost"):
    try:
        rdk = Robolink(robodk_ip=ip)
        rdk.Item("")   # ping
        print(f"[INFO] Connected to RoboDK at {ip}")
        return rdk
    except Exception as e:
        raise RuntimeError(f"Cannot connect to RoboDK at {ip}: {e}")


def pose_to_xyzrpw(pose):
    """Return (x, y, z, rx, ry, rz) in mm / degrees from a RoboDK Mat."""
    v = Pose_2_TxyzRxyz(pose)
    x, y, z = v[0], v[1], v[2]
    rx = math.degrees(v[3])
    ry = math.degrees(v[4])
    rz = math.degrees(v[5])
    return round(x, 4), round(y, 4), round(z, 4), round(rx, 6), round(ry, 6), round(rz, 6)


def approach_pose(grab_pose, offset_mm):
    """Offset along grab frame's local Z-axis (same as make_approach_pose in moving_a_cone.py)."""
    return grab_pose * transl(0, 0, offset_mm)


def find_dest_cones(rdk):
    """Return sorted list of all cone_grab_* targets from the station."""
    targets = [t for t in rdk.ItemList(ITEM_TYPE_TARGET)
               if t.Name().startswith("cone_grab_")]
    return sorted(targets, key=lambda t: t.Name())


# ── YAML writer (same format as export_base_cone_waypoints.py) ───────────────

def write_yaml(waypoints, edges, path):
    lines = ["waypoints:"]
    for w in waypoints:
        lines.append(f"  - name: {w['name']}")
        lines.append(f"    x: {w['x']}  y: {w['y']}  z: {w['z']}")
        lines.append(f"    rx: {w['rx']}  ry: {w['ry']}  rz: {w['rz']}")
        lines.append(f"    frame: world")
        lines.append(f"    move_type: {w['move_type']}")
        lines.append(f"    j7: null")
        if w.get("note"):
            lines.append(f"    note: \"{w['note']}\"")

    lines.append("")
    lines.append("edges:")
    for e in edges:
        lines.append(f"  - from: {e['from']}")
        lines.append(f"    to:   {e['to']}")
        lines.append(f"    tested: null")

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Export machine cone waypoints from RoboDK to YAML")
    parser.add_argument("--approach-mm", type=float, default=DEFAULT_APPROACH_MM,
                        help=f"Approach offset along grab Z-axis in mm (default {DEFAULT_APPROACH_MM})")
    parser.add_argument("--output",      default=DEFAULT_OUTPUT,
                        help=f"Output YAML path (default: {DEFAULT_OUTPUT})")
    parser.add_argument("--robodk-ip",   default="172.23.208.1",
                        help="RoboDK host IP (default: 172.23.208.1)")
    args = parser.parse_args()

    rdk = connect(args.robodk_ip)

    cones = find_dest_cones(rdk)
    if not cones:
        raise RuntimeError("No cone_grab_* targets found in station. "
                           "Is the station loaded? (run setup_station_caller.py first)")

    print(f"\nFound {len(cones)} cone_grab_* targets:")
    for t in cones:
        print(f"  {t.Name()}")

    waypoints = []
    edges = []

    for t in cones:
        name = t.Name()
        # World-space pose of the target
        world_pose = t.PoseAbs()
        app_pose   = approach_pose(world_pose, args.approach_mm)

        x,  y,  z,  rx,  ry,  rz  = pose_to_xyzrpw(world_pose)
        ax, ay, az, arx, ary, arz = pose_to_xyzrpw(app_pose)

        approach_name = f"{name}_approach"

        waypoints.append({
            "name": approach_name,
            "x": ax, "y": ay, "z": az,
            "rx": arx, "ry": ary, "rz": arz,
            "move_type": "MoveJ",
            "note": "machine cone approach",
        })
        waypoints.append({
            "name": name,
            "x": x, "y": y, "z": z,
            "rx": rx, "ry": ry, "rz": rz,
            "move_type": "MoveL",
            "note": "machine cone place",
        })

        # Bidirectional edges
        edges.append({"from": approach_name, "to": name})
        edges.append({"from": name,          "to": approach_name})

        print(f"  {name}: grab=({x:.1f},{y:.1f},{z:.1f})  approach=({ax:.1f},{ay:.1f},{az:.1f})")

    write_yaml(waypoints, edges, args.output)
    print(f"\n[OK] {len(waypoints)} waypoints, {len(edges)} edges -> {args.output}")


if __name__ == "__main__":
    main()
