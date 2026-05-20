"""
extract_waypoint_frames.py

Connects to a running RoboDK instance and extracts all frames whose names match
a given pattern (default: anything containing "curtain" or "safe", case-insensitive).

Outputs:
  - Console table of name / x / y / z / rx / ry / rz
  - robo_dk_output/waypoint_frames.json  — for Rhino/GhPython import
  - robo_dk_output/waypoint_frames.csv   — for spreadsheet inspection

Rhino import: in GhPython, json.load() the file and build a Plane from
  x, y, z + the rotation matrix columns (rx, ry, rz are Euler ZYX in degrees).

Usage:
    python robodk_code/extract_waypoint_frames.py
    python robodk_code/extract_waypoint_frames.py --pattern "CurtainSafe"
    python robodk_code/extract_waypoint_frames.py --pattern "transport"
"""

import sys
import os
import json
import csv
import argparse
import re

sys.path.append("C:/RoboDK/Python")

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output")


def parse_args():
    p = argparse.ArgumentParser(description="Extract named frames from RoboDK station.")
    p.add_argument(
        "--pattern",
        default="curtain|safe",
        help="Case-insensitive regex to match frame names (default: 'curtain|safe')",
    )
    p.add_argument(
        "--all-frames",
        action="store_true",
        help="Dump every frame in the station (ignores --pattern)",
    )
    return p.parse_args()


def connect_robodk():
    from robodk.robolink import Robolink, ITEM_TYPE_FRAME
    rdk = Robolink()
    try:
        rdk.Item("")  # ping
    except Exception as e:
        print(f"[ERROR] Cannot connect to RoboDK: {e}")
        sys.exit(1)
    print("[INFO] Connected to RoboDK")
    return rdk, ITEM_TYPE_FRAME


def pose_to_txyz_rxyz(pose):
    """Extract [x, y, z, rx_deg, ry_deg, rz_deg] from a RoboDK Mat (4x4 homogeneous)."""
    from robodk.robomath import Pose_2_TxyzRxyz
    import math
    vals = Pose_2_TxyzRxyz(pose)
    # vals = [x, y, z, rx_rad, ry_rad, rz_rad]
    return {
        "x":  round(vals[0], 4),
        "y":  round(vals[1], 4),
        "z":  round(vals[2], 4),
        "rx": round(math.degrees(vals[3]), 6),
        "ry": round(math.degrees(vals[4]), 6),
        "rz": round(math.degrees(vals[5]), 6),
    }


def rotation_columns(pose):
    """Return the three rotation-column vectors from the 4x4 pose matrix.

    Useful for building a Rhino Plane directly:
      xaxis = col 0 (X direction of the frame)
      yaxis = col 1 (Y direction of the frame)
      normal = col 2 (Z / normal direction of the frame)
    """
    mat = pose.rows
    return {
        "xaxis": [round(mat[0][0], 6), round(mat[1][0], 6), round(mat[2][0], 6)],
        "yaxis": [round(mat[0][1], 6), round(mat[1][1], 6), round(mat[2][1], 6)],
        "zaxis": [round(mat[0][2], 6), round(mat[1][2], 6), round(mat[2][2], 6)],
    }


def main():
    args = parse_args()
    rdk, ITEM_TYPE_FRAME = connect_robodk()

    pattern = None if args.all_frames else re.compile(args.pattern, re.IGNORECASE)

    frames = rdk.ItemList(ITEM_TYPE_FRAME)
    print(f"[INFO] Total frames in station: {len(frames)}")

    results = []
    for frame in frames:
        name = frame.Name()
        if pattern and not pattern.search(name):
            continue

        pose_abs = frame.PoseAbs()
        txyz = pose_to_txyz_rxyz(pose_abs)
        cols = rotation_columns(pose_abs)

        entry = {
            "name": name,
            **txyz,
            **cols,
        }
        results.append(entry)

    if not results:
        print(f"[WARN] No frames matched pattern '{args.pattern}'")
        print("       Try --all-frames to dump everything, or adjust --pattern")
        return

    # ── Console table ─────────────────────────────────────────────────────────
    col_w = max(len(r["name"]) for r in results) + 2
    header = f"{'Name':<{col_w}} {'x':>10} {'y':>10} {'z':>10} {'rx':>10} {'ry':>10} {'rz':>10}"
    print(f"\n{header}")
    print("-" * len(header))
    for r in results:
        print(
            f"{r['name']:<{col_w}} {r['x']:>10.2f} {r['y']:>10.2f} {r['z']:>10.2f}"
            f" {r['rx']:>10.4f} {r['ry']:>10.4f} {r['rz']:>10.4f}"
        )
    print(f"\n[INFO] {len(results)} frame(s) matched\n")

    # ── JSON output ───────────────────────────────────────────────────────────
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    json_path = os.path.join(OUTPUT_DIR, "waypoint_frames.json")
    with open(json_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"[INFO] JSON written to {json_path}")

    # ── CSV output ────────────────────────────────────────────────────────────
    csv_path = os.path.join(OUTPUT_DIR, "waypoint_frames.csv")
    fieldnames = ["name", "x", "y", "z", "rx", "ry", "rz", "xaxis", "yaxis", "zaxis"]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            writer.writerow({
                **r,
                "xaxis": r["xaxis"],
                "yaxis": r["yaxis"],
                "zaxis": r["zaxis"],
            })
    print(f"[INFO] CSV written to {csv_path}")

    print()
    print("Rhino / GhPython usage:")
    print("  import json, rhinoscriptsyntax as rs")
    print("  data = json.load(open('robo_dk_output/waypoint_frames.json'))")
    print("  for f in data:")
    print("      origin = rs.CreatePoint(f['x'], f['y'], f['z'])")
    print("      plane  = rs.PlaneFromNormal(origin, f['zaxis'], f['xaxis'])")


if __name__ == "__main__":
    main()
