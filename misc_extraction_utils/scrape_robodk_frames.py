"""
scrape_robodk_frames.py

Reads DHR's robodk.yaml (no RoboDK connection needed) and extracts frame
poses by name pattern. Composes the full parent-chain transforms so output
positions are in world space, matching what RoboDK would show.

Two default groups:

  --rail-pattern   (default: CurtainSafe$)
      One frame per machine at curtain/door level (no Down variants).
      e.g. ApproachMachine1CurtainSafe ... ApproachMachine26CurtainSafe

  --approach-pattern   (default: CurtainSafe)
      All curtain-safe frames including Down variants.

Output: robo_dk_output/scraped_frames.json
        Keys: "rail_points" and "approach_frames" (or "all_frames" with --all-frames).
        Each entry: name, x, y, z, rx, ry, rz, xaxis, yaxis, zaxis.

Usage:
    python misc_extraction_utils/scrape_robodk_frames.py
    python misc_extraction_utils/scrape_robodk_frames.py --rail-pattern "OptimizationApproach"
    python misc_extraction_utils/scrape_robodk_frames.py --all-frames
    python misc_extraction_utils/scrape_robodk_frames.py --yaml path/to/other.yaml
"""

import os
import sys
import json
import math
import argparse
import re

import yaml

YAML_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "clones", "knitwear-cell", "src", "main", "config", "robodk.yaml",
)
OUTPUT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output", "scraped_frames.json"
)


# ── Transform helpers (pure Python, no robodk needed) ────────────────────────

def _deg2rad(d):
    return d * math.pi / 180.0


def _rot_zyx(rx_deg, ry_deg, rz_deg):
    """Build a 3x3 rotation matrix from ZYX Euler angles (degrees).
    Convention: R = Rz * Ry * Rx  (same as RoboDK TxyzRxyz).
    Returns a list-of-rows [[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]].
    """
    cx, sx = math.cos(_deg2rad(rx_deg)), math.sin(_deg2rad(rx_deg))
    cy, sy = math.cos(_deg2rad(ry_deg)), math.sin(_deg2rad(ry_deg))
    cz, sz = math.cos(_deg2rad(rz_deg)), math.sin(_deg2rad(rz_deg))

    return [
        [cz*cy,  cz*sy*sx - sz*cx,  cz*sy*cx + sz*sx],
        [sz*cy,  sz*sy*sx + cz*cx,  sz*sy*cx - cz*sx],
        [-sy,    cy*sx,             cy*cx            ],
    ]


def _mat4(R, t):
    """Pack a 3x3 rotation R and translation [tx,ty,tz] into a 4x4 homogeneous matrix."""
    return [
        [R[0][0], R[0][1], R[0][2], t[0]],
        [R[1][0], R[1][1], R[1][2], t[1]],
        [R[2][0], R[2][1], R[2][2], t[2]],
        [0,       0,       0,       1   ],
    ]


def _identity():
    return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]


def _mul4(A, B):
    """Multiply two 4x4 matrices."""
    C = [[0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            for k in range(4):
                C[i][j] += A[i][k] * B[k][j]
    return C


def _pose_from_yaml(pose_dict):
    """Convert a YAML pose dict {x,y,z,rx,ry,rz} to a 4x4 homogeneous matrix."""
    p = pose_dict or {}
    tx = p.get("x", 0.0)
    ty = p.get("y", 0.0)
    tz = p.get("z", 0.0)
    rx = p.get("rx", 0.0)
    ry = p.get("ry", 0.0)
    rz = p.get("rz", 0.0)
    return _mat4(_rot_zyx(rx, ry, rz), [tx, ty, tz])


def _mat4_to_entry(name, M):
    """Convert a 4x4 world-space matrix to an output dict."""
    # Extract translation
    tx, ty, tz = M[0][3], M[1][3], M[2][3]
    # Extract rotation matrix columns (xaxis, yaxis, zaxis)
    xaxis = [M[0][0], M[1][0], M[2][0]]
    yaxis = [M[0][1], M[1][1], M[2][1]]
    zaxis = [M[0][2], M[1][2], M[2][2]]
    # Recover Euler ZYX angles from rotation matrix (for reference)
    sy = -M[2][0]
    cy = math.sqrt(M[0][0]**2 + M[1][0]**2)
    if cy > 1e-6:
        rx_r = math.atan2(M[2][1], M[2][2])
        ry_r = math.atan2(sy, cy)
        rz_r = math.atan2(M[1][0], M[0][0])
    else:
        rx_r = math.atan2(-M[1][2], M[1][1])
        ry_r = math.atan2(sy, cy)
        rz_r = 0.0

    def r(v, n=4): return round(v, n)

    return {
        "name":  name,
        "x":     r(tx),  "y":  r(ty),  "z":  r(tz),
        "rx":    r(math.degrees(rx_r), 6),
        "ry":    r(math.degrees(ry_r), 6),
        "rz":    r(math.degrees(rz_r), 6),
        "xaxis": [r(v, 6) for v in xaxis],
        "yaxis": [r(v, 6) for v in yaxis],
        "zaxis": [r(v, 6) for v in zaxis],
    }


# ── YAML tree walker ──────────────────────────────────────────────────────────

def _walk(node, parent_world, results):
    """Recursively walk the YAML item tree.

    node        -- a dict that may have 'name', 'pose', 'children'
    parent_world -- 4x4 world transform of the parent
    results      -- list to append {name, world_matrix} dicts to
    """
    if not isinstance(node, dict):
        return

    name = node.get("name")
    local = _pose_from_yaml(node.get("pose"))
    world = _mul4(parent_world, local)

    if name:
        results.append({"name": name, "world": world})

    # Children can appear under any key that holds a list of dicts
    for key, val in node.items():
        if key in ("name", "pose", "filename", "type", "states"):
            continue
        if isinstance(val, list):
            for item in val:
                _walk(item, world, results)
        elif isinstance(val, dict):
            _walk(val, world, results)


def load_all_frames(yaml_path):
    """Parse robodk.yaml and return a list of {name, entry} dicts with world poses."""
    with open(yaml_path, encoding="utf-8") as f:
        data = yaml.safe_load(f)

    raw = []
    root = _identity()

    if isinstance(data, dict):
        _walk(data, root, raw)
    elif isinstance(data, list):
        for item in data:
            _walk(item, root, raw)

    return [_mat4_to_entry(r["name"], r["world"]) for r in raw]


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Extract frame world poses from robodk.yaml (no RoboDK needed)."
    )
    p.add_argument(
        "--yaml",
        default=YAML_PATH,
        help="Path to robodk.yaml (default: clones/knitwear-cell/src/main/config/robodk.yaml)",
    )
    p.add_argument(
        "--rail-pattern",
        default="CurtainSafe$",
        help="Regex for rail-point frames (default: 'CurtainSafe$')",
    )
    p.add_argument(
        "--approach-pattern",
        default="CurtainSafe",
        help="Regex for approach frames (default: 'CurtainSafe')",
    )
    p.add_argument(
        "--all-frames",
        action="store_true",
        help="Dump every frame into a single 'all_frames' key",
    )
    return p.parse_args()


def filter_frames(all_frames, pattern):
    regex = re.compile(pattern, re.IGNORECASE)
    return [e for e in all_frames if regex.search(e["name"])]


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
