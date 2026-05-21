"""
robodk_yaml_reader.py

Core library: parse robodk.yaml and return world-space frame poses.
No RoboDK connection needed; pure Python matrix math.

Importable from any script. Also used by scrape_robodk_frames.py and
scrape_edges_to_json.py.
"""

import math
import os
import re

import yaml


# ── default YAML path ─────────────────────────────────────────────────────────

DEFAULT_YAML_PATH = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "clones", "knitwear-cell", "src", "main", "config", "robodk.yaml",
))


# ── Transform helpers ─────────────────────────────────────────────────────────

def _deg2rad(d):
    return d * math.pi / 180.0


def _rot_zyx(rx_deg, ry_deg, rz_deg):
    """3x3 rotation matrix from ZYX Euler angles (degrees).  R = Rz * Ry * Rx."""
    cx, sx = math.cos(_deg2rad(rx_deg)), math.sin(_deg2rad(rx_deg))
    cy, sy = math.cos(_deg2rad(ry_deg)), math.sin(_deg2rad(ry_deg))
    cz, sz = math.cos(_deg2rad(rz_deg)), math.sin(_deg2rad(rz_deg))
    return [
        [cz*cy,  cz*sy*sx - sz*cx,  cz*sy*cx + sz*sx],
        [sz*cy,  sz*sy*sx + cz*cx,  sz*sy*cx - cz*sx],
        [-sy,    cy*sx,             cy*cx            ],
    ]


def _mat4(R, t):
    return [
        [R[0][0], R[0][1], R[0][2], t[0]],
        [R[1][0], R[1][1], R[1][2], t[1]],
        [R[2][0], R[2][1], R[2][2], t[2]],
        [0,       0,       0,       1   ],
    ]


def _identity():
    return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]


def _mul4(A, B):
    C = [[0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            for k in range(4):
                C[i][j] += A[i][k] * B[k][j]
    return C


def _pose_from_yaml(pose_dict):
    p = pose_dict or {}
    return _mat4(_rot_zyx(p.get("rx", 0.0), p.get("ry", 0.0), p.get("rz", 0.0)),
                 [p.get("x", 0.0), p.get("y", 0.0), p.get("z", 0.0)])


def _r(v, n=4):
    return round(v, n)


def mat4_to_entry(name, M):
    """Convert a 4x4 world-space matrix to a frame entry dict."""
    tx, ty, tz = M[0][3], M[1][3], M[2][3]
    xaxis = [M[0][0], M[1][0], M[2][0]]
    yaxis = [M[0][1], M[1][1], M[2][1]]
    zaxis = [M[0][2], M[1][2], M[2][2]]
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
    return {
        "name":  name,
        "x":     _r(tx),  "y":  _r(ty),  "z":  _r(tz),
        "rx":    _r(math.degrees(rx_r), 6),
        "ry":    _r(math.degrees(ry_r), 6),
        "rz":    _r(math.degrees(rz_r), 6),
        "xaxis": [_r(v, 6) for v in xaxis],
        "yaxis": [_r(v, 6) for v in yaxis],
        "zaxis": [_r(v, 6) for v in zaxis],
    }


# ── YAML tree walker ──────────────────────────────────────────────────────────

def _walk(node, parent_world, results):
    if not isinstance(node, dict):
        return
    name = node.get("name")
    local = _pose_from_yaml(node.get("pose"))
    world = _mul4(parent_world, local)
    if name:
        results.append({"name": name, "world": world})
    for key, val in node.items():
        if key in ("name", "pose", "filename", "type", "states"):
            continue
        if isinstance(val, list):
            for item in val:
                _walk(item, world, results)
        elif isinstance(val, dict):
            _walk(val, world, results)


# ── Public API ────────────────────────────────────────────────────────────────

def load_all_frames(yaml_path=None):
    """
    Parse robodk.yaml and return a list of frame entry dicts with world poses.

    Each entry: {name, x, y, z, rx, ry, rz, xaxis, yaxis, zaxis}
    """
    if yaml_path is None:
        yaml_path = DEFAULT_YAML_PATH
    with open(yaml_path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    raw = []
    root = _identity()
    if isinstance(data, dict):
        _walk(data, root, raw)
    elif isinstance(data, list):
        for item in data:
            _walk(item, root, raw)
    return [mat4_to_entry(r["name"], r["world"]) for r in raw]


def filter_frames(all_frames, pattern):
    """Return entries whose name matches the regex pattern."""
    regex = re.compile(pattern, re.IGNORECASE)
    return [e for e in all_frames if regex.search(e["name"])]


def frames_by_name(all_frames):
    """Return a dict mapping name -> entry for fast lookup."""
    return {e["name"]: e for e in all_frames}
