"""
waypoint_ik_utils.py

Pure-Python utilities for waypoint IK solving.
No RoboDK import — fully testable in the cone_planner conda env.
"""

import math
import os
import re
import yaml
from collections import deque


# ── Pose building ─────────────────────────────────────────────────────────────

def build_pose(wp: dict) -> list:
    """Build a 4x4 homogeneous matrix from a waypoint dict.

    Uses ZYX Euler convention: R = Rz * Ry * Rx (same as GH export scripts).
    Returns list-of-lists suitable for robomath.Mat().

    wp must have keys: x, y, z. rx/ry/rz default to 0.0 if absent.
    """
    x, y, z = float(wp["x"]), float(wp["y"]), float(wp["z"])
    rx = math.radians(float(wp.get("rx", 0.0)))
    ry = math.radians(float(wp.get("ry", 0.0)))
    rz = math.radians(float(wp.get("rz", 0.0)))
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [
        [cy*cz,  cz*sx*sy - cx*sz,  cx*cz*sy + sx*sz,  x],
        [cy*sz,  cx*cz + sx*sy*sz,  cx*sy*sz - cz*sx,  y],
        [-sy,    cy*sx,             cx*cy,              z],
        [0,      0,                 0,                  1],
    ]


# ── Graph / BFS ───────────────────────────────────────────────────────────────

def bfs_solve_order(seed_names: set, edges: list) -> list:
    """BFS from seed_names through undirected edge graph.

    Returns a list of (waypoint_name, parent_name) tuples in BFS order.
    parent_name is the BFS parent — guaranteed to be a seed or to appear
    earlier in the returned list (so it will be solved before this node).
    Waypoints unreachable from seeds are not included.

    seed_names: names of waypoints that already have joints (starting nodes).
    edges: list of dicts with 'from' and 'to' keys.
    """
    graph = {}
    for e in edges:
        frm, to = e.get("from", ""), e.get("to", "")
        graph.setdefault(frm, set()).add(to)
        graph.setdefault(to, set()).add(frm)

    visited = set(seed_names)
    queue = deque()
    order = []

    for seed in sorted(seed_names):
        for neighbour in sorted(graph.get(seed, [])):
            if neighbour not in visited:
                visited.add(neighbour)
                queue.append((neighbour, seed))

    while queue:
        name, parent = queue.popleft()
        order.append((name, parent))
        for neighbour in sorted(graph.get(name, [])):
            if neighbour not in visited:
                visited.add(neighbour)
                queue.append((neighbour, name))

    return order


# ── Joint distance ────────────────────────────────────────────────────────────

def joint_distance(j1: list, j2: list) -> float:
    """Sum of squared joint differences (degrees). Lower = more similar config.

    No RoboDK equivalent exists — robomath.distance() is Cartesian only.
    """
    return sum((a - b) ** 2 for a, b in zip(j1, j2))


# ── YAML I/O ──────────────────────────────────────────────────────────────────

# ── Config loaders ────────────────────────────────────────────────────────────

def load_path_config(path_config_path: str) -> tuple:
    """Load path_config.yaml. Returns (waypoints: list[dict], edges: list[dict]).

    path_config.yaml stores waypoints as a dict (name → attrs). This converts
    them to the same list-of-dicts format used by all_waypoints.yaml so the
    IK solver can work with either file transparently.
    Joints-only waypoints (home, transport) are included — the solver skips them
    since they already have joints.
    """
    with open(path_config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    wp_dict = data.get("waypoints") or {}
    waypoints = []
    for name, attrs in wp_dict.items():
        if not isinstance(attrs, dict):
            continue
        wp = {"name": name}
        wp.update(attrs)
        waypoints.append(wp)
    edges = data.get("edges") or []
    return waypoints, edges


def save_path_config_ik_results(path_config_path: str, waypoints: list) -> None:
    """Write IK results (joints, ik_collision_verified, reachable, note) back to
    path_config.yaml without destroying its structure or comments.

    For each waypoint that has joints in the in-memory list, finds its name block
    in the YAML text and inserts/replaces the IK fields. Uses text manipulation to
    preserve all comments and human-written structure.
    """
    with open(path_config_path, "r", encoding="utf-8") as f:
        text = f.read()

    for wp in waypoints:
        name = wp.get("name", "")
        # Only process waypoints that the solver actually touched
        has_ik = "joints" in wp or "reachable" in wp
        if not has_ik:
            continue

        # Build the IK field lines to insert/replace
        ik_lines = []
        if "joints" in wp:
            joints_str = "[" + ", ".join(f"{j:.4f}" for j in wp["joints"]) + "]"
            ik_lines.append(f"    joints: {joints_str}")
        if "ik_collision_verified" in wp:
            ik_lines.append(f"    ik_collision_verified: {str(wp['ik_collision_verified']).lower()}")
        if wp.get("reachable") is False:
            ik_lines.append(f"    reachable: false")
        if "note" in wp:
            ik_lines.append(f'    note: "{wp["note"]}"')

        if not ik_lines:
            continue

        ik_block = "\n".join(ik_lines) + "\n"

        # Find the waypoint block: `  name:\n` followed by indented fields
        # Strategy: locate `  name:\n`, then find where the next same-level key starts
        import re as _re
        # Match the waypoint header line
        header_pat = _re.compile(r"^  " + _re.escape(name) + r":\n", _re.MULTILINE)
        m = header_pat.search(text)
        if not m:
            continue

        block_start = m.end()  # character after the `  name:\n` line

        # Find where this block ends: next line starting with non-whitespace or `  \w`
        # (i.e. another top-level or same-level key, or end of file)
        next_key_pat = _re.compile(r"^(?:  \S|\S)", _re.MULTILINE)
        m2 = next_key_pat.search(text, block_start)
        block_end = m2.start() if m2 else len(text)

        # Extract current block content, strip existing IK fields
        block_content = text[block_start:block_end]
        for field in ("joints", "ik_collision_verified", "reachable", "note"):
            block_content = _re.sub(
                r"    " + field + r":.*\n", "", block_content
            )

        # Append IK fields at end of block
        text = text[:block_start] + block_content.rstrip("\n") + "\n" + ik_block + text[block_end:]

    with open(path_config_path, "w", encoding="utf-8") as f:
        f.write(text)


def load_home_joints(path_config_path: str) -> list:
    """Return home joints from path_config.yaml as a fallback arm-config seed.

    Used when no BFS parent has joints yet (first run, cold start).
    Falls back to all-zeros if home is not defined.
    """
    with open(path_config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    waypoints = config.get("waypoints") or {}
    home = waypoints.get("home", {})
    joints = home.get("joints")
    if isinstance(joints, list):
        return [float(j) for j in joints]
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
