# Solve Waypoint IK Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** For every Cartesian waypoint in `all_waypoints.yaml`, solve IK and write the resulting joint values back, using BFS-nearest-solved-node as seed and static collision checking to select among candidate solutions.

**Architecture:** Pure-Python utilities (BFS, pose building, YAML I/O) live in `waypoint_ik_utils.py` and are fully unit-testable without RoboDK. The main script `solve_waypoint_ik.py` imports them, connects to RoboDK, and runs the solve loop. Results are written back to `all_waypoints.yaml` in-place.

**Workflow:** After any fresh GH export + `amalgamate_waypoints.py` run, re-run `solve_waypoint_ik.py` to re-enrich `all_waypoints.yaml` with joints. The file is Cartesian-only after amalgamation; joints are added by this script.

**Tech Stack:** Python 3, PyYAML, RoboDK Python API (`robodk.robolink`, `robodk.robomath`), pytest (cone_planner env for unit tests, robodk_v1 env for live run).

---

## File Map

| File | Status | Purpose |
|------|--------|---------|
| `robodk_code/waypoint_ik_utils.py` | **Create** | Pure functions: `build_pose`, `bfs_solve_order`, `joint_distance`, `load_all_waypoints`, `save_all_waypoints` |
| `robodk_code/solve_waypoint_ik.py` | **Create** | Main script: connects to RoboDK, runs BFS solve loop, writes joints back |
| `tests/test_waypoint_ik_utils.py` | **Create** | Unit tests for all pure functions |
| `robo_dk_output/waypoint_sources.json` | **Read only** | Provides path to `all_waypoints.yaml` |

---

## Background: How the Algorithm Works

**Bootstrap seeds** are waypoints that already have `joints:` — initially `home` and `transport` from `path_config.yaml`, plus any waypoints already solved in `all_waypoints.yaml`.

**BFS (Breadth-First Search)** expands outward from all seeds simultaneously through the edge graph in `all_waypoints.yaml`. Each edge counts as distance 1. A waypoint at distance N has its nearest seed N hops away. We process waypoints in BFS order (distance 1 first, then 2, etc.) so that by the time we solve a waypoint, its BFS parent has already been solved and its joints can be used as the IK seed.

**IK seed** = the joints of the BFS parent (nearest already-solved node). This biases the solver toward a similar arm configuration, producing consistent elbow-up/elbow-down choices across spatially nearby waypoints.

**j7 handling:**
- Waypoint has `j7:` field → pin j7 to that value using OptimAxes. Raise `RuntimeError` if no solution found (no silent skips).
- Waypoint has no `j7:` field → j7 is free. Use `SolveIK_All` at the seed's j7 position, sort candidates by joint-space distance to seed, pick first.

**Static collision check** — after finding an IK candidate, set robot to those joints and call `rdk.Collisions()`. If it returns 0, the pose is collision-free at rest. This is NOT a path check (no `MoveJ_Test`) — path collision testing happens later in `check_collision_free_paths.py`.

If all IK candidates collide: mark `reachable: false` and add a note in the YAML. Do not raise — continue to next waypoint.

---

## Task 1: Pure Utility Functions

**Files:**
- Create: `robodk_code/waypoint_ik_utils.py`

- [ ] **Step 1: Create `robodk_code/waypoint_ik_utils.py`**

```python
"""
waypoint_ik_utils.py

Pure-Python utilities for waypoint IK solving.
No RoboDK import — fully testable in the cone_planner conda env.
"""

import math
import os
import yaml
from collections import deque


# ── Pose building ─────────────────────────────────────────────────────────────

def build_pose(wp: dict):
    """Build a 4x4 homogeneous matrix from a waypoint dict.

    Uses ZYX Euler convention: R = Rz * Ry * Rx (same as GH export scripts).
    Returns a list-of-lists [[r00,r01,r02,tx],[r10,r11,r12,ty],[r20,r21,r22,tz],[0,0,0,1]]
    suitable for passing to robomath.Mat().

    wp must have keys: x, y, z, rx, ry, rz (degrees).
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
    """BFS from seed_names through edge graph.

    Each edge entry is a dict with 'from' and 'to' keys.
    Edges are treated as undirected for reachability.

    Returns a list of (waypoint_name, parent_name) tuples in BFS order.
    parent_name is the BFS parent — guaranteed to be in seed_names or
    already returned earlier in the list (i.e. will be solved before this node).
    Waypoints unreachable from seeds are not included.
    """
    graph = {}  # name -> set of neighbours
    for e in edges:
        frm, to = e.get("from", ""), e.get("to", "")
        graph.setdefault(frm, set()).add(to)
        graph.setdefault(to, set()).add(frm)

    visited = set(seed_names)
    queue = deque()
    order = []  # list of (name, parent_name)

    for seed in sorted(seed_names):  # sorted for determinism
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
    """Sum of squared joint differences (in degrees). Lower = more similar."""
    return sum((a - b) ** 2 for a, b in zip(j1, j2))


# ── YAML I/O ──────────────────────────────────────────────────────────────────

def load_all_waypoints(yaml_path: str) -> tuple:
    """Load all_waypoints.yaml. Returns (waypoints: list[dict], edges: list[dict])."""
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data.get("waypoints") or [], data.get("edges") or []


def save_all_waypoints(yaml_path: str, waypoints: list, edges: list) -> None:
    """Write waypoints and edges back to YAML.

    Uses a line-by-line writer to preserve the project's existing format
    (avoids PyYAML's default flow-style quoting of field names).
    """
    lines = ["waypoints:"]
    for w in waypoints:
        lines.append(f"  - name: {w['name']}")
        for key in ("x", "y", "z", "rx", "ry", "rz"):
            if key in w:
                lines.append(f"    {key}: {w[key]}")
        for key in ("frame", "move_type"):
            if key in w:
                lines.append(f"    {key}: {w[key]}")
        if "j7" in w:
            lines.append(f"    j7: {w['j7']}")
        if "source" in w:
            lines.append(f"    source: {w['source']}")
        if "joints" in w:
            joints_str = "[" + ", ".join(f"{j:.4f}" for j in w["joints"]) + "]"
            lines.append(f"    joints: {joints_str}")
        if "ik_collision_verified" in w:
            lines.append(f"    ik_collision_verified: {str(w['ik_collision_verified']).lower()}")
        if w.get("reachable") is False:
            lines.append(f"    reachable: false")
        if "note" in w:
            lines.append(f"    note: \"{w['note']}\"")

    lines.append("")
    lines.append("edges:")
    for e in edges:
        lines.append(f"  - from: {e['from']}")
        lines.append(f"    to:   {e['to']}")
        tested = e.get("tested")
        lines.append(f"    tested: {tested if tested is not None else 'null'}")

    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


# ── Config loader ─────────────────────────────────────────────────────────────

def resolve_output_yaml(repo_root: str) -> str:
    """Read waypoint_sources.json and return the absolute path to all_waypoints.yaml."""
    import json
    config_path = os.path.join(repo_root, "robo_dk_output", "waypoint_sources.json")
    with open(config_path, "r", encoding="utf-8") as f:
        config = json.load(f)
    rel = config["output"].replace("/", os.sep)
    return os.path.join(repo_root, rel)


def load_bootstrap_seeds(path_config_path: str) -> dict:
    """Load home and transport joints from path_config.yaml as bootstrap seeds.

    Returns {name: [j1..j7]} for all waypoints in path_config that have joints.
    """
    with open(path_config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)
    seeds = {}
    for name, wp in (config.get("waypoints") or {}).items():
        if isinstance(wp, dict) and isinstance(wp.get("joints"), list):
            seeds[name] = [float(j) for j in wp["joints"]]
    return seeds
```

- [ ] **Step 2: Commit**

```bash
git add robodk_code/waypoint_ik_utils.py
git commit -m "feat: add waypoint_ik_utils — pure BFS, pose, YAML helpers"
```

---

## Task 2: Unit Tests

**Files:**
- Create: `tests/test_waypoint_ik_utils.py`

- [ ] **Step 1: Write tests**

```python
"""
tests/test_waypoint_ik_utils.py

Unit tests for waypoint_ik_utils. No RoboDK needed — run with cone_planner env.
    conda activate cone_planner && pytest tests/test_waypoint_ik_utils.py -v
"""

import math
import pytest
from robodk_code.waypoint_ik_utils import build_pose, bfs_solve_order, joint_distance


# ── build_pose ────────────────────────────────────────────────────────────────

def test_build_pose_identity():
    """Zero rotation → identity rotation block, translation in last column."""
    wp = {"x": 100.0, "y": 200.0, "z": 300.0, "rx": 0.0, "ry": 0.0, "rz": 0.0}
    m = build_pose(wp)
    assert abs(m[0][0] - 1.0) < 1e-9
    assert abs(m[1][1] - 1.0) < 1e-9
    assert abs(m[2][2] - 1.0) < 1e-9
    assert abs(m[0][3] - 100.0) < 1e-9
    assert abs(m[1][3] - 200.0) < 1e-9
    assert abs(m[2][3] - 300.0) < 1e-9
    assert m[3] == [0, 0, 0, 1]


def test_build_pose_rz_90():
    """90 deg rotation about Z: X->Y, Y->-X."""
    wp = {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 90.0}
    m = build_pose(wp)
    # First column (X axis) should point in Y direction
    assert abs(m[0][0] - 0.0) < 1e-9   # cos90
    assert abs(m[1][0] - 1.0) < 1e-9   # sin90
    assert abs(m[2][0] - 0.0) < 1e-9


def test_build_pose_rx_90():
    """90 deg rotation about X: Y->Z, Z->-Y."""
    wp = {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 90.0, "ry": 0.0, "rz": 0.0}
    m = build_pose(wp)
    # Second column (Y axis of frame) should point in Z direction
    assert abs(m[0][1] - 0.0) < 1e-9
    assert abs(m[1][1] - 0.0) < 1e-9   # cos90
    assert abs(m[2][1] - 1.0) < 1e-9   # sin90


def test_build_pose_defaults_missing_rotation():
    """Missing rx/ry/rz default to 0."""
    wp = {"x": 1.0, "y": 2.0, "z": 3.0}
    m = build_pose(wp)
    assert abs(m[0][0] - 1.0) < 1e-9


# ── bfs_solve_order ───────────────────────────────────────────────────────────

def test_bfs_direct_neighbours():
    """Nodes directly connected to seed are returned first, distance=1."""
    edges = [
        {"from": "home", "to": "A"},
        {"from": "home", "to": "B"},
        {"from": "A", "to": "C"},
    ]
    order = bfs_solve_order({"home"}, edges)
    names = [n for n, _ in order]
    # A and B before C
    assert names.index("A") < names.index("C")
    assert names.index("B") < names.index("C")


def test_bfs_parent_is_nearest_seed():
    """Parent reported for each node is the BFS parent, not the root seed."""
    edges = [
        {"from": "home", "to": "A"},
        {"from": "A", "to": "B"},
    ]
    order = bfs_solve_order({"home"}, edges)
    parent_of = {name: parent for name, parent in order}
    assert parent_of["A"] == "home"
    assert parent_of["B"] == "A"


def test_bfs_unreachable_not_included():
    """Node with no path to any seed is not included in output."""
    edges = [
        {"from": "home", "to": "A"},
        {"from": "B", "to": "C"},   # disconnected island
    ]
    order = bfs_solve_order({"home"}, edges)
    names = [n for n, _ in order]
    assert "B" not in names
    assert "C" not in names


def test_bfs_multiple_seeds():
    """BFS expands from all seeds simultaneously."""
    edges = [
        {"from": "home", "to": "A"},
        {"from": "transport", "to": "B"},
        {"from": "A", "to": "C"},
        {"from": "B", "to": "C"},
    ]
    order = bfs_solve_order({"home", "transport"}, edges)
    names = [n for n, _ in order]
    # C is distance 2 from both seeds — must come after A and B
    assert names.index("A") < names.index("C") or names.index("B") < names.index("C")


def test_bfs_seed_not_in_output():
    """Seed nodes themselves are not included in the returned order."""
    edges = [{"from": "home", "to": "A"}]
    order = bfs_solve_order({"home"}, edges)
    names = [n for n, _ in order]
    assert "home" not in names


def test_bfs_empty_edges():
    """No edges → empty order."""
    order = bfs_solve_order({"home"}, [])
    assert order == []


# ── joint_distance ────────────────────────────────────────────────────────────

def test_joint_distance_identical():
    assert joint_distance([1, 2, 3], [1, 2, 3]) == 0.0


def test_joint_distance_known():
    # (3-1)^2 + (4-2)^2 + (5-3)^2 = 4 + 4 + 4 = 12
    assert joint_distance([1, 2, 3], [3, 4, 5]) == 12.0


def test_joint_distance_symmetric():
    j1 = [10.0, -20.0, 5.0]
    j2 = [0.0, 0.0, 0.0]
    assert joint_distance(j1, j2) == joint_distance(j2, j1)
```

- [ ] **Step 2: Run tests — verify they pass**

```bash
source /home/samst/miniconda3/etc/profile.d/conda.sh && conda activate cone_planner
pytest tests/test_waypoint_ik_utils.py -v
```

Expected: all 12 tests PASS.

- [ ] **Step 3: Commit**

```bash
git add tests/test_waypoint_ik_utils.py
git commit -m "test: unit tests for waypoint_ik_utils"
```

---

## Task 3: Main IK Solver Script

**Files:**
- Create: `robodk_code/solve_waypoint_ik.py`

Requires RoboDK running. Run with `robodk_v1` conda env.

- [ ] **Step 1: Create `robodk_code/solve_waypoint_ik.py`**

```python
"""
solve_waypoint_ik.py

For every Cartesian waypoint in all_waypoints.yaml that lacks joints:,
solve IK using the BFS-nearest already-solved waypoint as seed.
Writes joints and ik_collision_verified back to all_waypoints.yaml.

Requires RoboDK running with the station loaded.

Usage:
    python robodk_code/solve_waypoint_ik.py
    python robodk_code/solve_waypoint_ik.py --robodk-ip 172.23.208.1
    python robodk_code/solve_waypoint_ik.py --dry-run   # solve but don't write

Workflow:
    1. Export from GH -> base_cone_waypoints.yaml, machine_cone_waypoints.yaml
    2. python robodk_code/amalgamate_waypoints.py
    3. python robodk_code/solve_waypoint_ik.py         <- this script
    4. python robodk_code/check_collision_free_paths.py
"""

import sys
import os
import argparse
import yaml

sys.path.append("C:/RoboDK/Python")

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, REPO_ROOT)

from robodk_code.waypoint_ik_utils import (
    build_pose,
    bfs_solve_order,
    joint_distance,
    load_all_waypoints,
    save_all_waypoints,
    resolve_output_yaml,
    load_bootstrap_seeds,
)

ROBOT_NAME = "Fanuc R2000iC 125L"
PATH_CONFIG = os.path.join(REPO_ROOT, "robo_dk_output", "path_config.yaml")

# OptimAxes config for j7-pinned IK (same pattern as check_base_cone_reachability.py)
OPT_AXES_PIN_J7 = {
    "AbsJnt_7": 0, "AbsOn_7": 1, "AbsW_7": 1000,
    "Algorithm": 3, "MaxIter": 500, "Tol": 0.001,
    "RelOn_1..7": 1, "RelW_1..7": 50,
}


def connect(ip: str):
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
    rdk = Robolink(ip)
    robot = rdk.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError(f"Robot '{ROBOT_NAME}' not found in station.")
    return rdk, robot


def solve_constrained_j7(robot, pose_mat, j7_value: float, seed_joints: list):
    """Solve IK with j7 pinned to j7_value using OptimAxes.

    Returns [j1..j7] rounded to 4dp, or raises RuntimeError if no solution.
    """
    from robodk.robomath import Mat
    seed = list(seed_joints)
    seed[6] = j7_value

    robot.setParam("OptimAxes", OPT_AXES_PIN_J7)
    robot.setJoints(seed)
    try:
        robot.MoveJ(Mat(pose_mat))
    except Exception as e:
        raise RuntimeError(f"OptimAxes MoveJ failed: {e}")
    finally:
        robot.setParam("OptimAxes", {})  # clear OptimAxes

    result = robot.Joints().list()
    if abs(result[6] - j7_value) > 2.0:  # sanity check: j7 stayed near target
        raise RuntimeError(
            f"j7 drifted to {result[6]:.2f} (target {j7_value}) — "
            "OptimAxes weight may be insufficient"
        )
    return [round(float(j), 4) for j in result]


def solve_free_j7(robot, pose_mat, seed_joints: list) -> list:
    """Solve IK with j7 free. Returns list of candidate joint lists sorted by
    joint-space distance to seed (closest first). Empty list if no solution.
    """
    from robodk.robomath import Mat
    robot.setJoints(seed_joints)   # sets j7 context for SolveIK_All

    all_sols_mat = robot.SolveIK_All(Mat(pose_mat))

    # SolveIK_All returns Mat[n_joints x n_solutions].
    # Extract each column as one solution.
    candidates = []
    try:
        n_rows = all_sols_mat.rows
        n_cols = all_sols_mat.cols
        for col in range(n_cols):
            sol = [round(float(all_sols_mat[row][col]), 4) for row in range(n_rows)]
            if len(sol) == 7:
                candidates.append(sol)
    except Exception:
        # Fallback: try flat list (single solution case)
        flat = all_sols_mat.list()
        if len(flat) == 7:
            candidates = [[round(float(j), 4) for j in flat]]

    candidates.sort(key=lambda j: joint_distance(j, seed_joints))
    return candidates


def check_static_collision(rdk, robot, joints: list) -> bool:
    """Set robot to joints and check for static collisions.
    Returns True if no collisions (clear), False if any collision detected.
    """
    robot.setJoints(joints)
    return rdk.Collisions() == 0


def main():
    parser = argparse.ArgumentParser(description="Solve IK for all Cartesian waypoints in all_waypoints.yaml")
    parser.add_argument("--robodk-ip", default="localhost")
    parser.add_argument("--dry-run", action="store_true",
                        help="Solve but do not write results to YAML")
    parser.add_argument("--force", action="store_true",
                        help="Re-solve waypoints that already have joints")
    args = parser.parse_args()

    yaml_path = resolve_output_yaml(REPO_ROOT)
    print(f"Loading: {yaml_path}")
    waypoints, edges = load_all_waypoints(yaml_path)

    # Index waypoints by name for fast lookup
    wp_by_name = {w["name"]: w for w in waypoints}

    # Bootstrap: collect all already-solved waypoints as seeds
    # Priority: path_config.yaml (home, transport) + waypoints already in all_waypoints.yaml with joints
    seeds = load_bootstrap_seeds(PATH_CONFIG)
    print(f"Bootstrap seeds from path_config.yaml: {sorted(seeds.keys())}")

    for wp in waypoints:
        if isinstance(wp.get("joints"), list) and not args.force:
            seeds[wp["name"]] = [float(j) for j in wp["joints"]]

    print(f"Total seeds (including pre-solved in yaml): {len(seeds)}")

    # BFS solve order — only process waypoints that exist in all_waypoints.yaml
    all_names_in_yaml = {w["name"] for w in waypoints}
    valid_seeds = {n for n in seeds if n in all_names_in_yaml or n in ("home", "transport")}

    bfs_order = bfs_solve_order(valid_seeds, edges)
    to_solve = [
        (name, parent)
        for name, parent in bfs_order
        if name in wp_by_name and (args.force or not isinstance(wp_by_name[name].get("joints"), list))
    ]

    print(f"\nWaypoints to solve: {len(to_solve)} / {len(waypoints)} total")
    if not to_solve:
        print("Nothing to solve — all waypoints already have joints.")
        return

    rdk, robot = connect(args.robodk_ip)
    print(f"Connected: {robot.Name()}")

    # Set world frame (required for Cartesian IK — same pattern as moving_a_cone.py).
    # Verify the world frame name matches your station by checking check_base_cone_reachability.py.
    # Typically: rdk.Item("World") or rdk.Item("") with no type filter.
    from robodk.robolink import ITEM_TYPE_FRAME
    world_frame = rdk.Item("World")  # adjust name if station uses a different name
    original_frame = robot.PoseFrame()
    robot.setPoseFrame(world_frame)

    n_solved = 0
    n_unreachable = 0
    n_error = 0

    try:
        for wp_name, parent_name in to_solve:
            wp = wp_by_name[wp_name]

            # Get seed joints: prefer immediate BFS parent, fall back to any seed
            seed_joints = seeds.get(parent_name) or seeds.get("transport") or seeds.get("home")
            if seed_joints is None:
                print(f"  [SKIP] {wp_name}: no seed joints available (parent '{parent_name}' not solved)")
                n_error += 1
                continue

            pose_mat = build_pose(wp)

            has_j7 = "j7" in wp and wp["j7"] is not None

            if has_j7:
                # j7 constrained: must use OptimAxes, raise on failure
                j7_val = float(wp["j7"])
                try:
                    joints = solve_constrained_j7(robot, pose_mat, j7_val, seed_joints)
                except RuntimeError as e:
                    print(f"  [ERROR] {wp_name}: j7={j7_val} constrained IK failed — {e}")
                    raise  # j7-constrained failure is always an error, not a soft skip

                # Static collision check
                if check_static_collision(rdk, robot, joints):
                    wp["joints"] = joints
                    wp["ik_collision_verified"] = True
                    seeds[wp_name] = joints
                    n_solved += 1
                    print(f"  [OK]    {wp_name}  j7={j7_val:.1f} (constrained)")
                else:
                    # j7-constrained, but solution collides — raise, as this is unexpected
                    raise RuntimeError(
                        f"{wp_name}: j7-constrained IK solution collides. "
                        "Check station geometry or waypoint position."
                    )

            else:
                # j7 free: try each candidate, pick first collision-free
                candidates = solve_free_j7(robot, pose_mat, seed_joints)
                accepted = None
                for candidate in candidates:
                    if check_static_collision(rdk, robot, candidate):
                        accepted = candidate
                        break

                if accepted:
                    wp["joints"] = accepted
                    wp["ik_collision_verified"] = True
                    seeds[wp_name] = accepted
                    n_solved += 1
                    print(f"  [OK]    {wp_name}  (free j7={accepted[6]:.1f}, seed={parent_name})")
                else:
                    wp["reachable"] = False
                    wp["note"] = (
                        (wp.get("note", "") + " ").lstrip() +
                        "# TODO: all IK solutions have static collisions at this pose "
                        "-- investigate alternative approach or different tool configuration"
                    ).strip()
                    n_unreachable += 1
                    print(f"  [UNREACHABLE] {wp_name}: {len(candidates)} IK solutions all collide")

    finally:
        robot.setPoseFrame(original_frame)

    print(f"\n--- Summary ---")
    print(f"  Solved:      {n_solved}")
    print(f"  Unreachable: {n_unreachable}")
    print(f"  Errors:      {n_error}")
    print(f"  Already had joints (skipped): {len(waypoints) - len(to_solve)}")

    if not args.dry_run:
        save_all_waypoints(yaml_path, waypoints, edges)
        print(f"\n[OK] Written to {yaml_path}")
    else:
        print("\n[DRY RUN] Not writing to YAML.")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run a dry-run first to verify connections and BFS without writing**

With RoboDK open and station loaded:
```bash
source /home/samst/miniconda3/etc/profile.d/conda.sh && conda activate robodk_v1
python robodk_code/solve_waypoint_ik.py --robodk-ip 172.23.208.1 --dry-run
```

Expected output:
```
Loading: .../robo_dk_output/all_waypoints.yaml
Bootstrap seeds from path_config.yaml: ['home', 'transport']
Total seeds (including pre-solved in yaml): 2
Waypoints to solve: N / M total
Connected: Fanuc R2000iC 125L
  [OK]    base_cone_grab_0_approach  j7=0.0 (constrained)
  ...
--- Summary ---
  Solved:      ...
  Unreachable: 0
[DRY RUN] Not writing to YAML.
```

If you see `[ERROR]` on j7-constrained waypoints, the station or waypoint poses need investigation before proceeding.

- [ ] **Step 3: Run for real**

```bash
python robodk_code/solve_waypoint_ik.py --robodk-ip 172.23.208.1
```

Expected: `[OK] Written to .../all_waypoints.yaml`. Open the file and verify a few waypoints now have `joints:` and `ik_collision_verified: true`.

- [ ] **Step 4: Verify with batcat**

```bash
batcat robo_dk_output/all_waypoints.yaml | head -60
```

You should see entries like:
```yaml
  - name: base_cone_grab_0_approach
    x: 123.4
    ...
    j7: 0.0
    source: grasshopper
    joints: [0.0000, -45.0000, 20.0000, 0.0000, -10.0000, -90.0000, 0.0000]
    ik_collision_verified: true
```

- [ ] **Step 5: Commit**

```bash
git add robodk_code/solve_waypoint_ik.py
git add -f robo_dk_output/all_waypoints.yaml
git commit -m "feat: solve_waypoint_ik — BFS-seeded IK with static collision check"
```

---

## Task 4: Run Full Test Suite

- [ ] **Step 1: Run all unit tests**

```bash
source /home/samst/miniconda3/etc/profile.d/conda.sh && conda activate cone_planner
pytest tests/ -v
```

Expected: all tests pass (32 existing + 12 new = 44 total).

- [ ] **Step 2: Commit if any fixes needed**

```bash
git add -A
git commit -m "fix: adjust after full test run"
```

---

## Notes

**`all_waypoints.yaml` is regenerated by `amalgamate_waypoints.py`** — running amalgamate again will overwrite the joints written by this script. Always re-run `solve_waypoint_ik.py` after any fresh GH export + amalgamation.

**Path collision verification comes later** — `ik_collision_verified: true` means the robot can reach the pose statically without hitting anything. Whether it can travel *to* that pose safely is tested by `check_collision_free_paths.py` (Phase 5 of the build plan).

**Unreachable waypoints** — if a waypoint is marked `reachable: false`, check:
1. Is the Cartesian pose inside the robot's workspace at j7=0 (for base cones)?
2. Is the tool colliding with a fixture at that pose?
3. Are the x/y/z values correct (world-space vs robot-local confusion)?
