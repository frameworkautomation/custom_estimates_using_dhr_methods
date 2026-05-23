"""
tests/test_waypoint_ik_utils.py

Unit tests for waypoint_ik_utils. No RoboDK needed.
Run with: conda activate cone_planner && pytest tests/test_waypoint_ik_utils.py -v
"""

import math
import pytest
from robodk_code.waypoint_ik_utils import build_pose, bfs_solve_order, joint_distance


# ── build_pose ────────────────────────────────────────────────────────────────

def test_build_pose_identity():
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
    """90 deg about Z: X-axis of frame points in world +Y."""
    wp = {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 90.0}
    m = build_pose(wp)
    assert abs(m[0][0] - 0.0) < 1e-9
    assert abs(m[1][0] - 1.0) < 1e-9
    assert abs(m[2][0] - 0.0) < 1e-9


def test_build_pose_rx_90():
    """90 deg about X: Y-axis of frame points in world +Z."""
    wp = {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 90.0, "ry": 0.0, "rz": 0.0}
    m = build_pose(wp)
    assert abs(m[0][1] - 0.0) < 1e-9
    assert abs(m[1][1] - 0.0) < 1e-9
    assert abs(m[2][1] - 1.0) < 1e-9


def test_build_pose_defaults_missing_rotation():
    """Missing rx/ry/rz default to 0 — no KeyError."""
    wp = {"x": 1.0, "y": 2.0, "z": 3.0}
    m = build_pose(wp)
    assert abs(m[0][0] - 1.0) < 1e-9


def test_build_pose_last_row():
    """Last row is always [0, 0, 0, 1]."""
    wp = {"x": 5.0, "y": 10.0, "z": 15.0, "rx": 30.0, "ry": 45.0, "rz": 60.0}
    m = build_pose(wp)
    assert m[3][0] == 0 and m[3][1] == 0 and m[3][2] == 0 and m[3][3] == 1


def test_build_pose_rotation_matrix_orthonormal():
    """Rotation block columns should be unit vectors and orthogonal."""
    wp = {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 30.0, "ry": 45.0, "rz": 60.0}
    m = build_pose(wp)
    cols = [[m[r][c] for r in range(3)] for c in range(3)]
    for col in cols:
        norm = sum(v**2 for v in col) ** 0.5
        assert abs(norm - 1.0) < 1e-9, f"Column not unit: {col}"
    # Columns 0 and 1 should be orthogonal
    dot = sum(cols[0][i] * cols[1][i] for i in range(3))
    assert abs(dot) < 1e-9


# ── bfs_solve_order ───────────────────────────────────────────────────────────

def test_bfs_direct_neighbours_come_first():
    edges = [
        {"from": "home", "to": "A"},
        {"from": "home", "to": "B"},
        {"from": "A",    "to": "C"},
    ]
    order = bfs_solve_order({"home"}, edges)
    names = [n for n, _ in order]
    assert names.index("A") < names.index("C")
    assert names.index("B") < names.index("C")


def test_bfs_parent_is_nearest_seed():
    edges = [
        {"from": "home", "to": "A"},
        {"from": "A",    "to": "B"},
    ]
    order = bfs_solve_order({"home"}, edges)
    parent_of = {name: parent for name, parent in order}
    assert parent_of["A"] == "home"
    assert parent_of["B"] == "A"


def test_bfs_unreachable_not_included():
    edges = [
        {"from": "home", "to": "A"},
        {"from": "B",    "to": "C"},   # disconnected island
    ]
    order = bfs_solve_order({"home"}, edges)
    names = [n for n, _ in order]
    assert "B" not in names
    assert "C" not in names


def test_bfs_multiple_seeds():
    edges = [
        {"from": "home",      "to": "A"},
        {"from": "transport", "to": "B"},
        {"from": "A",         "to": "C"},
        {"from": "B",         "to": "C"},
    ]
    order = bfs_solve_order({"home", "transport"}, edges)
    names = [n for n, _ in order]
    # C is at distance 2 from both seeds; A and B must come before it
    assert "A" in names and "B" in names
    assert max(names.index("A"), names.index("B")) < names.index("C")


def test_bfs_seed_not_in_output():
    edges = [{"from": "home", "to": "A"}]
    order = bfs_solve_order({"home"}, edges)
    names = [n for n, _ in order]
    assert "home" not in names


def test_bfs_empty_seeds_returns_empty():
    edges = [{"from": "A", "to": "B"}]
    order = bfs_solve_order(set(), edges)
    assert order == []


def test_bfs_empty_edges_returns_empty():
    order = bfs_solve_order({"home"}, [])
    assert order == []


# ── joint_distance ────────────────────────────────────────────────────────────

def test_joint_distance_identical():
    assert joint_distance([1, 2, 3], [1, 2, 3]) == 0.0


def test_joint_distance_known():
    # (3-1)^2 + (4-2)^2 + (5-3)^2 = 4+4+4 = 12
    assert joint_distance([1, 2, 3], [3, 4, 5]) == 12.0


def test_joint_distance_symmetric():
    j1 = [10.0, -20.0, 5.0]
    j2 = [0.0, 0.0, 0.0]
    assert joint_distance(j1, j2) == joint_distance(j2, j1)
