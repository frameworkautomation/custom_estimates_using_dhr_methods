"""
tests/test_yaml_reader.py

Testbed for robodk_yaml_reader.py and the edge building functions.
No RoboDK or Rhino needed.

If the knitwear-cell YAML is not present (clones/ not set up), all
tests that need it are skipped automatically.
"""

import math
import os
import sys
import pytest

# Allow importing from misc_extraction_utils/
REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
MISC_DIR  = os.path.join(REPO_ROOT, "misc_extraction_utils")
sys.path.insert(0, MISC_DIR)

from robodk_yaml_reader import (
    _rot_zyx,
    _mul4,
    _identity,
    _pose_from_yaml,
    mat4_to_entry,
    load_all_frames,
    filter_frames,
    frames_by_name,
    DEFAULT_YAML_PATH,
)
from scrape_edges_to_json import (
    build_rail_edges,
    build_approach_edges,
    _machine_number,
)

YAML_PRESENT = os.path.isfile(DEFAULT_YAML_PATH)


# ── Matrix math unit tests ────────────────────────────────────────────────────

def test_identity():
    I = _identity()
    assert I[0][0] == 1
    assert I[1][1] == 1
    assert I[2][2] == 1
    assert I[3][3] == 1
    assert I[0][3] == 0


def test_rot_zyx_zero():
    R = _rot_zyx(0, 0, 0)
    assert abs(R[0][0] - 1) < 1e-9
    assert abs(R[1][1] - 1) < 1e-9
    assert abs(R[2][2] - 1) < 1e-9


def test_rot_zyx_90_z():
    """90-degree rotation about Z: x-axis maps to y-axis."""
    R = _rot_zyx(0, 0, 90)
    # Column 0 (what x-axis becomes) should be approx [0, 1, 0]
    assert abs(R[0][0]) < 1e-9   # x -> ~0
    assert abs(R[1][0] - 1) < 1e-9  # y -> ~1
    assert abs(R[2][0]) < 1e-9


def test_mul4_identity():
    I = _identity()
    A = [[1,2,3,4],[5,6,7,8],[9,10,11,12],[0,0,0,1]]
    result = _mul4(A, I)
    for i in range(4):
        for j in range(4):
            assert abs(result[i][j] - A[i][j]) < 1e-9


def test_mul4_translation():
    """Multiplying two pure translations should add them."""
    def T(tx, ty, tz):
        M = _identity()
        M[0][3] = tx
        M[1][3] = ty
        M[2][3] = tz
        return M

    result = _mul4(T(1, 2, 3), T(10, 20, 30))
    assert abs(result[0][3] - 11) < 1e-9
    assert abs(result[1][3] - 22) < 1e-9
    assert abs(result[2][3] - 33) < 1e-9


def test_pose_from_yaml_translation_only():
    p = _pose_from_yaml({"x": 100, "y": 200, "z": 300})
    assert p[0][3] == 100
    assert p[1][3] == 200
    assert p[2][3] == 300
    # No rotation: top-left 3x3 should be identity
    assert abs(p[0][0] - 1) < 1e-9


def test_pose_from_yaml_empty():
    p = _pose_from_yaml({})
    assert p[0][3] == 0
    assert p[0][0] == 1


def test_mat4_to_entry_round_trip():
    """Build a simple pose and check that mat4_to_entry extracts it correctly."""
    # Pure translation, no rotation
    M = _identity()
    M[0][3] = 500.0
    M[1][3] = 1000.0
    M[2][3] = 250.0
    entry = mat4_to_entry("TestFrame", M)
    assert entry["name"] == "TestFrame"
    assert abs(entry["x"] - 500.0) < 0.01
    assert abs(entry["y"] - 1000.0) < 0.01
    assert abs(entry["z"] - 250.0) < 0.01
    # Identity rotation -> xaxis = [1,0,0], yaxis = [0,1,0]
    assert abs(entry["xaxis"][0] - 1.0) < 1e-4
    assert abs(entry["yaxis"][1] - 1.0) < 1e-4


# ── Machine number extraction ─────────────────────────────────────────────────

@pytest.mark.parametrize("name, expected", [
    ("OptimizationApproachMachine1",  1),
    ("OptimizationApproachMachine26", 26),
    ("ApproachMachine3CurtainSafe",   3),
    ("NoNumber",                      -1),
])
def test_machine_number(name, expected):
    assert _machine_number(name) == expected


# ── Edge builder unit tests ───────────────────────────────────────────────────

def _make_rail(n, x):
    return {"name": f"OptimizationApproachMachine{n}", "x": x, "y": 0, "z": 0}

def _make_approach(n, x, y, z):
    return {"name": f"ApproachMachine{n}CurtainSafe", "x": x, "y": y, "z": z}


def test_build_rail_edges_order():
    """Edges should connect frames sorted by X, regardless of input order."""
    rail = [_make_rail(3, 300), _make_rail(1, 100), _make_rail(2, 200)]
    edges = build_rail_edges(rail)
    assert len(edges) == 2
    assert edges[0]["from"]["name"] == "OptimizationApproachMachine1"
    assert edges[0]["to"]["name"]   == "OptimizationApproachMachine2"
    assert edges[1]["from"]["name"] == "OptimizationApproachMachine2"
    assert edges[1]["to"]["name"]   == "OptimizationApproachMachine3"


def test_build_rail_edges_single():
    """Single rail point -> no edges."""
    edges = build_rail_edges([_make_rail(1, 0)])
    assert edges == []


def test_build_rail_edges_empty():
    assert build_rail_edges([]) == []


def test_build_approach_edges_matched():
    rail     = [_make_rail(1, 100), _make_rail(2, 200)]
    approach = [_make_approach(1, 150, 500, 900), _make_approach(2, 250, 500, 900)]
    edges = build_approach_edges(rail, approach)
    assert len(edges) == 2
    names = [e["name"] for e in edges]
    assert "OptimizationApproachMachine1 -> ApproachMachine1CurtainSafe" in names
    assert "OptimizationApproachMachine2 -> ApproachMachine2CurtainSafe" in names


def test_build_approach_edges_no_match():
    """Rail machine 5 with no CurtainSafe match -> no edge."""
    rail     = [_make_rail(5, 500)]
    approach = [_make_approach(99, 0, 0, 0)]
    edges = build_approach_edges(rail, approach)
    assert edges == []


def test_edge_from_to_coords():
    rail     = [_make_rail(1, 100)]
    approach = [_make_approach(1, 200, 300, 400)]
    edges = build_approach_edges(rail, approach)
    assert len(edges) == 1
    e = edges[0]
    assert e["from"]["x"] == 100
    assert e["to"]["x"]   == 200
    assert e["to"]["y"]   == 300
    assert e["to"]["z"]   == 400


# ── Integration tests (require knitwear-cell YAML) ────────────────────────────

@pytest.mark.skipif(not YAML_PRESENT, reason="knitwear-cell not cloned")
def test_load_all_frames_count():
    frames = load_all_frames()
    assert len(frames) > 100, "Expected many frames from full YAML"


@pytest.mark.skipif(not YAML_PRESENT, reason="knitwear-cell not cloned")
def test_rail_points_pattern():
    frames = load_all_frames()
    rail = filter_frames(frames, r"^OptimizationApproachMachine\d+$")
    assert len(rail) >= 26, f"Expected at least 26 machines, got {len(rail)}"
    # None should contain 'Shifted' or 'Oil'
    for e in rail:
        assert "Shifted" not in e["name"]
        assert "Oil" not in e["name"]


@pytest.mark.skipif(not YAML_PRESENT, reason="knitwear-cell not cloned")
def test_curtain_safe_pattern():
    frames = load_all_frames()
    approaches = filter_frames(frames, r"CurtainSafe$")
    assert len(approaches) >= 26


@pytest.mark.skipif(not YAML_PRESENT, reason="knitwear-cell not cloned")
def test_rail_edges_consecutive():
    """Rail edges should connect Machine1->Machine2->...->Machine26 by X order."""
    frames = load_all_frames()
    rail = filter_frames(frames, r"^OptimizationApproachMachine\d+$")
    edges = build_rail_edges(rail)
    # N rail points -> N-1 edges
    assert len(edges) == len(rail) - 1


@pytest.mark.skipif(not YAML_PRESENT, reason="knitwear-cell not cloned")
def test_approach_edges_match_machines():
    frames = load_all_frames()
    rail     = filter_frames(frames, r"^OptimizationApproachMachine\d+$")
    approach = filter_frames(frames, r"CurtainSafe$")
    edges = build_approach_edges(rail, approach)
    # Every rail machine should have a CurtainSafe counterpart
    assert len(edges) == len(rail)


@pytest.mark.skipif(not YAML_PRESENT, reason="knitwear-cell not cloned")
def test_frames_by_name_lookup():
    frames = load_all_frames()
    lookup = frames_by_name(frames)
    # Should be able to look up Machine1's optimization frame
    assert "OptimizationApproachMachine1" in lookup
    entry = lookup["OptimizationApproachMachine1"]
    assert "x" in entry and "y" in entry and "z" in entry


@pytest.mark.skipif(not YAML_PRESENT, reason="knitwear-cell not cloned")
def test_world_pose_is_not_zero():
    """OptimizationApproachMachine frames should have non-trivial world positions."""
    frames = load_all_frames()
    lookup = frames_by_name(frames)
    m1 = lookup.get("OptimizationApproachMachine1")
    assert m1 is not None
    # These frames mark machine positions -- world coords should be nonzero
    total = abs(m1["x"]) + abs(m1["y"]) + abs(m1["z"])
    assert total > 1.0, f"Expected nonzero world position, got {m1}"
