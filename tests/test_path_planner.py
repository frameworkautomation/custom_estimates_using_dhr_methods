"""Tests for check_collision_free_paths.py — no RoboDK connection required."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import hashlib, json, tempfile
import pytest
import yaml
from robodk_code.check_collision_free_paths import (
    load_config,
    compute_config_hashes,
    find_shortest_path,
    find_gateways,
)

# ── Fixtures ──────────────────────────────────────────────────────────────────

def _write_config(data):
    f = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(data, f)
    f.close()
    return f.name

BASE_CONFIG = {
    "default_tool": "pickup_closed",
    "cone_mesh_template": "base_cone_0",
    "waypoints": {
        "home": {"joints": [0.0]*7},
        "transport": {"joints": [0.0, -55.0, 30.0, 0.0, -30.0, -90.0, 0.0]},
    },
    "routing_candidates": ["home", "transport"],
    "destination_groups": {
        "machine_1": {"cones": ["cone_grab_1", "cone_grab_2"]},
    },
    "collision_enable": [],
    "collision_disable": [],
}

# ── load_config ────────────────────────────────────────────────────────────────

def test_load_config_returns_dict():
    path = _write_config(BASE_CONFIG)
    cfg = load_config(path)
    assert isinstance(cfg, dict)
    assert "waypoints" in cfg
    os.unlink(path)


def test_load_config_missing_file_raises():
    with pytest.raises(SystemExit):
        load_config("/nonexistent/path_config.yaml")


# ── compute_config_hashes ──────────────────────────────────────────────────────

def test_collision_critical_hash_changes_on_collision_enable():
    cfg1 = dict(BASE_CONFIG)
    cfg2 = dict(BASE_CONFIG, collision_enable=[["robot", "wall"]])
    h1 = compute_config_hashes(cfg1)
    h2 = compute_config_hashes(cfg2)
    assert h1["collision_critical"] != h2["collision_critical"]


def test_collision_critical_hash_changes_on_mesh_template():
    cfg1 = dict(BASE_CONFIG)
    cfg2 = dict(BASE_CONFIG, cone_mesh_template="base_cone_1")
    h1 = compute_config_hashes(cfg1)
    h2 = compute_config_hashes(cfg2)
    assert h1["collision_critical"] != h2["collision_critical"]


def test_structural_hash_changes_on_waypoint_joints():
    import copy
    cfg1 = copy.deepcopy(BASE_CONFIG)
    cfg2 = copy.deepcopy(BASE_CONFIG)
    cfg2["waypoints"]["home"]["joints"] = [1.0]*7
    h1 = compute_config_hashes(cfg1)
    h2 = compute_config_hashes(cfg2)
    assert h1["structural"] != h2["structural"]


def test_structural_hash_unchanged_on_new_group_only():
    """Adding a destination group does not change the structural hash (additive change)."""
    import copy
    cfg1 = copy.deepcopy(BASE_CONFIG)
    cfg2 = copy.deepcopy(BASE_CONFIG)
    cfg2["destination_groups"]["machine_2"] = {"cones": ["cone_grab_9"]}
    h1 = compute_config_hashes(cfg1)
    h2 = compute_config_hashes(cfg2)
    # structural hash covers waypoints and routing_candidates, not groups
    assert h1["structural"] == h2["structural"]


# ── find_shortest_path ─────────────────────────────────────────────────────────

def test_find_shortest_path_direct():
    edges = {("home", "transport"), ("transport", "gateway_a")}
    result = find_shortest_path(edges, "home", "gateway_a")
    assert result == ["home", "transport", "gateway_a"]


def test_find_shortest_path_no_path():
    edges = {("home", "transport")}
    result = find_shortest_path(edges, "home", "gateway_a")
    assert result is None


def test_find_shortest_path_prefers_fewer_hops():
    # Direct: home→B→dest (2 hops) vs home→A→B→dest (3 hops)
    edges = {
        ("home", "A"), ("A", "B"), ("B", "dest"),
        ("home", "B"),  # shortcut
    }
    result = find_shortest_path(edges, "home", "dest")
    assert result == ["home", "B", "dest"]


def test_find_shortest_path_same_start_end():
    result = find_shortest_path(set(), "home", "home")
    assert result == ["home"]


# ── find_gateways ──────────────────────────────────────────────────────────────

def _make_edge_cache(pairs: dict) -> dict:
    """Helper: {edge_key: collision_free} → full edge_cache format."""
    return {k: {"collision_free": v, "from_joints": [0]*7, "to_joints": [1]*7}
            for k, v in pairs.items()}


def test_find_gateways_valid_gateway():
    edge_cache = _make_edge_cache({
        "transport|cone_grab_1_approach": True,
        "cone_grab_1_approach|transport": True,
        "cone_grab_1_approach|cone_grab_1_grab": True,
        "cone_grab_1_grab|cone_grab_1_approach": True,
        "transport|cone_grab_2_approach": True,
        "cone_grab_2_approach|transport": True,
        "cone_grab_2_approach|cone_grab_2_grab": True,
        "cone_grab_2_grab|cone_grab_2_approach": True,
    })
    groups = {"machine_1": {"cones": ["cone_grab_1", "cone_grab_2"]}}
    routing_candidates = ["transport"]
    result = find_gateways(edge_cache, groups, routing_candidates)
    assert "transport" in result["machine_1"]["gateways"]
    assert result["machine_1"]["cones"]["cone_grab_1"]["tested"] is True
    assert result["machine_1"]["cones"]["cone_grab_2"]["tested"] is True


def test_find_gateways_partial_gateway_excluded():
    """transport reaches cone_1 but has collision to cone_2 → not a valid gateway."""
    edge_cache = _make_edge_cache({
        "transport|cone_grab_1_approach": True,
        "cone_grab_1_approach|transport": True,
        "cone_grab_1_approach|cone_grab_1_grab": True,
        "cone_grab_1_grab|cone_grab_1_approach": True,
        "transport|cone_grab_2_approach": False,  # collision
        "cone_grab_2_approach|cone_grab_2_grab": True,
        "cone_grab_2_grab|cone_grab_2_approach": True,
    })
    groups = {"machine_1": {"cones": ["cone_grab_1", "cone_grab_2"]}}
    result = find_gateways(edge_cache, groups, routing_candidates=["transport"])
    assert result["machine_1"]["gateways"] == []
    assert result["machine_1"]["cones"]["cone_grab_2"]["tested"] is False
    assert result["machine_1"]["cones"]["cone_grab_2"]["reason"] == "no_collision_free_path"


def test_find_gateways_ik_failed_cone_excluded():
    """If approach↔grab edge is missing entirely, cone is ik_failed."""
    edge_cache = _make_edge_cache({
        "transport|cone_grab_1_approach": True,
        "cone_grab_1_approach|transport": True,
        # approach|grab edges missing — IK failed for cone_1
    })
    groups = {"machine_1": {"cones": ["cone_grab_1"]}}
    result = find_gateways(edge_cache, groups, routing_candidates=["transport"])
    assert result["machine_1"]["cones"]["cone_grab_1"]["tested"] is False
    assert result["machine_1"]["cones"]["cone_grab_1"]["reason"] == "ik_failed"


def test_find_gateways_respects_gateway_candidates():
    """If gateway_candidates is specified, only those are tried."""
    edge_cache = _make_edge_cache({
        "home|cone_grab_1_approach": True,
        "cone_grab_1_approach|home": True,
        "cone_grab_1_approach|cone_grab_1_grab": True,
        "cone_grab_1_grab|cone_grab_1_approach": True,
        "transport|cone_grab_1_approach": True,
        "cone_grab_1_approach|transport": True,
    })
    groups = {"machine_1": {"cones": ["cone_grab_1"], "gateway_candidates": ["home"]}}
    result = find_gateways(edge_cache, groups, routing_candidates=["home", "transport"])
    assert "home" in result["machine_1"]["gateways"]
    assert "transport" not in result["machine_1"]["gateways"]
