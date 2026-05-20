"""Tests for plan loading and sequence validation in moving_a_cone.py."""
import sys, os, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
import yaml
from robodk_code.path_plan_utils import (
    load_path_plan,
    filter_tested_cones,
    validate_sequence,
    find_cone_group,
    build_sequence_names,
)

# ── Fixtures ───────────────────────────────────────────────────────────────────

VALID_PLAN = {
    "config_hashes": {"collision_critical": "aaa", "structural": "bbb"},
    "edge_cache": {
        "home|transport": {"collision_free": True, "from_joints": [0]*7, "to_joints": [1]*7},
        "transport|base_cone_grab_0_approach": {"collision_free": True, "from_joints": [1]*7, "to_joints": [2]*7},
        "base_cone_grab_0_approach|transport": {"collision_free": True, "from_joints": [2]*7, "to_joints": [1]*7},
        "base_cone_grab_0_approach|base_cone_grab_0_grab": {"collision_free": True, "from_joints": [2]*7, "to_joints": [3]*7},
        "base_cone_grab_0_grab|base_cone_grab_0_approach": {"collision_free": True, "from_joints": [3]*7, "to_joints": [2]*7},
        "transport|cone_grab_1_approach": {"collision_free": True, "from_joints": [1]*7, "to_joints": [4]*7},
        "cone_grab_1_approach|transport": {"collision_free": True, "from_joints": [4]*7, "to_joints": [1]*7},
        "cone_grab_1_approach|cone_grab_1_grab": {"collision_free": True, "from_joints": [4]*7, "to_joints": [5]*7},
        "cone_grab_1_grab|cone_grab_1_approach": {"collision_free": True, "from_joints": [5]*7, "to_joints": [4]*7},
        "transport|home": {"collision_free": True, "from_joints": [1]*7, "to_joints": [0]*7},
    },
    "base_cones": {
        "base_cone_grab_0": {
            "tested": True,
            "approach_joints": [2]*7,
            "grab_joints": [3]*7,
            "gateways": ["transport"],
        },
        "base_cone_grab_1": {"tested": False, "reason": "ik_failed"},
    },
    "destination_groups": {
        "machine_1": {
            "gateways": ["transport"],
            "cones": {
                "cone_grab_1": {
                    "tested": True,
                    "approach_joints": [4]*7,
                    "grab_joints": [5]*7,
                },
                "cone_grab_2": {"tested": False, "reason": "no_collision_free_path"},
            },
        },
    },
}


def _write_plan(plan):
    f = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(plan, f)
    f.close()
    return f.name


# ── load_path_plan ─────────────────────────────────────────────────────────────

def test_load_path_plan_returns_dict():
    path = _write_plan(VALID_PLAN)
    plan = load_path_plan(path, expected_hashes=None)
    assert isinstance(plan, dict)
    os.unlink(path)


def test_load_path_plan_missing_exits():
    with pytest.raises(SystemExit):
        load_path_plan("/nonexistent/path_plan.yaml", expected_hashes=None)


def test_load_path_plan_collision_critical_mismatch_exits():
    path = _write_plan(VALID_PLAN)
    with pytest.raises(SystemExit):
        load_path_plan(path, expected_hashes={"collision_critical": "WRONG", "structural": "bbb"})
    os.unlink(path)


def test_load_path_plan_structural_mismatch_exits():
    path = _write_plan(VALID_PLAN)
    with pytest.raises(SystemExit):
        load_path_plan(path, expected_hashes={"collision_critical": "aaa", "structural": "WRONG"})
    os.unlink(path)


# ── filter_tested_cones ────────────────────────────────────────────────────────

def test_filter_tested_cones_excludes_failed():
    result = filter_tested_cones(
        ["base_cone_grab_0", "base_cone_grab_1"],
        VALID_PLAN["base_cones"],
    )
    assert "base_cone_grab_0" in result
    assert "base_cone_grab_1" not in result


def test_filter_tested_cones_excludes_unknown():
    result = filter_tested_cones(
        ["base_cone_grab_0", "base_cone_grab_99"],
        VALID_PLAN["base_cones"],
    )
    assert "base_cone_grab_99" not in result


# ── find_cone_group ────────────────────────────────────────────────────────────

def test_find_cone_group_returns_group():
    group_name, cone_data = find_cone_group("cone_grab_1", VALID_PLAN["destination_groups"])
    assert group_name == "machine_1"
    assert cone_data["tested"] is True


def test_find_cone_group_missing_returns_none():
    result = find_cone_group("cone_grab_99", VALID_PLAN["destination_groups"])
    assert result is None


# ── validate_sequence ──────────────────────────────────────────────────────────

def test_validate_sequence_all_clear():
    problems = validate_sequence(
        ["home", "transport", "base_cone_grab_0_approach"],
        VALID_PLAN["edge_cache"],
    )
    assert problems == []


def test_validate_sequence_missing_edge():
    problems = validate_sequence(
        ["home", "nonexistent_node"],
        VALID_PLAN["edge_cache"],
    )
    assert len(problems) == 1
    assert "not tested" in problems[0].lower() or "missing" in problems[0].lower()


def test_validate_sequence_collision_edge():
    edge_cache = dict(VALID_PLAN["edge_cache"])
    edge_cache["home|transport"] = {"collision_free": False, "from_joints": [0]*7, "to_joints": [1]*7}
    problems = validate_sequence(["home", "transport"], edge_cache)
    assert len(problems) == 1
    assert "collision" in problems[0].lower()


# ── build_sequence_names ───────────────────────────────────────────────────────

def test_build_sequence_names_full_path():
    seq = build_sequence_names(
        base_cone_name="base_cone_grab_0",
        dest_cone_name="cone_grab_1",
        plan=VALID_PLAN,
        routing_candidates=["home", "transport"],
    )
    # Must visit: home, transport, base approach, base grab, retract, transport, dest approach, dest grab, retract, transport, home
    assert seq[0] == "home"
    assert "base_cone_grab_0_approach" in seq
    assert "base_cone_grab_0_grab" in seq
    assert "cone_grab_1_approach" in seq
    assert "cone_grab_1_grab" in seq
    assert seq[-1] == "home"
    # grab must be preceded and followed by approach
    bi = seq.index("base_cone_grab_0_grab")
    assert seq[bi - 1] == "base_cone_grab_0_approach"
    assert seq[bi + 1] == "base_cone_grab_0_approach"
