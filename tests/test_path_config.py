"""
test_path_config.py

Validates robo_dk_output/path_config.yaml structure.
RULE: Every waypoint listed in routing_candidates MUST have a joints: field.
      We never use Cartesian-only (target: or x/y/z-only) waypoints for routing —
      joint values must be explicitly validated by a human and committed.
"""
import os
import pytest
import yaml

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
CONFIG_PATH = os.path.join(REPO_ROOT, "robo_dk_output", "path_config.yaml")


@pytest.fixture(scope="module")
def config():
    if not os.path.exists(CONFIG_PATH):
        pytest.skip(f"path_config.yaml not found at {CONFIG_PATH}")
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_path_config_exists():
    assert os.path.exists(CONFIG_PATH), (
        f"path_config.yaml not found at {CONFIG_PATH}. "
        "This file must be committed — it is tracked via .gitignore exception."
    )


def test_routing_candidates_have_joints(config):
    """Every routing candidate must have an explicit joints: list.

    We never derive joint values from IK at planning time — joint values must
    be validated by a human (via save_joint_position.py) and committed.
    """
    waypoints = config.get("waypoints", {})
    routing = config.get("routing_candidates", [])

    missing = []
    not_a_list = []

    for name in routing:
        wp = waypoints.get(name)
        if wp is None:
            missing.append(f"{name} (not in waypoints section)")
            continue
        joints = wp.get("joints")
        if joints is None:
            missing.append(f"{name} (no joints: field — use save_joint_position.py to capture)")
        elif not isinstance(joints, list):
            not_a_list.append(f"{name} (joints: must be a list, got {type(joints).__name__})")

    errors = []
    if missing:
        errors.append("Missing joints:\n  " + "\n  ".join(missing))
    if not_a_list:
        errors.append("Malformed joints:\n  " + "\n  ".join(not_a_list))

    assert not errors, "\n".join(errors)


def test_all_waypoints_have_joints(config):
    """Every waypoint in path_config.yaml must have joints: — no Cartesian-only entries.

    If a waypoint doesn't have joints yet, capture them with:
        python robodk_code/save_joint_position.py --robodk-ip 172.23.208.1
    """
    waypoints = config.get("waypoints", {})
    missing = [
        name for name, wp in waypoints.items()
        if not isinstance(wp, dict) or wp.get("joints") is None
    ]
    assert not missing, (
        "These waypoints in path_config.yaml are missing joints:\n  "
        + "\n  ".join(missing)
        + "\n\nCapture with: python robodk_code/save_joint_position.py --robodk-ip 172.23.208.1"
    )


def test_routing_candidates_defined(config):
    """All routing_candidates must exist in waypoints."""
    waypoints = config.get("waypoints", {})
    routing = config.get("routing_candidates", [])
    undefined = [name for name in routing if name not in waypoints]
    assert not undefined, (
        "routing_candidates reference undefined waypoints: " + ", ".join(undefined)
    )
