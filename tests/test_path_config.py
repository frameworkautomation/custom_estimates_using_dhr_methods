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
    """Every waypoint in path_config.yaml must be resolvable to joint values.

    Valid forms:
      - joints: [...]              explicit joint list (human-validated)
      - target: "RoboDKName"       resolved by check_collision_free_paths.py at plan time
      - x/y/z + tool_name: "..."  Cartesian pose — tool_name required so IK can be computed

    Cartesian pose WITHOUT tool_name is rejected: we cannot compute joints without
    knowing which tool is mounted (affects TCP offset).
    """
    waypoints = config.get("waypoints", {})
    errors = []

    for name, wp in waypoints.items():
        if not isinstance(wp, dict):
            errors.append(f"{name}: not a mapping")
            continue

        has_joints = isinstance(wp.get("joints"), list)
        has_target = wp.get("target") is not None
        has_cartesian = any(k in wp for k in ("x", "y", "z"))
        has_tool_name = wp.get("tool_name") is not None

        if has_joints or has_target:
            continue  # valid

        if has_cartesian and not has_tool_name:
            errors.append(
                f"{name}: has Cartesian pose (x/y/z) but no tool_name — "
                "cannot compute joints without knowing the tool. "
                "Add tool_name: or capture joints with save_joint_position.py"
            )
        elif not has_cartesian:
            errors.append(
                f"{name}: no joints:, target:, or x/y/z pose — waypoint is incomplete. "
                "Capture joints with: python robodk_code/save_joint_position.py --robodk-ip 172.23.208.1"
            )

    assert not errors, "Waypoint validation errors in path_config.yaml:\n  " + "\n  ".join(errors)


def test_routing_candidates_defined(config):
    """All routing_candidates must exist in waypoints."""
    waypoints = config.get("waypoints", {})
    routing = config.get("routing_candidates", [])
    undefined = [name for name in routing if name not in waypoints]
    assert not undefined, (
        "routing_candidates reference undefined waypoints: " + ", ".join(undefined)
    )
