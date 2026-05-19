# Collision-Free Path Planning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an offline collision-free path checker that tests every directed edge between routing waypoints and cone targets using RoboDK's `MoveJ_Test`, discovers gateway waypoints per destination group, writes a `path_plan.yaml`, and integrates with `moving_a_cone.py` so every move is pre-validated before execution.

**Architecture:** Mixin pipeline (modelled on knitwear-cell `robot.py`) in `robot_controller.py` provides `MoveJTestModel` and `MoveJModel`. `check_collision_free_paths.py` builds a node graph, attaches cone mesh (worst-case geometry), tests all directed edges, runs Dijkstra to find gateway waypoints, and writes `path_plan.yaml`. `moving_a_cone.py` loads the plan, filters untested cones, builds a named-node sequence, validates every edge is in the cache and collision-free, then executes. No RoboDK calls happen inside `moving_a_cone.py` for IK or collision testing — everything is pre-computed.

**Tech Stack:** Python 3.10+, RoboDK Python API (`robolink`, `robomath`), PyYAML, pytest, `unittest.mock`

**Branch:** `collision_free_path_planning` (branched from `determining_how_position_gripper` after bug fix is merged)

---

## File Map

| File | Created/Modified | Responsibility |
|------|-----------------|----------------|
| `robodk_code/robot_controller.py` | Create | Mixin pipeline: `RobotControllerMixin`, `MoveJTestModel`, `MoveJModel`, `Robot` |
| `robodk_code/check_collision_free_paths.py` | Create | Config loading, node construction, edge testing, gateway discovery, pathfinding, YAML writer |
| `robo_dk_output/path_config.yaml` | Create | Human-editable template: waypoints, routing candidates, destination groups, collision pairs |
| `robodk_code/moving_a_cone.py` | Modify | Load plan, filter cones, build sequence, validate edges, execute with via-points |
| `tests/conftest.py` | Create | Shared pytest fixtures (mock RoboDK objects) |
| `tests/test_robot_controller.py` | Create | Unit tests for mixin pipeline and edge caching |
| `tests/test_path_planner.py` | Create | Unit tests for pathfinding, gateway discovery, hash computation |
| `tests/test_path_plan_integration.py` | Create | Unit tests for plan loading, cone filtering, sequence building, edge validation |

---

## Node Naming Convention

Used consistently across all files:

- Routing candidates: `"{waypoint_name}"` — e.g., `"home"`, `"transport"`
- Base cone approach: `"base_cone_grab_{N}_approach"` — e.g., `"base_cone_grab_0_approach"`
- Base cone grab: `"base_cone_grab_{N}_grab"`
- Destination cone approach: `"cone_grab_{N}_approach"`
- Destination cone grab: `"cone_grab_{N}_grab"`

Edge cache keys: `"{from_node}|{to_node}"`

---

## Task 1: Set up pytest and test infrastructure

**Files:**
- Create: `tests/__init__.py`
- Create: `tests/conftest.py`
- Create: `pytest.ini`

- [ ] **Step 1: Install pytest and pyyaml**

```bash
pip install pytest pyyaml
```

- [ ] **Step 2: Create `pytest.ini`**

```ini
[pytest]
testpaths = tests
python_files = test_*.py
python_classes = Test*
python_functions = test_*
```

- [ ] **Step 3: Create `tests/__init__.py`**

Empty file:
```python
```

- [ ] **Step 4: Create `tests/conftest.py`**

```python
"""Shared fixtures for all tests. RoboDK is mocked — no live connection needed."""
import pytest
from unittest.mock import MagicMock


@pytest.fixture
def mock_robot():
    """Minimal RoboDK robot Item mock."""
    robot = MagicMock()
    robot.Joints.return_value.list.return_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot.MoveJ_Test.return_value = 0  # default: no collision
    return robot


@pytest.fixture
def mock_rdk():
    """Minimal Robolink mock."""
    rdk = MagicMock()
    return rdk
```

- [ ] **Step 5: Verify pytest discovers tests (will show 0 tests, no errors)**

```bash
cd /mnt/c/Users/samst/Framework/clones/custom_estimates_using_dhr_methods
pytest --collect-only
```
Expected: `no tests ran` with exit code 5 (no tests found yet).

- [ ] **Step 6: Commit**

```bash
git add pytest.ini tests/
git commit -m "test: set up pytest and shared fixtures"
```

---

## Task 2: `robot_controller.py` — mixin pipeline

**Files:**
- Create: `robodk_code/robot_controller.py`
- Create: `tests/test_robot_controller.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/test_robot_controller.py`:

```python
"""Tests for the mixin pipeline in robot_controller.py."""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from unittest.mock import MagicMock
from robodk_code.robot_controller import Robot, MoveJTestModel, MoveJModel


def test_move_j_test_caches_clear_result(mock_robot):
    controller = Robot(MoveJTestModel)
    mock_robot.MoveJ_Test.return_value = 0  # no collision
    state = {
        "robot": mock_robot,
        "from_joints": [0.0] * 7,
        "to_joints": [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
    controller.execute(state)
    assert state["collision_free"] is True
    assert len(controller.edge_cache) == 1


def test_move_j_test_caches_collision_result(mock_robot):
    mock_robot.MoveJ_Test.return_value = 1  # collision
    controller = Robot(MoveJTestModel)
    state = {
        "robot": mock_robot,
        "from_joints": [0.0] * 7,
        "to_joints": [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
    controller.execute(state)
    assert state["collision_free"] is False


def test_move_j_test_uses_cache_on_second_call(mock_robot):
    mock_robot.MoveJ_Test.return_value = 0
    controller = Robot(MoveJTestModel)
    state = {
        "robot": mock_robot,
        "from_joints": [0.0] * 7,
        "to_joints": [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
    controller.execute(state)
    controller.execute(state)  # same state — should hit cache
    mock_robot.MoveJ_Test.assert_called_once()  # only called once total


def test_move_j_model_executes_when_clear(mock_robot):
    controller = Robot(MoveJModel)
    joints = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    state = {"robot": mock_robot, "target_joints": joints, "collision_free": True}
    controller.execute(state)
    mock_robot.MoveJ.assert_called_once_with(joints)


def test_move_j_model_skips_when_collision(mock_robot):
    controller = Robot(MoveJModel)
    state = {
        "robot": mock_robot,
        "target_joints": [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "collision_free": False,
    }
    controller.execute(state)
    mock_robot.MoveJ.assert_not_called()


def test_robot_edge_cache_shared_across_calls(mock_robot):
    """Edge cache persists on the Robot instance across multiple execute() calls."""
    mock_robot.MoveJ_Test.return_value = 0
    controller = Robot(MoveJTestModel)

    state1 = {"robot": mock_robot, "from_joints": [0.0]*7, "to_joints": [1.0]*7}
    state2 = {"robot": mock_robot, "from_joints": [1.0]*7, "to_joints": [2.0]*7}

    controller.execute(state1)
    controller.execute(state2)

    assert len(controller.edge_cache) == 2
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
pytest tests/test_robot_controller.py -v
```
Expected: `ImportError: No module named 'robodk_code.robot_controller'`

- [ ] **Step 3: Create `robodk_code/robot_controller.py`**

```python
"""
robot_controller.py

Mixin pipeline modelled on knitwear-cell robot.py.
Used by check_collision_free_paths.py (MoveJTestModel) and
moving_a_cone.py (MoveJModel).

Usage:
    checker = Robot(MoveJTestModel)
    state = {"robot": robot_item, "from_joints": [...], "to_joints": [...]}
    checker.execute(state)
    # state["collision_free"] is now set; checker.edge_cache is populated

    mover = Robot(MoveJModel)
    state = {"robot": robot_item, "target_joints": [...], "collision_free": True}
    mover.execute(state)
"""

import hashlib
import json
from abc import ABC, abstractmethod
from typing import List


def _hash_joints(joints: list) -> str:
    """Deterministic hash of a joint list for cache keying."""
    return hashlib.sha256(json.dumps([round(j, 6) for j in joints]).encode()).hexdigest()


class RobotControllerMixin(ABC):
    """Base class for all robot controller mixins."""

    def __init__(self, parent: "Robot"):
        self._parent = parent

    @abstractmethod
    def execute(self, state: dict) -> None:
        """
        Execute this step. Reads and writes fields in `state`:
          - from_joints (list[float]): start configuration
          - to_joints (list[float]): target configuration (for test)
          - target_joints (list[float]): target configuration (for move)
          - collision_free (bool): set by MoveJTestModel, read by MoveJModel
          - robot: RoboDK Item (robot)
        """


class MoveJTestModel(RobotControllerMixin):
    """
    Test a joint move for collisions using robot.MoveJ_Test().
    Caches results on the parent Robot by edge key (from_joints|to_joints).
    Sets state["collision_free"] = True if no collision, False otherwise.
    """

    def execute(self, state: dict) -> None:
        from_joints = state["from_joints"]
        to_joints = state["to_joints"]
        robot = state["robot"]

        cache_key = _hash_joints(from_joints) + "|" + _hash_joints(to_joints)

        if cache_key in self._parent.edge_cache:
            entry = self._parent.edge_cache[cache_key]
            state["collision_free"] = entry["collision_free"]
            return

        result = robot.MoveJ_Test(from_joints, to_joints)
        collision_free = (result == 0)

        self._parent.edge_cache[cache_key] = {
            "from_joints": list(from_joints),
            "to_joints": list(to_joints),
            "collision_free": collision_free,
        }
        state["collision_free"] = collision_free


class MoveJModel(RobotControllerMixin):
    """
    Execute robot.MoveJ(target_joints) only if state["collision_free"] is True.
    If collision_free is not set in state, defaults to allowing the move
    (use when collision testing was done offline).
    """

    def execute(self, state: dict) -> None:
        if not state.get("collision_free", True):
            return
        state["robot"].MoveJ(state["target_joints"])


class Robot:
    """
    Composes mixin classes into a pipeline. Mixins execute in order.
    Holds the edge_cache dict shared across all execute() calls.

    Example:
        checker = Robot(MoveJTestModel)
        mover = Robot(MoveJModel)
    """

    def __init__(self, *mixin_classes):
        self.edge_cache: dict = {}
        self._mixins: List[RobotControllerMixin] = [m(self) for m in mixin_classes]

    def execute(self, state: dict) -> dict:
        """Run all mixins in order. Returns the mutated state dict."""
        for mixin in self._mixins:
            mixin.execute(state)
        return state
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
pytest tests/test_robot_controller.py -v
```
Expected: all 6 tests pass.

- [ ] **Step 5: Commit**

```bash
git add robodk_code/robot_controller.py tests/test_robot_controller.py
git commit -m "feat: add robot_controller mixin pipeline (MoveJTestModel, MoveJModel)"
```

---

## Task 3: `path_config.yaml` — human-editable template

**Files:**
- Create: `robo_dk_output/path_config.yaml`

- [ ] **Step 1: Create the template**

```bash
mkdir -p robo_dk_output
```

Create `robo_dk_output/path_config.yaml`:

```yaml
# path_config.yaml
# Human-editable configuration for the collision-free path planner.
# Edit this file, then re-run check_collision_free_paths.py to update path_plan.yaml.

# Tool mounted on robot during all collision checks.
default_tool: "pickup_closed"

# Any valid base cone mesh name. Attached to tool during ALL checks (worst-case geometry).
# If a path is clear with the cone attached, it is clear without it too.
cone_mesh_template: "base_cone_0"

# Named waypoints. Each must specify either:
#   joints: [j1, j2, j3, j4, j5, j6, j7]   — explicit joint values
#   target: "RoboDKTargetName"               — resolved to joints by the checker at plan-creation time
# Optional per-waypoint tool override:
#   tool: "tool_name"                        — overrides default_tool at this waypoint
waypoints:
  home:
    joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  transport:
    joints: [0.0, -55.0, 30.0, 0.0, -30.0, -90.0, 0.0]
  # Example target-based waypoint (uncomment and fill in):
  # curtain_safe_machine_1:
  #   target: "approach_machine_1_curtain_safe"
  #   tool: "pickup_open"

# Waypoints the pathfinder may route through.
# Must be a subset of the keys in `waypoints` above.
routing_candidates:
  - home
  - transport

# Destination cone groups. Human-defined by machine/zone.
# cones: list of RoboDK target names (cone_grab_* items in station)
# gateway_candidates (optional): restrict which routing_candidates are tested as
#   direct gateways for this group. If omitted, all routing_candidates are tried.
destination_groups:
  machine_1:
    # gateway_candidates: [transport]   # uncomment to restrict
    cones:
      - cone_grab_1
      - cone_grab_2
      - cone_grab_3
  # machine_2:
  #   cones:
  #     - cone_grab_4

# Collision pairs to enable/disable beyond RoboDK station defaults.
# Format: [item_name_a, item_name_b]
collision_enable: []
collision_disable: []
```

- [ ] **Step 2: Commit**

```bash
git add robo_dk_output/path_config.yaml
git commit -m "feat: add path_config.yaml template"
```

---

## Task 4: `check_collision_free_paths.py` — config loading, node construction, hash computation

**Files:**
- Create: `robodk_code/check_collision_free_paths.py`
- Create: `tests/test_path_planner.py`

- [ ] **Step 1: Write failing tests for config loading and hash computation**

Create `tests/test_path_planner.py`:

```python
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
```

- [ ] **Step 2: Run to confirm failure**

```bash
pytest tests/test_path_planner.py -v -k "load_config or hash"
```
Expected: `ImportError: No module named 'robodk_code.check_collision_free_paths'`

- [ ] **Step 3: Create `robodk_code/check_collision_free_paths.py` with load_config and compute_config_hashes**

```python
"""
check_collision_free_paths.py

Offline collision-free path checker. Requires RoboDK to be running.

Usage:
    python robodk_code/check_collision_free_paths.py

Reads:  robo_dk_output/path_config.yaml
Writes: robo_dk_output/path_plan.yaml
"""

import sys
import os
import hashlib
import json
import datetime

sys.path.append("C:/RoboDK/Python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import yaml

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output", "path_config.yaml"
)
PLAN_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output", "path_plan.yaml"
)
IK_SOLUTIONS_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "ik_solutions"
)
ROBODK_OUTPUT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output"
)


# ── Config loading ─────────────────────────────────────────────────────────────

def load_config(path: str) -> dict:
    """Load and return path_config.yaml. Exits with message if file missing."""
    if not os.path.isfile(path):
        print(f"[ERROR] path_config.yaml not found at: {path}")
        print("        Create it from the template in robo_dk_output/path_config.yaml")
        sys.exit(1)
    with open(path) as f:
        return yaml.safe_load(f)


# ── Config hashing ─────────────────────────────────────────────────────────────

def compute_config_hashes(config: dict) -> dict:
    """
    Compute two hashes of the config:
      collision_critical — covers: collision_enable, collision_disable, cone_mesh_template
      structural         — covers: waypoints (names + joints), routing_candidates

    If collision_critical changes, all cached edges are invalid (re-run required).
    If structural changes, edges referencing changed/removed waypoints are invalid (re-run required).
    Additive changes (new groups, new waypoints not yet in routing_candidates) do not
    affect either hash and are safe to continue with.
    """
    def _hash(obj) -> str:
        return hashlib.sha256(
            json.dumps(obj, sort_keys=True).encode()
        ).hexdigest()[:16]

    collision_critical_data = {
        "collision_enable": config.get("collision_enable", []),
        "collision_disable": config.get("collision_disable", []),
        "cone_mesh_template": config.get("cone_mesh_template", ""),
    }

    structural_data = {
        "waypoints": {
            name: wp.get("joints") or wp.get("target")
            for name, wp in config.get("waypoints", {}).items()
        },
        "routing_candidates": sorted(config.get("routing_candidates", [])),
    }

    return {
        "collision_critical": _hash(collision_critical_data),
        "structural": _hash(structural_data),
    }
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
pytest tests/test_path_planner.py -v -k "load_config or hash"
```
Expected: 6 tests pass.

- [ ] **Step 5: Commit**

```bash
git add robodk_code/check_collision_free_paths.py tests/test_path_planner.py
git commit -m "feat: add config loading and hash computation"
```

---

## Task 5: `check_collision_free_paths.py` — pathfinding and gateway discovery

**Files:**
- Modify: `robodk_code/check_collision_free_paths.py` (add `find_shortest_path`, `find_gateways`)
- Modify: `tests/test_path_planner.py` (add pathfinding and gateway tests)

- [ ] **Step 1: Add failing tests to `tests/test_path_planner.py`**

Append to `tests/test_path_planner.py`:

```python
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
```

- [ ] **Step 2: Run to confirm failure**

```bash
pytest tests/test_path_planner.py -v -k "shortest_path or gateway"
```
Expected: `ImportError` — `find_shortest_path` and `find_gateways` not defined yet.

- [ ] **Step 3: Add `find_shortest_path` to `check_collision_free_paths.py`**

Append to `robodk_code/check_collision_free_paths.py`:

```python
# ── Pathfinding ────────────────────────────────────────────────────────────────

def find_shortest_path(collision_free_edges: set, start: str, end: str) -> list | None:
    """
    Find the shortest path (fewest hops) from start to end using only edges
    in collision_free_edges (set of (from, to) tuples).
    Returns ordered list of node names, or None if no path exists.
    """
    import heapq
    if start == end:
        return [start]
    heap = [(0, start, [start])]
    visited = set()
    while heap:
        cost, node, path = heapq.heappop(heap)
        if node in visited:
            continue
        visited.add(node)
        if node == end:
            return path
        for (f, t) in collision_free_edges:
            if f == node and t not in visited:
                heapq.heappush(heap, (cost + 1, t, path + [t]))
    return None
```

- [ ] **Step 4: Add `find_gateways` to `check_collision_free_paths.py`**

Append to `robodk_code/check_collision_free_paths.py`:

```python
# ── Gateway discovery ──────────────────────────────────────────────────────────

def _approach_key(cone_name: str) -> str:
    return f"{cone_name}_approach"

def _grab_key(cone_name: str) -> str:
    return f"{cone_name}_grab"

def _edge_clear(edge_cache: dict, from_node: str, to_node: str) -> bool:
    key = f"{from_node}|{to_node}"
    return edge_cache.get(key, {}).get("collision_free", False)

def _cone_ik_ok(edge_cache: dict, cone_name: str) -> bool:
    """IK is considered OK if approach→grab and grab→approach edges are both present and clear."""
    ak = _approach_key(cone_name)
    gk = _grab_key(cone_name)
    return (
        _edge_clear(edge_cache, ak, gk) and
        _edge_clear(edge_cache, gk, ak)
    )

def find_gateways(edge_cache: dict, destination_groups: dict, routing_candidates: list) -> dict:
    """
    For each destination group, find which routing_candidates qualify as gateways.

    A candidate R is a gateway for a group if, for EVERY cone in the group:
      - R → cone_approach is collision_free
      - cone_approach → R is collision_free (retract)
      - cone_approach → cone_grab is collision_free
      - cone_grab → cone_approach is collision_free

    Returns:
      {
        group_name: {
          "gateways": [candidate_name, ...],
          "cones": {
            cone_name: {"tested": True} | {"tested": False, "reason": "ik_failed"|"no_collision_free_path"}
          }
        }
      }
    """
    result = {}

    for group_name, group in destination_groups.items():
        cones = group["cones"]
        candidates = group.get("gateway_candidates", routing_candidates)

        # Determine per-cone IK status first (independent of gateway)
        cone_ik = {c: _cone_ik_ok(edge_cache, c) for c in cones}

        # Find valid gateways (must reach every cone with good IK)
        valid_gateways = []
        for candidate in candidates:
            is_gateway = all(
                cone_ik[c] and
                _edge_clear(edge_cache, candidate, _approach_key(c)) and
                _edge_clear(edge_cache, _approach_key(c), candidate)
                for c in cones
            )
            if is_gateway:
                valid_gateways.append(candidate)

        # Determine per-cone tested status
        cones_result = {}
        for cone_name in cones:
            if not cone_ik[cone_name]:
                cones_result[cone_name] = {"tested": False, "reason": "ik_failed"}
            elif not any(
                _edge_clear(edge_cache, gw, _approach_key(cone_name))
                for gw in valid_gateways
            ):
                cones_result[cone_name] = {"tested": False, "reason": "no_collision_free_path"}
            else:
                cones_result[cone_name] = {"tested": True}

        result[group_name] = {
            "gateways": valid_gateways,
            "cones": cones_result,
        }

    return result
```

- [ ] **Step 5: Run all path_planner tests**

```bash
pytest tests/test_path_planner.py -v
```
Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add robodk_code/check_collision_free_paths.py tests/test_path_planner.py
git commit -m "feat: add pathfinding and gateway discovery"
```

---

## Task 6: `check_collision_free_paths.py` — node construction and edge testing (requires RoboDK)

**Files:**
- Modify: `robodk_code/check_collision_free_paths.py` (add `build_nodes`, `test_all_edges`, `write_plan`, `main`)

These functions talk to RoboDK directly and are not unit-testable without a live instance. Test by running the script with RoboDK open.

- [ ] **Step 1: Add `build_nodes` to `check_collision_free_paths.py`**

Append:

```python
# ── Node construction ──────────────────────────────────────────────────────────

def _resolve_waypoint_joints(rdk, robot, wp_config: dict) -> list | None:
    """
    Resolve a waypoint config entry to a joint list.
    Accepts:
      {"joints": [...]}           — used directly
      {"target": "TargetName"}    — RoboDK target resolved via SolveIK
    Returns None if resolution fails.
    """
    from robodk.robolink import ITEM_TYPE_TARGET, ITEM_TYPE_FRAME
    from robodk.robomath import eye

    if "joints" in wp_config:
        return list(wp_config["joints"])

    target_name = wp_config.get("target")
    if target_name is None:
        print(f"[ERROR] Waypoint has neither 'joints' nor 'target'")
        return None

    target = rdk.Item(target_name, ITEM_TYPE_TARGET)
    if not target.Valid():
        print(f"[ERROR] RoboDK target not found: '{target_name}'")
        return None

    world_frame = rdk.Item("WorldFrame", ITEM_TYPE_FRAME)
    robot.setPoseFrame(world_frame)
    pose = target.PoseAbs()
    result = robot.SolveIK(pose)
    try:
        joints = result.list()
    except AttributeError:
        joints = list(result)
    if len(joints) < 6:
        print(f"[ERROR] IK failed for target '{target_name}'")
        return None
    return joints


def build_nodes(rdk, robot, config: dict, base_ik: dict, dest_ik: dict) -> dict:
    """
    Build the complete node dict: {node_name: joint_list}.

    Sources:
      - routing_candidates from config["waypoints"] (resolved via _resolve_waypoint_joints)
      - base cone approach + grab from base_ik (pre-computed IK solutions)
      - destination cone approach + grab from dest_ik

    base_ik format (from load_latest_base_cone_ik):
      {cone_name: {"grab_ok": bool, "grab_joints": [...], "app_ok": bool, "app_joints": [...]}}

    dest_ik format (from compute_dest_ik):
      same schema as base_ik
    """
    nodes = {}

    # Routing candidate waypoints
    for name in config.get("routing_candidates", []):
        wp = config["waypoints"].get(name)
        if wp is None:
            print(f"[WARN] routing_candidate '{name}' not in waypoints — skipping")
            continue
        joints = _resolve_waypoint_joints(rdk, robot, wp)
        if joints is not None:
            nodes[name] = joints

    # Base cone nodes
    for cone_name, ik in base_ik.items():
        if ik.get("app_ok") and ik.get("grab_ok"):
            nodes[f"{cone_name}_approach"] = list(ik["app_joints"])
            nodes[f"{cone_name}_grab"] = list(ik["grab_joints"])
        else:
            print(f"[INFO] Skipping {cone_name} (IK failed)")

    # Destination cone nodes
    for cone_name, ik in dest_ik.items():
        if ik.get("app_ok") and ik.get("grab_ok"):
            nodes[f"{cone_name}_approach"] = list(ik["app_joints"])
            nodes[f"{cone_name}_grab"] = list(ik["grab_joints"])
        else:
            print(f"[INFO] Skipping {cone_name} (IK failed)")

    print(f"[INFO] Built {len(nodes)} nodes for edge testing")
    return nodes
```

- [ ] **Step 2: Add `test_all_edges` to `check_collision_free_paths.py`**

Append:

```python
# ── Edge testing ───────────────────────────────────────────────────────────────

def test_all_edges(rdk, robot, tool, cone_mesh, nodes: dict, config: dict) -> dict:
    """
    Test every directed pair of nodes with robot.MoveJ_Test().
    Cone mesh is attached to tool before testing (worst-case geometry).
    Returns edge_cache dict: {"{from}|{to}": {"from_joints", "to_joints", "collision_free"}}.

    collision_enable/disable pairs from config are applied before testing.
    """
    from robodk.robolink import COLLISION_ON, COLLISION_OFF, ITEM_TYPE_FRAME
    from robot_controller import Robot, MoveJTestModel

    # Apply collision pair configuration
    for pair in config.get("collision_enable", []):
        item_a = rdk.Item(pair[0])
        item_b = rdk.Item(pair[1])
        if item_a.Valid() and item_b.Valid():
            rdk.setCollisionActivePair(COLLISION_ON, item_a, item_b)

    for pair in config.get("collision_disable", []):
        item_a = rdk.Item(pair[0])
        item_b = rdk.Item(pair[1])
        if item_a.Valid() and item_b.Valid():
            rdk.setCollisionActivePair(COLLISION_OFF, item_a, item_b)

    # Attach cone mesh to tool (worst-case geometry for all tests)
    world_frame = rdk.Item("WorldFrame", ITEM_TYPE_FRAME)
    if cone_mesh is not None and cone_mesh.Valid():
        cone_mesh.setParentStatic(tool)
        print(f"[INFO] Cone mesh '{cone_mesh.Name()}' attached to '{tool.Name()}' for collision testing")

    robot.setTool(tool)
    rdk.Render(False)

    checker = Robot(MoveJTestModel)
    node_names = sorted(nodes.keys())
    total = len(node_names) * (len(node_names) - 1)
    tested = 0

    print(f"\nTesting {total} directed edges between {len(node_names)} nodes ...")
    for from_name in node_names:
        for to_name in node_names:
            if from_name == to_name:
                continue
            state = {
                "robot": robot,
                "from_joints": nodes[from_name],
                "to_joints": nodes[to_name],
            }
            checker.execute(state)
            edge_key = f"{from_name}|{to_name}"
            checker.edge_cache[edge_key] = {
                "from_joints": nodes[from_name],
                "to_joints": nodes[to_name],
                "collision_free": state["collision_free"],
            }
            tested += 1
            if tested % 20 == 0:
                print(f"  {tested}/{total} edges tested ...")

    rdk.Render(True)

    # Detach cone mesh back to world
    if cone_mesh is not None and cone_mesh.Valid():
        cone_mesh.setParentStatic(world_frame)

    clear = sum(1 for v in checker.edge_cache.values() if v["collision_free"])
    print(f"[INFO] Edge testing complete: {clear}/{len(checker.edge_cache)} collision-free")
    return checker.edge_cache
```

- [ ] **Step 3: Add `write_plan` and `main` to `check_collision_free_paths.py`**

Append:

```python
# ── Plan writer ────────────────────────────────────────────────────────────────

def write_plan(plan_path: str, config: dict, edge_cache: dict,
               base_cones_result: dict, dest_groups_result: dict) -> None:
    """Write path_plan.yaml. base_cones_result and dest_groups_result come from find_gateways."""
    hashes = compute_config_hashes(config)

    plan = {
        "generated": datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "config_hashes": hashes,
        "edge_cache": edge_cache,
        "base_cones": base_cones_result,
        "destination_groups": dest_groups_result,
    }

    os.makedirs(os.path.dirname(plan_path), exist_ok=True)
    with open(plan_path, "w") as f:
        yaml.dump(plan, f, default_flow_style=False, sort_keys=True)
    print(f"\n[INFO] path_plan.yaml written to: {plan_path}")


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    import sys
    sys.path.append("C:/RoboDK/Python")
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME

    # Import existing IK helpers from moving_a_cone.py
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from moving_a_cone import (
        load_latest_base_cone_ik,
        compute_all_offsets,
        find_base_cones,
        compute_dest_ik,
        find_destination_cones,
        ROBOT_NAME,
        TOOL_NAME,
    )

    config = load_config(CONFIG_PATH)

    # Connect to RoboDK
    try:
        rdk = Robolink()
        rdk.Item("")
        print("[INFO] Connected to RoboDK")
    except Exception as e:
        print(f"[ERROR] Cannot connect to RoboDK: {e}")
        sys.exit(1)

    robot = rdk.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        print(f"[ERROR] Robot '{ROBOT_NAME}' not found")
        sys.exit(1)

    tool_name = config.get("default_tool", TOOL_NAME)
    tool = rdk.Item(tool_name, ITEM_TYPE_TOOL)
    if not tool.Valid():
        print(f"[ERROR] Tool '{tool_name}' not found")
        sys.exit(1)

    cone_mesh_name = config.get("cone_mesh_template", "base_cone_0")
    cone_mesh = rdk.Item(cone_mesh_name)
    if not cone_mesh.Valid():
        print(f"[WARN] cone_mesh_template '{cone_mesh_name}' not found — collision checks without cone geometry")
        cone_mesh = None

    # Load or compute base cone IK
    base_ik = load_latest_base_cone_ik()
    if base_ik is None:
        base_cones = find_base_cones(rdk)
        base_ik = compute_all_offsets(rdk, robot, base_cones)

    # Load or compute dest cone IK
    dest_cones = find_destination_cones(rdk)
    dest_ik = compute_dest_ik(rdk, robot, dest_cones, tool)

    # Build nodes and test edges
    nodes = build_nodes(rdk, robot, config, base_ik, dest_ik)
    edge_cache = test_all_edges(rdk, robot, tool, cone_mesh, nodes, config)

    # Build routing candidate graph for pathfinding (used by gateway discovery)
    collision_free_candidate_edges = set()
    routing_candidates = config.get("routing_candidates", [])
    for f in routing_candidates:
        for t in routing_candidates:
            if f != t and edge_cache.get(f"{f}|{t}", {}).get("collision_free"):
                collision_free_candidate_edges.add((f, t))

    # Find gateways for destination groups
    dest_groups_result = find_gateways(
        edge_cache,
        config.get("destination_groups", {}),
        routing_candidates,
    )

    # Find gateways for base cones
    base_cones_result = {}
    for cone_name, ik in base_ik.items():
        ak = f"{cone_name}_approach"
        gk = f"{cone_name}_grab"
        ik_ok = (
            edge_cache.get(f"{ak}|{gk}", {}).get("collision_free", False) and
            edge_cache.get(f"{gk}|{ak}", {}).get("collision_free", False)
        )
        if not ik_ok:
            base_cones_result[cone_name] = {"tested": False, "reason": "ik_failed"}
            continue

        gateways = [
            c for c in routing_candidates
            if edge_cache.get(f"{c}|{ak}", {}).get("collision_free", False) and
               edge_cache.get(f"{ak}|{c}", {}).get("collision_free", False)
        ]
        if not gateways:
            base_cones_result[cone_name] = {"tested": False, "reason": "no_collision_free_path"}
        else:
            base_cones_result[cone_name] = {
                "tested": True,
                "approach_joints": list(nodes[ak]),
                "grab_joints": list(nodes[gk]),
                "gateways": gateways,
            }

    # Add approach/grab joints to tested dest cones
    for group_name, group_result in dest_groups_result.items():
        for cone_name, cone_result in group_result["cones"].items():
            if cone_result["tested"]:
                ak = f"{cone_name}_approach"
                gk = f"{cone_name}_grab"
                cone_result["approach_joints"] = list(nodes[ak])
                cone_result["grab_joints"] = list(nodes[gk])

    write_plan(PLAN_PATH, config, edge_cache, base_cones_result, dest_groups_result)

    # Summary
    base_ok = sum(1 for v in base_cones_result.values() if v.get("tested"))
    base_total = len(base_cones_result)
    print(f"\n[SUMMARY] Base cones:        {base_ok}/{base_total} usable")
    for g, gr in dest_groups_result.items():
        ok = sum(1 for c in gr["cones"].values() if c.get("tested"))
        tot = len(gr["cones"])
        gws = gr["gateways"]
        print(f"          {g}: {ok}/{tot} cones usable, gateways={gws}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Test by running with RoboDK open**

```bash
python robodk_code/check_collision_free_paths.py
```
Expected output:
```
[INFO] Connected to RoboDK
[INFO] Built N nodes for edge testing
Testing X directed edges between N nodes ...
[INFO] Edge testing complete: Y/X collision-free
[INFO] path_plan.yaml written to: robo_dk_output/path_plan.yaml
[SUMMARY] Base cones: N/M usable
          machine_1: K/L cones usable, gateways=[...]
```
Verify `robo_dk_output/path_plan.yaml` exists and contains `edge_cache`, `base_cones`, `destination_groups`.

- [ ] **Step 5: Commit**

```bash
git add robodk_code/check_collision_free_paths.py
git commit -m "feat: add node construction, edge testing, plan writer, and main()"
```

---

## Task 7: `moving_a_cone.py` — plan loading, cone filtering, sequence validation

**Files:**
- Modify: `robodk_code/moving_a_cone.py`
- Create: `tests/test_path_plan_integration.py`

- [ ] **Step 1: Write failing integration tests**

Create `tests/test_path_plan_integration.py`:

```python
"""Tests for plan loading and sequence validation in moving_a_cone.py."""
import sys, os, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
import yaml
from robodk_code.moving_a_cone import (
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
```

- [ ] **Step 2: Run to confirm failure**

```bash
pytest tests/test_path_plan_integration.py -v
```
Expected: `ImportError` for `load_path_plan`, `filter_tested_cones`, etc.

- [ ] **Step 3: Add the new functions to `moving_a_cone.py`**

Add the following block near the top of `moving_a_cone.py`, after the existing imports:

```python
# ── Path plan support ──────────────────────────────────────────────────────────

import yaml as _yaml

_PATH_PLAN: dict | None = None  # loaded once at startup


def load_path_plan(plan_path: str, expected_hashes: dict | None) -> dict:
    """
    Load path_plan.yaml. Exits if:
      - file missing
      - collision_critical hash mismatch (cached results invalid)
      - structural hash mismatch (waypoint joints changed)
    Warns (continues) if only additive config changes detected.
    """
    if not os.path.isfile(plan_path):
        print(f"[ERROR] path_plan.yaml not found: {plan_path}")
        print("        Run check_collision_free_paths.py to generate it.")
        sys.exit(1)

    with open(plan_path) as f:
        plan = _yaml.safe_load(f)

    if expected_hashes is not None:
        stored = plan.get("config_hashes", {})
        if stored.get("collision_critical") != expected_hashes.get("collision_critical"):
            print("[ERROR] Collision configuration has changed — cached paths are invalid.")
            print("        Re-run check_collision_free_paths.py before proceeding.")
            sys.exit(1)
        if stored.get("structural") != expected_hashes.get("structural"):
            print("[ERROR] Waypoint definitions have changed — cached edge joints are stale.")
            print("        Re-run check_collision_free_paths.py before proceeding.")
            sys.exit(1)

    return plan


def filter_tested_cones(cone_names: list, cone_plan: dict) -> list:
    """Return only cone names that are tested: true in the plan."""
    return [
        name for name in cone_names
        if cone_plan.get(name, {}).get("tested") is True
    ]


def find_cone_group(cone_name: str, destination_groups: dict):
    """
    Find which destination group contains cone_name.
    Returns (group_name, cone_data) or None if not found.
    """
    for group_name, group in destination_groups.items():
        cones = group.get("cones", {})
        if cone_name in cones:
            return group_name, cones[cone_name]
    return None


def validate_sequence(sequence_names: list, edge_cache: dict) -> list:
    """
    Check every consecutive edge in sequence_names against edge_cache.
    Returns list of problem strings (empty = all clear).
    """
    problems = []
    for i in range(len(sequence_names) - 1):
        f = sequence_names[i]
        t = sequence_names[i + 1]
        key = f"{f}|{t}"
        if key not in edge_cache:
            problems.append(f"Edge {f} → {t}: not tested. Re-run check_collision_free_paths.py.")
        elif not edge_cache[key]["collision_free"]:
            problems.append(f"Edge {f} → {t}: collision detected in simulation.")
    return problems


def build_sequence_names(
    base_cone_name: str,
    dest_cone_name: str,
    plan: dict,
    routing_candidates: list,
) -> list:
    """
    Build the ordered list of node names for a complete pick-and-place.

    Sequence:
      home → [path to base_gateway] → base_approach → base_grab → base_approach
           → [path back to base_gateway] → [path to dest_gateway]
           → dest_approach → dest_grab → dest_approach
           → [path back to dest_gateway] → [path to home]

    Uses find_shortest_path over the routing_candidates subgraph.
    Raises RuntimeError if no path exists between any required pair.
    """
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from check_collision_free_paths import find_shortest_path

    edge_cache = plan["edge_cache"]
    base_info = plan["base_cones"][base_cone_name]
    group_result = find_cone_group(dest_cone_name, plan["destination_groups"])
    if group_result is None:
        raise RuntimeError(f"Destination cone '{dest_cone_name}' not found in any group in path_plan.yaml.")
    group_name, dest_info = group_result
    group = plan["destination_groups"][group_name]

    base_gateway = base_info["gateways"][0]
    dest_gateways = group["gateways"]

    # Build collision-free edges over routing candidates only
    candidate_edges = set()
    for f in routing_candidates:
        for t in routing_candidates:
            if f != t and edge_cache.get(f"{f}|{t}", {}).get("collision_free"):
                candidate_edges.add((f, t))

    def _path(start, end):
        p = find_shortest_path(candidate_edges, start, end)
        if p is None:
            raise RuntimeError(
                f"No tested collision-free path from '{start}' to '{end}'. "
                "Add more routing_candidates and re-run check_collision_free_paths.py."
            )
        return p

    # Find which dest gateway has a tested path from base_gateway
    dest_gateway = None
    for dg in dest_gateways:
        try:
            _path(base_gateway, dg)
            dest_gateway = dg
            break
        except RuntimeError:
            continue
    if dest_gateway is None:
        raise RuntimeError(
            f"No tested collision-free path from base gateway '{base_gateway}' "
            f"to any dest gateway {dest_gateways}. "
            "Re-run check_collision_free_paths.py with more routing_candidates."
        )

    base_ak = f"{base_cone_name}_approach"
    base_gk = f"{base_cone_name}_grab"
    dest_ak = f"{dest_cone_name}_approach"
    dest_gk = f"{dest_cone_name}_grab"

    seq = []
    seq += _path("home", base_gateway)
    seq.append(base_ak)
    seq.append(base_gk)
    seq.append(base_ak)                         # retract
    seq += _path(base_gateway, dest_gateway)[1:]  # skip base_gateway (already there)
    seq.append(dest_ak)
    seq.append(dest_gk)
    seq.append(dest_ak)                         # retract
    seq += list(reversed(_path("home", dest_gateway)))[1:]  # reverse back to home
    return seq
```

- [ ] **Step 4: Run integration tests**

```bash
pytest tests/test_path_plan_integration.py -v
```
Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add robodk_code/moving_a_cone.py tests/test_path_plan_integration.py
git commit -m "feat: add plan loading, sequence building, and edge validation to moving_a_cone"
```

---

## Task 8: `moving_a_cone.py` — wire plan into `main()` execution

**Files:**
- Modify: `robodk_code/moving_a_cone.py` (modify `main()` to load plan, filter cones, build and validate sequence, execute with via-points)

No new tests — the integration is RoboDK-dependent. Verify manually.

- [ ] **Step 1: Add plan loading at the top of `main()`**

In `main()`, after `RDK = connect()` and before Step 1 (base cones), insert:

```python
    # ── Load path plan ────────────────────────────────────────────────────────
    _plan_path = os.path.join(ROBODK_OUTPUT_DIR, "path_plan.yaml")
    _config_path = os.path.join(ROBODK_OUTPUT_DIR, "path_config.yaml")

    if os.path.isfile(_config_path):
        import sys as _sys
        _sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from check_collision_free_paths import load_config as _load_config, compute_config_hashes as _compute_hashes
        _current_hashes = _compute_hashes(_load_config(_config_path))
    else:
        _current_hashes = None
        print("[WARN] path_config.yaml not found — hash validation skipped.")

    path_plan = load_path_plan(_plan_path, expected_hashes=_current_hashes)
    _routing_candidates = path_plan.get("routing_candidates",
        list(path_plan.get("edge_cache", {}).keys()))  # fallback: infer from edge_cache
    # Re-derive routing_candidates from config if available
    if os.path.isfile(_config_path):
        from check_collision_free_paths import load_config as _lc
        _routing_candidates = _lc(_config_path).get("routing_candidates", ["home"])
    _log(f"[INFO] Path plan loaded (generated {path_plan.get('generated', '?')})")
```

- [ ] **Step 2: Filter base cones using plan, after base_ik_map is loaded (around line 633)**

Replace:
```python
        cone_names = [t.Name() for t in base_cones]
```
With:
```python
        all_cone_names = [t.Name() for t in base_cones]
        cone_names = filter_tested_cones(all_cone_names, path_plan["base_cones"])
        excluded = set(all_cone_names) - set(cone_names)
        if excluded:
            _log(f"[INFO] Excluded {len(excluded)} base cone(s) with no tested path: {sorted(excluded)}")
```

- [ ] **Step 3: Filter destination cones using plan, after dest_ik_map is computed (around line 663)**

Replace the display loop after `dest_ik_map = compute_dest_ik(...)`:
```python
    dest_cones = find_destination_cones(RDK)
    if not dest_cones:
        raise RuntimeError("No cone_grab_* targets found in station.")

    print(f"\nDestination cones (placement targets) — {len(dest_cones)} found:")
    for i, t in enumerate(dest_cones):
        print(f"  [{i}] {t.Name()}")

    dest_ik_map = compute_dest_ik(RDK, robot, dest_cones, tool, recompute=args.recompute_dest)
```
With:
```python
    dest_cones = find_destination_cones(RDK)
    if not dest_cones:
        raise RuntimeError("No cone_grab_* targets found in station.")

    dest_ik_map = compute_dest_ik(RDK, robot, dest_cones, tool, recompute=args.recompute_dest)

    # Filter to plan-tested cones only (flatten all groups)
    _all_dest_plan = {}
    for group in path_plan["destination_groups"].values():
        _all_dest_plan.update(group.get("cones", {}))
    dest_cones = [t for t in dest_cones if _all_dest_plan.get(t.Name(), {}).get("tested") is True]

    print(f"\nDestination cones (tested, collision-free) — {len(dest_cones)} available:")
    for i, t in enumerate(dest_cones):
        print(f"  [{i}] {t.Name()}")
```

- [ ] **Step 4: Replace the motion sequence in `main()` with plan-driven execution**

Find the `# ── Motion sequence` block (around line 738) and replace everything from `try:` through the `finally:` with:

```python
    try:
        # Build and validate the full sequence before any motion
        seq_names = build_sequence_names(
            base_cone_name=base_name,
            dest_cone_name=tgt_name,
            plan=path_plan,
            routing_candidates=_routing_candidates,
        )
        problems = validate_sequence(seq_names, path_plan["edge_cache"])
        if problems:
            _log("[ERROR] Sequence validation failed — refusing to move:")
            for p in problems:
                _log(f"         {p}")
            raise RuntimeError("Untested or colliding edges in planned sequence. Re-run check_collision_free_paths.py.")

        _log(f"\n[INFO] Sequence validated ({len(seq_names)} waypoints, all edges tested and clear):")
        for i, n in enumerate(seq_names):
            _log(f"  [{i:02d}] {n}")

        # Resolve node names to joint values
        def _joints_for(node_name: str) -> list:
            """Get joint values for a node from plan or from known structures."""
            # Routing candidates come from edge_cache (stored in each edge entry)
            # Try base cones
            if node_name.endswith("_approach"):
                base = node_name[:-len("_approach")]
                if base in path_plan["base_cones"] and path_plan["base_cones"][base].get("tested"):
                    return path_plan["base_cones"][base]["approach_joints"]
                # Destination cone
                for group in path_plan["destination_groups"].values():
                    if base in group.get("cones", {}):
                        return group["cones"][base]["approach_joints"]
            if node_name.endswith("_grab"):
                base = node_name[:-len("_grab")]
                if base in path_plan["base_cones"] and path_plan["base_cones"][base].get("tested"):
                    return path_plan["base_cones"][base]["grab_joints"]
                for group in path_plan["destination_groups"].values():
                    if base in group.get("cones", {}):
                        return group["cones"][base]["grab_joints"]
            # Routing candidate: get joints from any edge that starts at this node
            for key, entry in path_plan["edge_cache"].items():
                if key.startswith(f"{node_name}|"):
                    return entry["from_joints"]
            raise RuntimeError(f"Cannot resolve joints for node '{node_name}'")

        # Execute sequence with confirmation at key steps
        world_frame = RDK.Item("WorldFrame", ITEM_TYPE_FRAME)
        saved_frame = robot.getLink(ITEM_TYPE_FRAME)
        robot.setPoseFrame(world_frame)
        robot.setSpeed(speed_linear=SPEED_MM_S, speed_joints=SPEED_J_DEG_S)

        cone_attached = False
        base_grab_idx = seq_names.index(f"{base_name}_grab")
        dest_grab_idx = seq_names.index(f"{tgt_name}_grab")

        for i, node_name in enumerate(seq_names[:-1]):
            next_name = seq_names[i + 1]
            joints = _joints_for(next_name)
            label = f"[{i:02d}→{i+1:02d}] {node_name} → {next_name}"

            # Attach cone after base grab
            if i + 1 == base_grab_idx + 1 and not cone_attached:
                if cone_mesh is not None:
                    tcp_pose = robot.Pose()
                    parent_abs = cone_mesh_orig_parent.PoseAbs() if (cone_mesh_orig_parent and cone_mesh_orig_parent.Valid()) else eye(4)
                    cone_mesh.setPose(invH(parent_abs) * tcp_pose)
                    _attach_to = tool if tool.Valid() else robot
                    cone_mesh.setParentStatic(_attach_to)
                    RDK.Render(True)
                    cone_attached = True
                    _log(f"[INFO] Cone attached to '{_attach_to.Name()}'")

            # Detach cone and place at destination after dest grab
            if i + 1 == dest_grab_idx + 1 and cone_attached:
                if cone_mesh is not None:
                    wf_abs = world_frame.PoseAbs()
                    local_pose = invH(wf_abs) * tgt_grab_pose
                    cone_mesh.setParentStatic(world_frame)
                    cone_mesh.setPose(local_pose)
                    RDK.Render(True)
                    cone_attached = False
                    ax, ay, az = _pose_xyz(cone_mesh.PoseAbs())
                    tx, ty, tz = _pose_xyz(tgt_grab_pose)
                    err = math.sqrt((tx-ax)**2 + (ty-ay)**2 + (tz-az)**2)
                    _log(f"[INFO] Cone placed at destination (err={err:.2f}mm)")

            if not proceed(
                f"Step {i+1}/{len(seq_names)-1} — {next_name}",
                f"Moving to: {next_name}\nJoints: {fmt_joints(joints)}"
            ):
                _log(f"[ABORT] User cancelled at step {i+1}: {next_name}")
                return

            expected = None
            if next_name == f"{tgt_name}_approach":
                expected = tgt_app_pose
            elif next_name == f"{tgt_name}_grab":
                expected = tgt_grab_pose

            if not do_move(robot, joints, label, expected_pose=expected):
                return

        _log("\n[INFO] Pick-and-place complete.")

    finally:
        if saved_frame.Valid():
            robot.setPoseFrame(saved_frame)
        if _DEBUG_LOG is not None:
            _DEBUG_LOG.close()
```

- [ ] **Step 5: Test end-to-end with RoboDK open**

```bash
python robodk_code/moving_a_cone.py --mode ai --base 0 --dest 0
```
Expected:
- Plan loads, hashes checked
- Sequence printed with N waypoints
- Robot moves through via-points
- Cone mesh carried and placed correctly
- Debug log shows placement error < 1mm

- [ ] **Step 6: Run full test suite**

```bash
pytest tests/ -v
```
Expected: all tests pass.

- [ ] **Step 7: Commit**

```bash
git add robodk_code/moving_a_cone.py
git commit -m "feat: wire path plan into moving_a_cone.py — validated sequence execution"
```

---

## Task 9: Update CLAUDE.md resume point

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: Update the resume point section in `CLAUDE.md`**

Replace the `── RESUME POINT` section with:

```markdown
## ── RESUME POINT (left off 2026-05-18) ──────────────────────────────────────

**Branch:** `collision_free_path_planning`

**Status:** Collision-free path planner implemented and integrated into `moving_a_cone.py`.
Bug fix (cone placement coordinate frame) implemented on `determining_how_position_gripper`.

**What works:**
- `robot_controller.py`: MoveJTestModel, MoveJModel mixin pipeline
- `check_collision_free_paths.py`: builds node graph, tests all edges, finds gateways, writes path_plan.yaml
- `moving_a_cone.py`: loads plan, validates all edges before any motion, executes with via-points

**Next steps:**
- Merge `determining_how_position_gripper` into `collision_free_path_planning`
- Run `check_collision_free_paths.py` with real path_config.yaml to generate production path_plan.yaml
- Populate `path_config.yaml` with actual machine zones and gateway waypoints from Grasshopper
```

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: update CLAUDE.md resume point for path planner completion"
```
