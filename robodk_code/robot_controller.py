"""
robot_controller.py

Mixin pipeline modelled on knitwear-cell robot.py.
Used by check_collision_free_paths.py (PathEvaluationModel) and
moving_a_cone.py (MoveJModel).

Usage:
    checker = Robot(PathEvaluationModel)
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
          - collision_free (bool): set by PathEvaluationModel, read by MoveJModel
          - robot: RoboDK Item (robot)
        """


class PathEvaluationModel(RobotControllerMixin):
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
        checker = Robot(PathEvaluationModel)
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
