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
