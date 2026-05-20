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
