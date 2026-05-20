"""
dhr_robot.py

Port of DHR's knitwear-cell/src/main/robot/robot.py.

Stripped of: @autowired DI -> module-level setup(), Redis -> no caching,
loguru -> print(), Station wrapper -> direct rdk.Item() calls,
TaskMessage/TaskStatus -> bool, numpy -> float().

Call dhr_robot.setup(rdk, robot_item) once before executing any State.
All kinematics models then use _rdk and _robot from this module.

XQuery-generated state classes use:
    robot_controller: Robot = Field(default_factory=lambda: Robot(KinematicsModel, MoveModel))
The Robot() factory requires no args at construction time -- setup() wires in
the live RoboDK references separately.
"""
import sys
sys.path.append("C:/RoboDK/Python")

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from robodk.robomath import Pose_2_TxyzRxyz

if TYPE_CHECKING:
    from state import State

# ── Module-level RoboDK registry ──────────────────────────────────────────────
_rdk = None
_robot = None

RAIL_JOINT_LIMIT_CLEARANCE_MM = 1

_AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}

# Base OptimAxes dict -- AbsJnt_7 is set per-call by OptimizationKinematicsModel.
_OPT_AXES_BASE = {
    "AbsOn_7":   1,   "AbsW_7":   100,
    "Algorithm": 3,   "MaxIter":  500, "Tol": 0.001,
    "RelOn_1": 1, "RelOn_2": 1, "RelOn_3": 1, "RelOn_4": 1,
    "RelOn_5": 1, "RelOn_6": 1, "RelOn_7": 1,
    "RelW_1": 50, "RelW_2": 50, "RelW_3": 50, "RelW_4": 50,
    "RelW_5": 50, "RelW_6": 50, "RelW_7": 50,
}


def setup(rdk, robot_item):
    """Wire in the live RoboDK session. Call once before executing any State."""
    global _rdk, _robot
    _rdk = rdk
    _robot = robot_item


def _clamp_rail(requested: float) -> float:
    """Clamp a requested rail position to the robot's joint limits."""
    try:
        jl_min, jl_max = _robot.JointLimits()
        lower = jl_min.tolist()[0] if hasattr(jl_min, "tolist") else list(jl_min)
        upper = jl_max.tolist()[0] if hasattr(jl_max, "tolist") else list(jl_max)
        return max(
            lower[-1] + RAIL_JOINT_LIMIT_CLEARANCE_MM,
            min(requested, upper[-1] - RAIL_JOINT_LIMIT_CLEARANCE_MM),
        )
    except Exception:
        return requested


def _avoid_zero_rail(robot):
    """RoboDK OptimAxes fails at j7=0.0 exactly -- nudge to 0.001 if needed."""
    current = robot.Joints().list()
    if len(current) == 7 and current[6] == 0.0:
        current[6] = 0.001
        robot.setJoints(current)


# ── Mixin base ────────────────────────────────────────────────────────────────

class RobotControllerMixin(ABC):
    def __init__(self, parent: "Robot"):
        self._parent = parent

    def execute(self, state: "State") -> None:
        self.concrete_execute(state)

    @abstractmethod
    def concrete_execute(self, state: "State") -> None:
        pass


# ── Kinematics models (set _target) ──────────────────────────────────────────

class AbsoluteJointKinematicsModel(RobotControllerMixin):
    """Move to explicit joint values from state.move_absolute_joints (th1-th7).
    Joints left as None keep the robot's current value.
    """
    def concrete_execute(self, state: "State") -> None:
        current = _robot.Joints().list()
        if state.move_absolute_joints is not None:
            for i in range(len(current)):
                val = getattr(state.move_absolute_joints, f"th{i+1}", None)
                if val is not None:
                    current[i] = val
        self._parent._target = current


class FrameKinematicsModel(RobotControllerMixin):
    """Standard 7-DOF IK to a named RoboDK frame (no j7 constraint).
    Sets pose frame to WorldFrame; _target is the frame's absolute pose.
    """
    def concrete_execute(self, state: "State") -> None:
        frame = _rdk.Item(state.frame)
        pose = frame.PoseAbs()

        if state.tool_name is not None:
            _robot.setPoseTool(_rdk.Item(state.tool_name))

        _robot.setPoseFrame(_rdk.Item("WorldFrame"))
        _avoid_zero_rail(_robot)

        self._parent._target = pose


class OptimizationKinematicsModel(RobotControllerMixin):
    """IK to a named frame with j7 constrained to the frame's rail-axis position.
    Mirrors DHR's OptimizationKinematicsModel (OptimAxes Algorithm 3 / DLS).

    state.optimization_axis: 'X', 'Y', or 'Z' -- which world axis the rail runs on.
    state.optimization_frame: optional override for the frame whose position drives j7.
    """
    def concrete_execute(self, state: "State") -> None:
        axis_idx = _AXIS_INDEX.get(state.optimization_axis or "X", 0)

        frame = _rdk.Item(state.frame)
        pose_abs = frame.PoseAbs()

        if state.optimization_frame is not None:
            opt_frame = _rdk.Item(state.optimization_frame)
            coords = Pose_2_TxyzRxyz(opt_frame.PoseAbs())
        else:
            coords = Pose_2_TxyzRxyz(pose_abs)

        requested_rail = coords[axis_idx]
        clamped_rail = _clamp_rail(requested_rail)

        props = dict(_OPT_AXES_BASE)
        props["AbsJnt_7"] = clamped_rail

        if state.tool_name is not None:
            _robot.setPoseTool(_rdk.Item(state.tool_name))

        _robot.setParam("OptimAxes", props)
        _robot.setPoseFrame(_rdk.Item("WorldFrame"))
        _avoid_zero_rail(_robot)

        self._parent._target = pose_abs


class TargetKinematicsModel(RobotControllerMixin):
    """Move to a named RoboDK joint or Cartesian target item."""
    def concrete_execute(self, state: "State") -> None:
        name = state.target
        if name is None and state.parameters is not None and state.parameters.targets:
            name = state.parameters.targets[0]

        item = _rdk.Item(name)
        self._parent._target = item

        if not item.isJointTarget():
            _robot.setPoseFrame(item.Parent())

        if state.tool_name is not None:
            _robot.setPoseTool(_rdk.Item(state.tool_name))


class FrameKinematicsOnlyRailModel(RobotControllerMixin):
    """Move only j7 (the linear rail) to the frame's position on the rail axis.
    All arm joints (j1-j6) keep their current values.
    """
    def concrete_execute(self, state: "State") -> None:
        axis_idx = _AXIS_INDEX.get(state.optimization_axis or "X", 0)

        frame = _rdk.Item(state.frame)
        coords = Pose_2_TxyzRxyz(frame.PoseAbs())
        requested_rail = coords[axis_idx]
        clamped = _clamp_rail(requested_rail)

        current = _robot.Joints().list()
        current[-1] = clamped
        self._parent._target = current


# ── Movement models (consume _target) ────────────────────────────────────────

class MoveJModel(RobotControllerMixin):
    """Execute robot.MoveJ(_target) if no collision was detected."""
    def concrete_execute(self, state: "State") -> None:
        if not self._parent._test_result:
            return
        robot = _rdk.Item(state.robot) if state.robot else _robot
        if state.tool_name is not None:
            robot.setPoseTool(_rdk.Item(state.tool_name))
        robot.MoveJ(self._parent._target)


class MoveLModel(RobotControllerMixin):
    """Execute robot.MoveL(_target) if no collision was detected."""
    def concrete_execute(self, state: "State") -> None:
        if not self._parent._test_result:
            return
        robot = _rdk.Item(state.robot) if state.robot else _robot
        robot.MoveL(self._parent._target)


class MoveJTestModel(RobotControllerMixin):
    """Test a joint move for collisions (in-memory, no Redis).
    Sets _test_result = False on collision; restores joints after test.
    """
    def concrete_execute(self, state: "State") -> None:
        current = _robot.Joints()
        result = _robot.MoveJ_Test(current, self._parent._target)
        _robot.setJoints(current)
        if result != 0:
            print(f"[WARN] MoveJTestModel: collision detected (code={result})")
            self._parent._test_result = False


class MoveLTestModel(RobotControllerMixin):
    """Test a linear move for collisions.
    Sets _test_result = False on collision; restores joints after test.
    """
    def concrete_execute(self, state: "State") -> None:
        current = _robot.Joints()
        result = _robot.MoveL_Test(current, self._parent._target)
        _robot.setJoints(current)
        if result != 0:
            print(f"[WARN] MoveLTestModel: collision detected (code={result})")
            self._parent._test_result = False


# ── Robot (the mixin pipeline) ────────────────────────────────────────────────

class Robot:
    """Composes RobotControllerMixin classes into a sequential pipeline.

    Matches DHR's Robot class interface so XQuery-generated state classes
    (robot_controller: Robot = Field(default_factory=lambda: Robot(A, B)))
    work without modification.

    setup() must be called before the first execute().
    """

    def __init__(self, *mixin_classes):
        self._test_result = True   # True = OK, False = collision / error
        self._target = None        # set by kinematics models, consumed by move models
        self._target_pose = None   # spare slot used by some models
        self._mixins = [m(self) for m in mixin_classes]

    def execute(self, state: "State") -> bool:
        """Run all mixins in order. Returns True on success, False on any error."""
        self._test_result = True
        self._target = None
        self._target_pose = None
        for mixin in self._mixins:
            mixin.execute(state)
            if not self._test_result:
                break
        return self._test_result
