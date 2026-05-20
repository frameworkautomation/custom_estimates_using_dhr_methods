"""
state_machine.py

Stripped-down port of DHR's knitwear-cell/src/main/robot/state_machine.py.
Removed: @autowired DI, Redis, loguru, CombinedContainer / create_model.

Usage:
    import dhr_robot
    from state_machine import StateMachine
    from generated_states import StatesContainer   # from XQuery output

    dhr_robot.setup(rdk, robot_item)
    sm = StateMachine(StatesContainer())

    sm.set_state("some_state_1")     # attribute name on the container
    ok = sm.handle()                 # executes current state, returns bool

State names on the container follow the pattern the XQuery generates:
    frame name "CurtainSafeMachine1" with 2 states ->
    "curtain_safe_machine_1_1" and "curtain_safe_machine_1_2"
(see local:to-snake-case and local:generate-container-field in yaml_to_state_class.xq)
"""
from typing import Any

from state import State


class StateMachine:
    """Manages robot state transitions and execution."""

    in_process: bool = False

    def __init__(self, states_container: Any):
        """
        Args:
            states_container: A Pydantic BaseModel whose attributes are State instances.
                              Typically a StatesContainer from generated_states.py, or a
                              combined model merging generated and hand-written states.
        """
        self._states = states_container
        self._current_state: State = None
        self._previous_state: State = None

    def set_state(self, trigger: str) -> None:
        """Transition to a state by its attribute name on the container."""
        self._previous_state = self._current_state
        self._current_state = getattr(self._states, trigger)

    def get_current_state(self) -> State:
        return self._current_state

    def get_previous_state(self) -> State:
        return self._previous_state

    def handle(self) -> bool:
        """Execute the current state. Returns True on success, False on error."""
        return self._current_state.handle()

    def set_parameters(self, parameters: Any) -> None:
        """Inject runtime parameters into the current state before handle()."""
        self._current_state.parameters = parameters
