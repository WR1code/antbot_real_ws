from rebot_xbox_servo.safety import (
    SafetyState,
    SpeedSelector,
    a_button_rising_edge,
    bounded_tier_speed,
)


def test_a_button_only_toggles_on_rising_edges():
    state = SafetyState()
    previous = 0
    observed = []
    for current in [0, 1, 1, 1, 0, 0, 1, 1, 0, 1]:
        if a_button_rising_edge(current, previous):
            observed.append(state.toggle(True, True))
        previous = current
    assert observed == ["armed", "locked", "armed"]
    assert state.armed is True


def test_centered_sticks_are_required_before_arming():
    state = SafetyState()
    assert state.toggle(False, True) == "center_required"
    assert state.armed is False
    assert state.toggle(True, True) == "armed"


def test_timeout_locks_and_reconnect_does_not_rearm():
    state = SafetyState(armed=True)
    state.timeout()
    assert state.armed is False
    assert state.joystick_timed_out is True
    state.reconnect()
    assert state.armed is False
    assert state.joystick_timed_out is False


def test_emergency_stop_locks():
    state = SafetyState(armed=True)
    state.emergency_stop()
    assert state.armed is False


def test_speed_selector_is_bounded_and_starts_low():
    selector = SpeedSelector([0.25, 0.50, 1.0])
    assert selector.percentage == 25
    assert selector.increase() is True
    assert selector.percentage == 50
    assert selector.increase() is True
    assert selector.percentage == 100
    assert selector.increase() is False
    assert selector.decrease() is True
    assert selector.percentage == 50


def test_tier_speed_never_exceeds_yaml_maximum():
    assert bounded_tier_speed(1.0, 1.0, 0.005, 0.08) == 0.08
    assert bounded_tier_speed(0.01, 0.25, 0.005, 0.08) == 0.005
