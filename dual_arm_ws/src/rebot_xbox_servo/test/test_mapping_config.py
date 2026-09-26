"""Protect the measured physical Xbox mapping from accidental regression."""

from pathlib import Path

import yaml


MAPPING_PATH = Path(__file__).parents[1] / 'config' / 'xbox_mapping.yaml'


def load_parameters():
    """Load the production control and physical mapping records."""
    with MAPPING_PATH.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    return (
        document['rebot_xbox_twist']['ros__parameters'],
        document['xbox_physical_mapping']['ros__parameters'],
    )


def test_measured_axis_and_button_indices():
    """Require the exact indices measured from this Generic X-Box pad."""
    control, physical = load_parameters()
    assert physical['axes'] == {
        'left_stick_x': 0,
        'left_stick_y': 1,
        'left_trigger': 2,
        'right_stick_x': 3,
        'right_stick_y': 4,
        'right_trigger': 5,
        'dpad_x': 6,
        'dpad_y': 7,
    }
    assert physical['buttons'] == {
        'a': 0,
        'b': 1,
        'x': 2,
        'y': 3,
        'lb': 4,
        'rb': 5,
        'back': 6,
        'start': 7,
        'xbox_mode': 8,
        'left_stick_press': 9,
        'right_stick_press': 10,
    }
    assert control['axes'] == {
        'linear_x': 1,
        'linear_y': 0,
        'linear_z': 4,
        'angular_yaw': 3,
        'angular_pitch': 7,
    }
    assert control['buttons'] == {
        'toggle_arm': 0,
        'emergency_stop': 7,
        'roll_left': 4,
        'roll_right': 5,
        'speed_down': 9,
        'speed_up': 10,
        'frame_switch': 6,
        'control_switch': 8,
    }
    assert control['triggers'] == {
        'gripper_open_axis': 2,
        'gripper_close_axis': 5,
        'deadzone': 0.05,
    }
    assert control['limits']['linear_speed'] == 0.08
    assert control['limits']['maximum_linear_speed'] == 0.08
    assert control['limits']['angular_speed'] == 0.40
    assert control['limits']['maximum_angular_speed'] == 0.50


def test_motion_axes_center_at_zero_and_use_requested_directions():
    """Protect centered axes and the user-requested stick reversal."""
    control, physical = load_parameters()
    directions = physical['axis_directions']
    motion_indices = set(control['axes'].values())
    assert physical['axes']['left_trigger'] not in motion_indices
    assert physical['axes']['right_trigger'] not in motion_indices
    stick_axes = (
        'left_stick_x', 'left_stick_y',
        'right_stick_x', 'right_stick_y',
    )
    for name in stick_axes:
        assert 'centered=0.0' in directions[name]
    assert 'released=0.0' in directions['dpad_x']
    assert 'released=0.0' in directions['dpad_y']
    assert control['invert'] == {
        'linear_x': False,
        'linear_y': False,
        'linear_z': False,
        'angular_yaw': False,
        'angular_pitch': True,
    }


def test_startup_pose_is_slow_and_servo_safe():
    """Protect the six-axis sleep-to-safe initialization contract."""
    with MAPPING_PATH.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    startup = document['rebot_xbox_arm_initializer']['ros__parameters']
    assert startup['joint_names'] == [f'joint{index}' for index in range(1, 7)]
    assert startup['target_positions'] == [0.0, 1.75, 0.7, -0.7, 0.0, 0.0]
    assert startup['move_duration'] >= 6.0
    assert startup['controller_wait_timeout'] >= 30.0
    assert startup['completion_timeout'] > startup['move_duration']
