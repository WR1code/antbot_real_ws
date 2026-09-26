from rebot_teach_mode.action_library import ActionGroupLibrary
from rebot_teach_mode.piperh_handeye_presets import (
    PIPERH_EYE_IN_HAND_ACTION_PREFIX,
    PIPERH_EYE_IN_HAND_SEQUENCE,
    PIPERH_EYE_ON_BASE_SEQUENCE,
    ensure_piperh_handeye_presets,
)


NAMES = tuple(f"joint{index}" for index in range(1, 7))
LOWER = (-2.618, 0.0, -2.96706, -2.356195, -1.56207, -3.14)
UPPER = (2.618, 3.14, 0.0, 2.356195, 1.56207, 3.14)


def test_piperh_handeye_presets_install_safe_multi_pose_sequences(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES, robot_model="piperh")
    ensure_piperh_handeye_presets(library)

    for sequence_name, prefix, pose_count in (
        (PIPERH_EYE_ON_BASE_SEQUENCE, "手眼标定姿态_", 12),
        (PIPERH_EYE_IN_HAND_SEQUENCE, PIPERH_EYE_IN_HAND_ACTION_PREFIX, 18),
    ):
        sequence = library.load_sequence(sequence_name)
        assert sequence.action_names == tuple(
            f"{prefix}{index:02d}" for index in range(1, pose_count + 1)
        )
        poses = []
        for action_name in sequence.action_names:
            info, trajectory = library.load(action_name)
            assert info.robot_model == "piperh"
            assert trajectory.duration == 3.0
            assert trajectory.points[0].positions == trajectory.points[-1].positions
            assert all(
                lower <= value <= upper
                for value, lower, upper in zip(
                    trajectory.points[0].positions, LOWER, UPPER
                )
            )
            poses.append(trajectory.points[0].positions)
        assert len(set(poses)) == pose_count
        if sequence_name == PIPERH_EYE_IN_HAND_SEQUENCE:
            # The wrist camera is fixed to Link5, upstream of joint6.  Moving
            # joint6 cannot provide hand-eye motion and only corrupts a Link6
            # calibration model.
            assert all(pose[5] == 0.0 for pose in poses)


def test_piperh_preset_install_is_idempotent(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES, robot_model="piperh")
    ensure_piperh_handeye_presets(library)
    original = library.path_for("手眼标定姿态_01").read_bytes()
    ensure_piperh_handeye_presets(library)
    assert library.path_for("手眼标定姿态_01").read_bytes() == original
