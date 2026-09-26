import json
from pathlib import Path

import pytest

from rebot_teach_mode.action_library import (
    ActionGroupLibrary,
    action_filename,
    validate_action_name,
)
from rebot_teach_mode.trajectory import RecordedPoint, RecordedTrajectory


NAMES = tuple(f"joint{index}" for index in range(1, 7))


def trajectory(names=NAMES):
    return RecordedTrajectory(
        names,
        (
            RecordedPoint(0.0, (0.0,) * 6),
            RecordedPoint(1.0, (0.1,) * 6),
        ),
        "2026-08-25T00:00:00+00:00",
    )


def shape_parameters():
    return {
        "kind": "shape",
        "shape": "text",
        "source": "HELLO",
        "pose": {
            "position": [0.28, 0.0, 0.12],
            "orientation": [0.0, 0.0, 0.0, 2.0],
        },
        "width": 0.17,
        "height": 0.08,
        "pen_length_m": 0.117,
        "pen_lift_m": 0.064,
    }


def test_unicode_name_has_a_stable_safe_filename():
    filename = action_filename("挥手 动作")
    assert filename.endswith(".teach.json")
    assert "/" not in filename
    assert filename == action_filename("挥手 动作")


@pytest.mark.parametrize(
    "name", ["", ".", "..", "bad/name", "bad\\name", "x\ny"]
)
def test_unsafe_names_are_rejected(name):
    with pytest.raises(ValueError):
        validate_action_name(name)


def test_save_load_list_and_overwrite_guard(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    info = library.save("挥手", "观众问候", trajectory())
    assert info.name == "挥手"
    assert info.robot_model == "rebotarm_rs"
    assert info.path.is_file()

    loaded_info, loaded = library.load("挥手")
    assert loaded_info.description == "观众问候"
    assert loaded == trajectory()
    groups, warnings = library.list_groups()
    assert [item.name for item in groups] == ["挥手"]
    assert warnings == []

    with pytest.raises(FileExistsError):
        library.save("挥手", "重复", trajectory())
    library.save("挥手", "已覆盖", trajectory(), overwrite=True)
    assert library.load("挥手")[0].description == "已覆盖"


def test_shape_source_parameters_round_trip_and_remain_optional(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    saved = library.save(
        "可编辑字符", "", trajectory(), shape_parameters=shape_parameters()
    )

    assert saved.shape_parameters["shape"] == "text"
    assert saved.shape_parameters["source"] == "HELLO"
    assert saved.shape_parameters["pose"]["orientation"] == [0.0, 0.0, 0.0, 1.0]
    assert library.list_groups()[0][0].shape_parameters == saved.shape_parameters

    legacy = library.save("拖动录制", "", trajectory())
    assert legacy.shape_parameters is None


def test_invalid_shape_source_parameters_are_rejected(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    invalid = shape_parameters()
    invalid["width"] = 0.0
    with pytest.raises(ValueError, match="width and height"):
        library.save("无效参数", "", trajectory(), shape_parameters=invalid)


def test_wrong_joint_layout_is_rejected(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    with pytest.raises(ValueError, match="joint names"):
        library.save("wrong", "", trajectory(tuple(reversed(NAMES))))


def test_invalid_trajectory_is_rejected_before_file_creation(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    invalid = RecordedTrajectory(
        NAMES,
        (
            RecordedPoint(0.0, (0.0,) * 6),
            RecordedPoint(float("nan"), (0.1,) * 6),
        ),
        "",
    )
    with pytest.raises(ValueError, match="invalid time"):
        library.save("invalid", "", invalid)
    assert not library.path_for("invalid").exists()


def test_corrupt_and_wrong_model_files_are_reported(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    info = library.save("model", "", trajectory())
    document = json.loads(info.path.read_text(encoding="utf-8"))
    document["robot_model"] = "another_arm"
    info.path.write_text(json.dumps(document), encoding="utf-8")
    (tmp_path / "broken.teach.json").write_text("{", encoding="utf-8")

    groups, warnings = library.list_groups()
    assert groups == []
    assert len(warnings) == 2
    with pytest.raises(ValueError, match="robot model"):
        library.load("model")


def test_named_sequence_preserves_order_and_allows_repeated_actions(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    library.save("拿起", "", trajectory())
    library.save("放下", "", trajectory())

    saved = library.save_sequence("搬运一轮", ["拿起", "放下", "拿起"])

    assert saved.action_names == ("拿起", "放下", "拿起")
    assert library.load_sequence("搬运一轮") == saved
    sequences, warnings = library.list_sequences()
    assert sequences == [saved]
    assert warnings == []


def test_sequence_rejects_a_missing_action(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    with pytest.raises(FileNotFoundError, match="does not exist"):
        library.save_sequence("无效动作组", ["不存在"])


def test_rename_updates_every_sequence_reference(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    original = library.save(
        "旧名字", "保留描述", trajectory(), shape_parameters=shape_parameters()
    )
    library.save_sequence("组合", ["旧名字", "旧名字"])

    renamed = library.rename("旧名字", "新名字")

    assert renamed.name == "新名字"
    assert renamed.description == "保留描述"
    assert renamed.shape_parameters == original.shape_parameters
    assert not library.path_for("旧名字").exists()
    assert library.load_sequence("组合").action_names == ("新名字", "新名字")


def test_delete_removes_an_unreferenced_action(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    saved = library.save("待删除", "", trajectory())

    deleted = library.delete("待删除")

    assert deleted == saved
    assert not saved.path.exists()
    with pytest.raises(FileNotFoundError, match="does not exist"):
        library.load("待删除")


def test_delete_rejects_an_action_referenced_by_a_sequence(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    saved = library.save("被引用", "", trajectory())
    library.save_sequence("生产流程", ["被引用"])

    with pytest.raises(ValueError, match="生产流程"):
        library.delete("被引用")

    assert saved.path.exists()
    assert library.load_sequence("生产流程").action_names == ("被引用",)


def test_copy_retains_trajectory_description_and_shape_parameters(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    source = library.save(
        "原动作", "保留说明", trajectory(), shape_parameters=shape_parameters()
    )

    copied = library.copy("原动作", "原动作 副本")

    copied_info, copied_trajectory = library.load("原动作 副本")
    assert copied == copied_info
    assert copied_info.description == source.description
    assert copied_info.shape_parameters == source.shape_parameters
    assert copied_trajectory == trajectory()
    assert source.path.exists()


def test_copy_rejects_an_existing_or_unchanged_name(tmp_path):
    library = ActionGroupLibrary(tmp_path, NAMES)
    library.save("原动作", "", trajectory())
    library.save("已存在", "", trajectory())

    with pytest.raises(ValueError, match="different name"):
        library.copy("原动作", "原动作")
    with pytest.raises(FileExistsError, match="already exists"):
        library.copy("原动作", "已存在")


@pytest.mark.parametrize(
    ("sequence_name", "action_prefix"),
    [
        ("自动手眼标定_DM_12姿态", "手眼标定姿态_"),
        ("自动手眼标定_DM_眼在手上_12姿态", "眼在手上标定姿态_"),
    ],
)
def test_bundled_dm_handeye_sequences_contain_twelve_stationary_safe_poses(
    sequence_name, action_prefix
):
    root = Path(__file__).parents[1] / "action_groups" / "dm"
    library = ActionGroupLibrary(root, NAMES, robot_model="rebotarm_dm")

    sequence = library.load_sequence(sequence_name)

    assert len(sequence.action_names) == 12
    assert len(set(sequence.action_names)) == 12
    assert sequence.action_names == tuple(
        f"{action_prefix}{index:02d}" for index in range(1, 13)
    )
    poses = []
    lower = (-2.8, -3.14, -3.14, -1.87, -1.57, -3.14)
    upper = (2.8, 0.005, 0.005, 1.57, 1.57, 3.14)
    for name in sequence.action_names:
        info, saved = library.load(name)
        assert info.robot_model == "rebotarm_dm"
        assert saved.duration >= 3.0
        assert saved.points[0].positions == saved.points[-1].positions
        assert all(
            minimum <= value <= maximum
            for value, minimum, maximum in zip(
                saved.points[-1].positions, lower, upper
            )
        )
        poses.append(saved.points[-1].positions)
    assert len(set(poses)) == 12
