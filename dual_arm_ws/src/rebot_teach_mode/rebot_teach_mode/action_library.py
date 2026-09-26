"""Named, atomic action-group storage for validated teaching trajectories."""

from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from .trajectory import RecordedPoint, RecordedTrajectory, TrajectoryRecorder


LIBRARY_FORMAT = "rebot_teach_action_group"
LIBRARY_VERSION = 1
SEQUENCE_FORMAT = "rebot_teach_action_sequence"
SEQUENCE_VERSION = 1


@dataclass(frozen=True)
class ActionGroupInfo:
    name: str
    description: str
    robot_model: str
    path: Path
    created_utc: str
    duration_sec: float
    point_count: int
    shape_parameters: dict | None = None


@dataclass(frozen=True)
class ActionSequenceInfo:
    name: str
    robot_model: str
    path: Path
    action_names: tuple[str, ...]


def validate_action_name(value: str) -> str:
    name = unicodedata.normalize("NFKC", str(value)).strip()
    if not name or len(name) > 64:
        raise ValueError("action group name must contain 1 to 64 characters")
    if name in {".", ".."} or any(
        character in name for character in ("\x00", "\n", "\r", "\t", "/", "\\")
    ):
        raise ValueError("action group name contains an unsafe character")
    return name


def action_filename(name: str) -> str:
    normalized = validate_action_name(name)
    readable = "".join(
        character if character.isalnum() or character in {"-", "_"} else "_"
        for character in normalized
    ).strip("_")
    if not readable:
        readable = "action"
    digest = hashlib.sha256(normalized.encode("utf-8")).hexdigest()[:8]
    return f"{readable[:48]}-{digest}.teach.json"


def sequence_filename(name: str) -> str:
    normalized = validate_action_name(name)
    readable = "".join(
        character if character.isalnum() or character in {"-", "_"} else "_"
        for character in normalized
    ).strip("_")
    if not readable:
        readable = "sequence"
    digest = hashlib.sha256(normalized.encode("utf-8")).hexdigest()[:8]
    return f"{readable[:48]}-{digest}.sequence.json"


def _trajectory_document(
    name: str,
    description: str,
    robot_model: str,
    trajectory: RecordedTrajectory,
    shape_parameters: Mapping | None = None,
) -> dict:
    document = {
        "format": LIBRARY_FORMAT,
        "version": LIBRARY_VERSION,
        "name": name,
        "description": description,
        "robot_model": robot_model,
        "created_utc": trajectory.created_utc,
        "joint_names": list(trajectory.joint_names),
        "duration_sec": trajectory.duration,
        "timing_valid": trajectory.timing_valid and all(point.timing_valid for point in trajectory.points),
        "joint_limits_valid": trajectory.joint_limits_valid,
        "points": [
            {
                "time_from_start": point.time_from_start,
                "positions": list(point.positions),
            } | ({"feedback_timestamp": point.feedback_timestamp}
                 if point.feedback_timestamp is not None else {})
              | ({field: getattr(point, field) for field in
                  ("timestamp_j12", "timestamp_j34", "timestamp_j56",
                   "callback_monotonic_time", "intra_cycle_span")
                  if getattr(point, field) is not None})
              | ({"timing_valid": False} if not point.timing_valid else {})
            for point in trajectory.points
        ],
    }
    normalized_shape_parameters = _normalize_shape_parameters(shape_parameters)
    if normalized_shape_parameters is not None:
        document["shape_parameters"] = normalized_shape_parameters
    return document


def _finite_values(value, count: int, field: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"shape parameter {field} must contain {count} values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"shape parameter {field} contains a non-finite value")
    return result


def _normalize_shape_parameters(value: Mapping | None) -> dict | None:
    """Validate optional source parameters while keeping old action files readable."""
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise ValueError("shape_parameters must be a mapping")
    if str(value.get("kind", "")) != "shape":
        raise ValueError("shape_parameters kind must be 'shape'")
    shape = str(value.get("shape", "")).strip().lower()
    if shape not in {"rectangle", "triangle", "circle", "star", "heart", "text", "image"}:
        raise ValueError(f"unsupported editable shape {shape!r}")
    source = str(value.get("source", ""))
    pose = value.get("pose")
    if not isinstance(pose, Mapping):
        raise ValueError("shape parameter pose must be a mapping")
    position = _finite_values(pose.get("position"), 3, "pose.position")
    orientation = _finite_values(pose.get("orientation"), 4, "pose.orientation")
    norm = math.sqrt(sum(item * item for item in orientation))
    if norm <= 1e-9:
        raise ValueError("shape parameter pose.orientation must be non-zero")
    orientation = [item / norm for item in orientation]
    width = float(value.get("width", 0.0))
    height = float(value.get("height", 0.0))
    pen_length_m = float(value.get("pen_length_m", -1.0))
    pen_lift_m = float(value.get("pen_lift_m", 0.0))
    if not all(math.isfinite(item) for item in (width, height, pen_length_m, pen_lift_m)):
        raise ValueError("shape dimensions must be finite")
    if width <= 0.0 or height <= 0.0:
        raise ValueError("shape width and height must be positive")
    if pen_length_m < 0.0 or pen_lift_m <= 0.0:
        raise ValueError("pen length must be non-negative and pen lift must be positive")
    return {
        "kind": "shape",
        "shape": shape,
        "source": source,
        "pose": {"position": position, "orientation": orientation},
        "width": width,
        "height": height,
        "pen_length_m": pen_length_m,
        "pen_lift_m": pen_lift_m,
    }


def _validate_trajectory_data(
    trajectory: RecordedTrajectory, expected_names: Sequence[str]
) -> None:
    if trajectory.joint_names != tuple(expected_names):
        raise ValueError("trajectory joint names do not match action library")
    if len(trajectory.points) < 2:
        raise ValueError("action group must contain at least two points")
    previous_time = -1.0
    for index, point in enumerate(trajectory.points):
        stamp = float(point.time_from_start)
        if not math.isfinite(stamp) or stamp < 0.0 or stamp < previous_time:
            raise ValueError(f"point {index} has invalid time")
        if len(point.positions) != len(expected_names) or not all(
            math.isfinite(value) for value in point.positions
        ):
            raise ValueError(f"point {index} has invalid positions")
        if point.feedback_timestamp is not None and (
            not math.isfinite(point.feedback_timestamp) or point.feedback_timestamp <= 0.0
        ):
            raise ValueError(f"point {index} has invalid feedback timestamp")
        previous_time = stamp
    if trajectory.duration <= 0.0:
        raise ValueError("action group duration must be positive")


def _atomic_write(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _parse_document(
    path: Path,
    expected_names: Sequence[str],
    expected_model: str,
) -> tuple[ActionGroupInfo, RecordedTrajectory]:
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if document.get("format") != LIBRARY_FORMAT:
        raise ValueError("unsupported action group format")
    if int(document.get("version", -1)) != LIBRARY_VERSION:
        raise ValueError("unsupported action group version")
    name = validate_action_name(document.get("name", ""))
    description = str(document.get("description", "")).strip()
    robot_model = str(document.get("robot_model", "")).strip()
    if robot_model != expected_model:
        raise ValueError(
            f"action group robot model {robot_model!r} does not match {expected_model!r}"
        )
    names = tuple(str(item) for item in document.get("joint_names", []))
    if names != tuple(expected_names):
        raise ValueError("action group joint names do not match this robot")
    raw_points = document.get("points")
    if not isinstance(raw_points, list) or len(raw_points) < 2:
        raise ValueError("action group must contain at least two points")
    points: list[RecordedPoint] = []
    previous_time = -1.0
    for index, raw in enumerate(raw_points):
        if not isinstance(raw, dict):
            raise ValueError(f"point {index} must be a mapping")
        stamp = float(raw.get("time_from_start", -1.0))
        positions = tuple(float(value) for value in raw.get("positions", []))
        if not math.isfinite(stamp) or stamp < 0.0 or stamp < previous_time:
            raise ValueError(f"point {index} has invalid time")
        if len(positions) != len(names):
            raise ValueError(f"point {index} has invalid position count")
        if not all(math.isfinite(value) for value in positions):
            raise ValueError(f"point {index} contains non-finite positions")
        feedback_stamp = raw.get("feedback_timestamp")
        if feedback_stamp is not None:
            feedback_stamp = float(feedback_stamp)
            if not math.isfinite(feedback_stamp) or feedback_stamp <= 0.0:
                raise ValueError(f"point {index} has invalid feedback timestamp")
        metadata = {field: raw.get(field) for field in
                    ("timestamp_j12", "timestamp_j34", "timestamp_j56",
                     "callback_monotonic_time", "intra_cycle_span")
                    if raw.get(field) is not None}
        metadata["timing_valid"] = raw.get("timing_valid", True)
        points.append(RecordedPoint(
            stamp, positions, feedback_timestamp=feedback_stamp,
            **TrajectoryRecorder._sample_metadata(metadata)))
        previous_time = stamp
    trajectory = RecordedTrajectory(
        joint_names=names,
        points=tuple(points),
        created_utc=str(document.get("created_utc", "")),
        timing_valid=bool(document.get("timing_valid", True)),
        joint_limits_valid=bool(document.get("joint_limits_valid", True)),
    )
    info = ActionGroupInfo(
        name=name,
        description=description,
        robot_model=robot_model,
        path=path,
        created_utc=trajectory.created_utc,
        duration_sec=trajectory.duration,
        point_count=len(points),
        shape_parameters=_normalize_shape_parameters(
            document.get("shape_parameters")
        ),
    )
    return info, trajectory


def _sequence_document(
    name: str, robot_model: str, action_names: Sequence[str]
) -> dict:
    return {
        "format": SEQUENCE_FORMAT,
        "version": SEQUENCE_VERSION,
        "name": name,
        "robot_model": robot_model,
        "action_names": list(action_names),
    }


def _parse_sequence_document(
    path: Path, expected_model: str
) -> ActionSequenceInfo:
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if document.get("format") != SEQUENCE_FORMAT:
        raise ValueError("unsupported action sequence format")
    if int(document.get("version", -1)) != SEQUENCE_VERSION:
        raise ValueError("unsupported action sequence version")
    name = validate_action_name(document.get("name", ""))
    robot_model = str(document.get("robot_model", "")).strip()
    if robot_model != expected_model:
        raise ValueError(
            f"action sequence robot model {robot_model!r} does not match {expected_model!r}"
        )
    raw_action_names = document.get("action_names")
    if not isinstance(raw_action_names, list) or not raw_action_names:
        raise ValueError("action sequence must contain at least one action group")
    action_names = tuple(validate_action_name(item) for item in raw_action_names)
    return ActionSequenceInfo(name, robot_model, path, action_names)


class ActionGroupLibrary:
    """Store one dense taught trajectory per named action group."""

    def __init__(
        self,
        directory: str | Path,
        joint_names: Sequence[str],
        robot_model: str = "rebotarm_rs",
    ) -> None:
        self.directory = Path(directory).expanduser().resolve()
        self.joint_names = tuple(str(name) for name in joint_names)
        self.robot_model = str(robot_model)
        if not self.joint_names:
            raise ValueError("joint_names must not be empty")
        if not self.robot_model:
            raise ValueError("robot_model must not be empty")

    def path_for(self, name: str) -> Path:
        return self.directory / action_filename(name)

    @property
    def sequence_directory(self) -> Path:
        return self.directory / "sequences"

    def sequence_path_for(self, name: str) -> Path:
        return self.sequence_directory / sequence_filename(name)

    def save(
        self,
        name: str,
        description: str,
        trajectory: RecordedTrajectory,
        *,
        overwrite: bool = False,
        shape_parameters: Mapping | None = None,
    ) -> ActionGroupInfo:
        normalized = validate_action_name(name)
        _validate_trajectory_data(trajectory, self.joint_names)
        target = self.path_for(normalized)
        if target.exists() and not overwrite:
            raise FileExistsError(
                f"action group {normalized!r} already exists; set overwrite=true"
            )
        _atomic_write(
            target,
            _trajectory_document(
                normalized,
                str(description).strip(),
                self.robot_model,
                trajectory,
                shape_parameters,
            ),
        )
        info, _ = _parse_document(target, self.joint_names, self.robot_model)
        return info

    def load(self, name: str) -> tuple[ActionGroupInfo, RecordedTrajectory]:
        normalized = validate_action_name(name)
        target = self.path_for(normalized)
        if not target.is_file():
            raise FileNotFoundError(f"action group {normalized!r} does not exist")
        info, trajectory = _parse_document(
            target, self.joint_names, self.robot_model
        )
        if info.name != normalized:
            raise ValueError("action group file name/content mismatch")
        return info, trajectory

    def list_groups(self) -> tuple[list[ActionGroupInfo], list[str]]:
        groups: list[ActionGroupInfo] = []
        warnings: list[str] = []
        if not self.directory.is_dir():
            return groups, warnings
        for path in sorted(self.directory.glob("*.teach.json")):
            try:
                info, _ = _parse_document(path, self.joint_names, self.robot_model)
                if path != self.path_for(info.name):
                    raise ValueError("action group file name/content mismatch")
                groups.append(info)
            except Exception as error:
                warnings.append(f"{path.name}: {error}")
        groups.sort(key=lambda item: item.name)
        return groups, warnings

    def rename(self, old_name: str, new_name: str) -> ActionGroupInfo:
        """Rename an action and atomically update sequence references."""
        old_normalized = validate_action_name(old_name)
        new_normalized = validate_action_name(new_name)
        old_info, trajectory = self.load(old_normalized)
        if old_normalized == new_normalized:
            return old_info
        target = self.path_for(new_normalized)
        if target.exists():
            raise FileExistsError(f"action group {new_normalized!r} already exists")

        sequences, warnings = self.list_sequences(validate_actions=False)
        if warnings:
            raise ValueError("cannot safely rename while sequence files are invalid: " + "; ".join(warnings))
        updated_sequences = [
            (
                sequence,
                tuple(
                    new_normalized if item == old_normalized else item
                    for item in sequence.action_names
                ),
            )
            for sequence in sequences
            if old_normalized in sequence.action_names
        ]

        new_info = self.save(
            new_normalized,
            old_info.description,
            trajectory,
            overwrite=False,
            shape_parameters=old_info.shape_parameters,
        )
        try:
            for sequence, action_names in updated_sequences:
                _atomic_write(
                    sequence.path,
                    _sequence_document(sequence.name, self.robot_model, action_names),
                )
            old_info.path.unlink()
        except Exception:
            try:
                new_info.path.unlink()
            except FileNotFoundError:
                pass
            raise
        return new_info

    def delete(self, name: str) -> ActionGroupInfo:
        """Delete an action that is not referenced by a named sequence."""
        normalized = validate_action_name(name)
        info, _ = self.load(normalized)
        sequences, warnings = self.list_sequences(validate_actions=False)
        if warnings:
            raise ValueError(
                "cannot safely delete while sequence files are invalid: "
                + "; ".join(warnings)
            )
        referenced_by = [
            sequence.name
            for sequence in sequences
            if normalized in sequence.action_names
        ]
        if referenced_by:
            raise ValueError(
                f"action group {normalized!r} is referenced by action sequences: "
                + ", ".join(repr(item) for item in referenced_by)
            )
        info.path.unlink()
        return info

    def copy(self, source_name: str, new_name: str) -> ActionGroupInfo:
        """Copy an action under a new name while retaining all metadata."""
        source_normalized = validate_action_name(source_name)
        new_normalized = validate_action_name(new_name)
        if source_normalized == new_normalized:
            raise ValueError("copied action group must use a different name")
        source_info, trajectory = self.load(source_normalized)
        return self.save(
            new_normalized,
            source_info.description,
            trajectory,
            overwrite=False,
            shape_parameters=source_info.shape_parameters,
        )

    def save_sequence(
        self,
        name: str,
        action_names: Sequence[str],
        *,
        overwrite: bool = False,
    ) -> ActionSequenceInfo:
        normalized = validate_action_name(name)
        normalized_actions = tuple(validate_action_name(item) for item in action_names)
        if not normalized_actions:
            raise ValueError("action sequence must contain at least one action group")
        for action_name in normalized_actions:
            self.load(action_name)
        target = self.sequence_path_for(normalized)
        if target.exists() and not overwrite:
            raise FileExistsError(
                f"action sequence {normalized!r} already exists; set overwrite=true"
            )
        _atomic_write(
            target,
            _sequence_document(normalized, self.robot_model, normalized_actions),
        )
        return _parse_sequence_document(target, self.robot_model)

    def load_sequence(self, name: str) -> ActionSequenceInfo:
        normalized = validate_action_name(name)
        target = self.sequence_path_for(normalized)
        if not target.is_file():
            raise FileNotFoundError(f"action sequence {normalized!r} does not exist")
        info = _parse_sequence_document(target, self.robot_model)
        if info.name != normalized:
            raise ValueError("action sequence file name/content mismatch")
        for action_name in info.action_names:
            self.load(action_name)
        return info

    def list_sequences(
        self, *, validate_actions: bool = True
    ) -> tuple[list[ActionSequenceInfo], list[str]]:
        sequences: list[ActionSequenceInfo] = []
        warnings: list[str] = []
        if not self.sequence_directory.is_dir():
            return sequences, warnings
        for path in sorted(self.sequence_directory.glob("*.sequence.json")):
            try:
                info = _parse_sequence_document(path, self.robot_model)
                if path != self.sequence_path_for(info.name):
                    raise ValueError("action sequence file name/content mismatch")
                if validate_actions:
                    for action_name in info.action_names:
                        self.load(action_name)
                sequences.append(info)
            except Exception as error:
                warnings.append(f"{path.name}: {error}")
        sequences.sort(key=lambda item: item.name)
        return sequences, warnings
