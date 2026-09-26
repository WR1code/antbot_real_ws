"""Pure trajectory recording, persistence, and conservative replay planning."""

from __future__ import annotations

import json
import math
import os
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence


FORMAT_VERSION = 1


@dataclass(frozen=True)
class RecordedPoint:
    time_from_start: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...] = ()
    efforts: tuple[float, ...] = ()
    feedback_timestamp: float | None = None
    timestamp_j12: float | None = None
    timestamp_j34: float | None = None
    timestamp_j56: float | None = None
    callback_monotonic_time: float | None = None
    intra_cycle_span: float | None = None
    timing_valid: bool = True


@dataclass(frozen=True)
class RecordedTrajectory:
    joint_names: tuple[str, ...]
    points: tuple[RecordedPoint, ...]
    created_utc: str
    timing_valid: bool = True
    joint_limits_valid: bool = True

    @property
    def duration(self) -> float:
        return self.points[-1].time_from_start if self.points else 0.0


@dataclass(frozen=True)
class PlaybackPlan:
    points: tuple[RecordedPoint, ...]
    includes_reverse_return: bool

    @property
    def duration(self) -> float:
        return self.points[-1].time_from_start if self.points else 0.0


def _finite_positions(values: Sequence[float], expected: int) -> tuple[float, ...]:
    positions = tuple(float(value) for value in values)
    if len(positions) != expected or not all(math.isfinite(value) for value in positions):
        raise ValueError(f"expected {expected} finite joint positions")
    return positions


def _optional_finite(values: Sequence[float] | None, expected: int) -> tuple[float, ...]:
    if values is None or len(values) == 0:
        return ()
    return _finite_positions(values, expected)


class TrajectoryRecorder:
    """Record monotonic joint samples; tagged feedback is never duplicated."""

    def __init__(
        self,
        joint_names: Sequence[str],
        sample_period_sec: float = 0.02,
        minimum_position_delta: float = 0.001,
        maximum_duration_sec: float = 120.0,
        maximum_samples: int = 6000,
    ) -> None:
        self.joint_names = tuple(str(name) for name in joint_names)
        if not self.joint_names or len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be non-empty and unique")
        if sample_period_sec <= 0.0 or minimum_position_delta < 0.0:
            raise ValueError("sample period must be positive and delta non-negative")
        if maximum_duration_sec <= 0.0 or maximum_samples < 2:
            raise ValueError("recording limits must be positive")
        self.sample_period_sec = float(sample_period_sec)
        self.minimum_position_delta = float(minimum_position_delta)
        self.maximum_duration_sec = float(maximum_duration_sec)
        self.maximum_samples = int(maximum_samples)
        self._start_time: float | None = None
        self._points: list[RecordedPoint] = []

    @property
    def recording(self) -> bool:
        return self._start_time is not None

    @property
    def sample_count(self) -> int:
        return len(self._points)

    def start(
        self, now_sec: float, positions: Sequence[float],
        velocities: Sequence[float] | None = None,
        efforts: Sequence[float] | None = None,
        *, feedback_timestamp: float | None = None, sample_metadata: dict | None = None,
    ) -> None:
        if self.recording:
            raise RuntimeError("recording already active")
        if not math.isfinite(float(now_sec)):
            raise ValueError("recording start time must be finite")
        stamp = self._feedback_stamp(feedback_timestamp)
        self._start_time = float(now_sec)
        self._points = [
            RecordedPoint(
                0.0, _finite_positions(positions, len(self.joint_names)),
                _optional_finite(velocities, len(self.joint_names)),
                _optional_finite(efforts, len(self.joint_names)),
                stamp, **self._sample_metadata(sample_metadata),
            )
        ]

    @staticmethod
    def _feedback_stamp(value: float | None) -> float | None:
        if value is None:
            return None
        stamp = float(value)
        if not math.isfinite(stamp) or stamp <= 0.0:
            raise ValueError("feedback timestamp must be finite and positive")
        return stamp

    def add(
        self, now_sec: float, positions: Sequence[float], *,
        velocities: Sequence[float] | None = None,
        efforts: Sequence[float] | None = None,
        force: bool = False,
        feedback_timestamp: float | None = None,
        sample_metadata: dict | None = None,
    ) -> bool:
        if self._start_time is None:
            raise RuntimeError("recording is not active")
        elapsed = float(now_sec) - self._start_time
        if not math.isfinite(elapsed) or elapsed < 0.0:
            raise ValueError("sample time moved backwards")
        if elapsed > self.maximum_duration_sec:
            raise RuntimeError("maximum recording duration exceeded")
        current = _finite_positions(positions, len(self.joint_names))
        previous = self._points[-1]
        stamp = self._feedback_stamp(feedback_timestamp)
        if stamp is not None and previous.feedback_timestamp is not None:
            if stamp == previous.feedback_timestamp:
                return False
            if stamp < previous.feedback_timestamp:
                raise ValueError("feedback timestamp moved backwards")
        time_delta = elapsed - previous.time_from_start
        position_delta = max(abs(a - b) for a, b in zip(current, previous.positions))
        if not force and (
            time_delta < self.sample_period_sec
            or position_delta < self.minimum_position_delta
        ):
            return False
        point = RecordedPoint(
            max(elapsed, previous.time_from_start), current,
            _optional_finite(velocities, len(self.joint_names)),
            _optional_finite(efforts, len(self.joint_names)),
            stamp, **self._sample_metadata(sample_metadata),
        )
        if force and stamp is None and position_delta < 1e-12:
            self._points[-1] = point
            return True
        if elapsed <= previous.time_from_start:
            raise ValueError("recording timestamps must be strictly increasing")
        if len(self._points) >= self.maximum_samples:
            raise RuntimeError("maximum recording sample count exceeded")
        self._points.append(point)
        return True

    def finish(
        self,
        now_sec: float,
        positions: Sequence[float],
        *,
        velocities: Sequence[float] | None = None,
        efforts: Sequence[float] | None = None,
        feedback_timestamp: float | None = None,
        sample_metadata: dict | None = None,
        minimum_points: int = 3,
        minimum_duration_sec: float = 0.2,
    ) -> RecordedTrajectory:
        self.add(now_sec, positions, velocities=velocities, efforts=efforts,
                 force=True, feedback_timestamp=feedback_timestamp,
                 sample_metadata=sample_metadata)
        points = tuple(self._points)
        self._start_time = None
        self._points = []
        if len(points) < minimum_points:
            raise ValueError(f"recording needs at least {minimum_points} points")
        if points[-1].time_from_start < minimum_duration_sec:
            raise ValueError(
                f"recording duration must be at least {minimum_duration_sec:.3f}s"
            )
        return RecordedTrajectory(
            joint_names=self.joint_names,
            points=points,
            created_utc=datetime.now(timezone.utc).isoformat(),
        )

    def discard(self) -> None:
        self._start_time = None
        self._points = []

    @classmethod
    def _sample_metadata(cls, value: dict | None) -> dict:
        if not value:
            return {}
        stamps = {name: cls._feedback_stamp(value.get(name))
                  for name in ("timestamp_j12", "timestamp_j34", "timestamp_j56")}
        callback = value.get("callback_monotonic_time")
        span = value.get("intra_cycle_span")
        if callback is not None and (not math.isfinite(float(callback)) or float(callback) <= 0):
            raise ValueError("callback monotonic time must be positive and finite")
        if span is not None and (not math.isfinite(float(span)) or float(span) < 0):
            raise ValueError("intra-cycle span must be nonnegative and finite")
        return {**stamps,
                "callback_monotonic_time": float(callback) if callback is not None else None,
                "intra_cycle_span": float(span) if span is not None else None,
                "timing_valid": bool(value.get("timing_valid", True))}


def validate_joint_limits(
    trajectory: RecordedTrajectory,
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    margin: float = 0.0,
) -> None:
    count = len(trajectory.joint_names)
    lower = _finite_positions(lower_limits, count)
    upper = _finite_positions(upper_limits, count)
    if margin < 0.0:
        raise ValueError("joint limit margin must be non-negative")
    for joint_index, (minimum, maximum) in enumerate(zip(lower, upper)):
        if minimum + margin >= maximum - margin:
            raise ValueError("joint limit margin leaves no valid range")
        for point_index, point in enumerate(trajectory.points):
            value = point.positions[joint_index]
            if not minimum + margin <= value <= maximum - margin:
                name = trajectory.joint_names[joint_index]
                raise ValueError(
                    f"point {point_index} {name}={value:.6f} outside guarded limits"
                )


def save_trajectory(path: str | Path, trajectory: RecordedTrajectory) -> None:
    target = Path(path).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    document = {
        "format": "rebot_teach_trajectory",
        "version": FORMAT_VERSION,
        "created_utc": trajectory.created_utc,
        "joint_names": list(trajectory.joint_names),
        "duration_sec": trajectory.duration,
        "timing_valid": trajectory.timing_valid and all(point.timing_valid for point in trajectory.points),
        "joint_limits_valid": trajectory.joint_limits_valid,
        "points": [
            ({
                "time_from_start": point.time_from_start,
                "positions": list(point.positions),
            } | ({"velocities": list(point.velocities)} if point.velocities else {})
              | ({"efforts": list(point.efforts)} if point.efforts else {})
              | ({"feedback_timestamp": point.feedback_timestamp}
                 if point.feedback_timestamp is not None else {})
              | ({name: getattr(point, name) for name in
                  ("timestamp_j12", "timestamp_j34", "timestamp_j56",
                   "callback_monotonic_time", "intra_cycle_span")
                  if getattr(point, name) is not None})
              | ({"timing_valid": False} if not point.timing_valid else {}))
            for point in trajectory.points
        ],
    }
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=target.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, target)
    except Exception:
        try:
            os.unlink(temporary_path)
        except FileNotFoundError:
            pass
        raise


def load_trajectory(path: str | Path) -> RecordedTrajectory:
    source = Path(path).expanduser().resolve()
    with source.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if document.get("format") != "rebot_teach_trajectory":
        raise ValueError("unsupported trajectory file format")
    if int(document.get("version", -1)) != FORMAT_VERSION:
        raise ValueError("unsupported trajectory file version")
    names = tuple(str(name) for name in document.get("joint_names", []))
    if not names or len(set(names)) != len(names):
        raise ValueError("trajectory joint_names must be non-empty and unique")
    raw_points = document.get("points")
    if not isinstance(raw_points, list) or len(raw_points) < 2:
        raise ValueError("trajectory must contain at least two points")
    points: list[RecordedPoint] = []
    previous_time = -1.0
    for index, item in enumerate(raw_points):
        if not isinstance(item, dict):
            raise ValueError(f"point {index} must be a mapping")
        stamp = float(item.get("time_from_start", -1.0))
        if not math.isfinite(stamp) or stamp < 0.0 or stamp <= previous_time:
            raise ValueError(f"point {index} has invalid time_from_start")
        positions = _finite_positions(item.get("positions", []), len(names))
        velocities = _optional_finite(item.get("velocities"), len(names))
        efforts = _optional_finite(item.get("efforts"), len(names))
        feedback_stamp = item.get("feedback_timestamp")
        if feedback_stamp is not None:
            feedback_stamp = TrajectoryRecorder._feedback_stamp(feedback_stamp)
        metadata = {name: item.get(name) for name in
                    ("timestamp_j12", "timestamp_j34", "timestamp_j56",
                     "callback_monotonic_time", "intra_cycle_span")
                    if item.get(name) is not None}
        metadata["timing_valid"] = item.get("timing_valid", True)
        points.append(RecordedPoint(stamp, positions, velocities, efforts, feedback_stamp,
                                    **TrajectoryRecorder._sample_metadata(metadata)))
        previous_time = stamp
    return RecordedTrajectory(
        joint_names=names,
        points=tuple(points),
        created_utc=str(document.get("created_utc", "")),
        timing_valid=bool(document.get("timing_valid", True)),
        joint_limits_valid=bool(document.get("joint_limits_valid", True)),
    )


def _maximum_delta(first: Sequence[float], second: Sequence[float]) -> float:
    return max(abs(a - b) for a, b in zip(first, second))


def sample_trajectory(
    trajectory: RecordedTrajectory, progress: float
) -> tuple[float, ...]:
    """Linearly sample a recorded path by normalized elapsed time."""
    if not trajectory.points:
        raise ValueError("trajectory must contain at least one point")
    normalized = float(progress)
    if not math.isfinite(normalized) or not 0.0 <= normalized <= 1.0:
        raise ValueError("preview progress must be in [0, 1]")
    if len(trajectory.points) == 1 or trajectory.duration <= 0.0:
        return trajectory.points[0].positions
    target = normalized * trajectory.duration
    if target <= trajectory.points[0].time_from_start:
        return trajectory.points[0].positions
    for first, second in zip(trajectory.points, trajectory.points[1:]):
        if target <= second.time_from_start:
            duration = second.time_from_start - first.time_from_start
            if duration <= 0.0:
                return second.positions
            ratio = (target - first.time_from_start) / duration
            return tuple(
                start + (end - start) * ratio
                for start, end in zip(first.positions, second.positions)
            )
    return trajectory.points[-1].positions


def prepare_rviz_preview(
    trajectory: RecordedTrajectory,
    *,
    frame_period_sec: float = 0.02,
    maximum_idle_sec: float = 0.20,
    idle_position_delta: float = 0.002,
) -> RecordedTrajectory:
    """Create a smooth display-only trajectory from irregular recorded data.

    Long intervals whose endpoints differ only by encoder noise are compressed
    for inspection, then the path is linearly sampled at a stable display rate.
    The stored trajectory used for hardware replay is never modified.
    """
    if frame_period_sec <= 0.0 or maximum_idle_sec <= 0.0:
        raise ValueError("preview frame period and maximum idle time must be positive")
    if idle_position_delta < 0.0:
        raise ValueError("preview idle position delta must be non-negative")
    if not trajectory.points:
        raise ValueError("trajectory must contain at least one point")
    if len(trajectory.points) == 1 or trajectory.duration <= 0.0:
        return trajectory

    display_time = 0.0
    compressed = [RecordedPoint(0.0, trajectory.points[0].positions)]
    for previous, current in zip(trajectory.points, trajectory.points[1:]):
        segment_time = current.time_from_start - previous.time_from_start
        position_delta = _maximum_delta(previous.positions, current.positions)
        if segment_time > maximum_idle_sec and position_delta <= idle_position_delta:
            segment_time = maximum_idle_sec
        display_time += segment_time
        compressed.append(RecordedPoint(display_time, current.positions))

    compressed_trajectory = RecordedTrajectory(
        joint_names=trajectory.joint_names,
        points=tuple(compressed),
        created_utc=trajectory.created_utc,
    )
    frame_count = max(1, int(math.ceil(display_time / frame_period_sec)))
    frames = tuple(
        RecordedPoint(
            display_time * index / frame_count,
            sample_trajectory(compressed_trajectory, index / frame_count),
        )
        for index in range(frame_count + 1)
    )
    return RecordedTrajectory(
        joint_names=trajectory.joint_names,
        points=frames,
        created_utc=trajectory.created_utc,
    )


def concatenate_trajectories(
    trajectories: Sequence[RecordedTrajectory],
) -> RecordedTrajectory:
    """Join actions in order, preserving a zero-time jump at each boundary."""
    items = tuple(trajectories)
    if not items:
        raise ValueError("at least one trajectory is required")
    joint_names = items[0].joint_names
    if not joint_names:
        raise ValueError("trajectory joint names must not be empty")
    output: list[RecordedPoint] = []
    offset = 0.0
    for index, trajectory in enumerate(items):
        if trajectory.joint_names != joint_names:
            raise ValueError("all trajectories must use the same joint names")
        if len(trajectory.points) < 2 or trajectory.duration <= 0.0:
            raise ValueError(f"trajectory {index} must contain timed motion")
        if index == 0:
            output.extend(trajectory.points)
        else:
            # The equal timestamp is intentional: RViz changes to the next
            # recorded start pose instead of inventing connecting motion.
            output.append(RecordedPoint(offset, trajectory.points[0].positions))
            output.extend(
                RecordedPoint(offset + point.time_from_start, point.positions)
                for point in trajectory.points[1:]
            )
        offset += trajectory.duration
    return RecordedTrajectory(
        joint_names=joint_names,
        points=tuple(output),
        created_utc=items[0].created_utc,
    )


def build_preview_loop(
    trajectory: RecordedTrajectory, progress: float
) -> RecordedTrajectory:
    """Rotate a preview loop to start at normalized elapsed-time progress.

    The returned trajectory keeps the source duration and timing.  Its only
    discontinuity is the same end-to-start jump that RViz's loop animation
    already performs, but the jump is moved into the middle of the message so
    playback can begin at the requested pose and continue immediately.
    """
    start_positions = sample_trajectory(trajectory, progress)
    normalized = float(progress)
    if normalized == 0.0 or len(trajectory.points) == 1 or trajectory.duration <= 0.0:
        return trajectory

    duration = trajectory.duration
    target_time = normalized * duration
    points = [RecordedPoint(0.0, start_positions)]

    # Finish the source path from the selected time to its endpoint.
    points.extend(
        RecordedPoint(point.time_from_start - target_time, point.positions)
        for point in trajectory.points
        if point.time_from_start > target_time
    )
    # RViz normally jumps from the endpoint back to the start between loops.
    # Preserve that zero-time jump, then play the beginning up to the seek pose.
    points.extend(
        RecordedPoint(duration - target_time + point.time_from_start, point.positions)
        for point in trajectory.points
        if point.time_from_start < target_time
    )
    points.append(RecordedPoint(duration, start_positions))
    return RecordedTrajectory(
        joint_names=trajectory.joint_names,
        points=tuple(points),
        created_utc=trajectory.created_utc,
    )


def build_playback_plan(
    trajectory: RecordedTrajectory,
    current_positions: Sequence[float],
    *,
    transition_trajectory: RecordedTrajectory | None = None,
    endpoint_tolerance: float = 0.08,
    speed_scale: float = 1.0,
    maximum_joint_velocity: float = 0.30,
    minimum_segment_sec: float = 0.02,
    reverse_return: bool = True,
    start_dwell_sec: float = 0.5,
) -> PlaybackPlan:
    """Build replay, optionally following a planned path to the recorded start."""
    if len(trajectory.points) < 2:
        raise ValueError("trajectory must contain at least two points")
    if (not math.isfinite(trajectory.points[0].time_from_start)
        or trajectory.points[0].time_from_start < 0.0
        or any(not math.isfinite(point.time_from_start)
               or point.time_from_start <= previous.time_from_start
               for previous, point in zip(trajectory.points, trajectory.points[1:]))):
        raise ValueError("recorded timestamps must be finite and strictly increasing")
    if not 0.0 < speed_scale <= 2.0:
        raise ValueError("speed_scale must be in (0, 2]")
    if endpoint_tolerance < 0.0 or maximum_joint_velocity <= 0.0:
        raise ValueError("endpoint tolerance and velocity limits are invalid")
    if minimum_segment_sec <= 0.0 or start_dwell_sec < 0.0:
        raise ValueError("segment and dwell times are invalid")
    current = _finite_positions(current_positions, len(trajectory.joint_names))
    at_start = _maximum_delta(current, trajectory.points[0].positions) <= endpoint_tolerance
    at_end = _maximum_delta(current, trajectory.points[-1].positions) <= endpoint_tolerance
    if transition_trajectory is not None:
        if transition_trajectory.joint_names != trajectory.joint_names:
            raise ValueError("transition joint names do not match recorded action")
        if len(transition_trajectory.points) < 2:
            raise ValueError("transition trajectory must contain at least two points")
        if (
            _maximum_delta(current, transition_trajectory.points[0].positions)
            > endpoint_tolerance
        ):
            raise ValueError("current pose is not near the planned transition start")
        if (
            _maximum_delta(
                transition_trajectory.points[-1].positions,
                trajectory.points[0].positions,
            )
            > endpoint_tolerance
        ):
            raise ValueError("planned transition does not reach the recorded start")
    elif not at_start and not (reverse_return and at_end):
        raise ValueError(
            "current pose is not near the recorded start or end; refusing replay"
        )

    commands: list[tuple[tuple[float, ...], float]] = []
    if transition_trajectory is not None:
        first_transition = transition_trajectory.points[0]
        if _maximum_delta(current, first_transition.positions) > 1e-9:
            commands.append((first_transition.positions, 0.0))
        for previous, point in zip(
            transition_trajectory.points, transition_trajectory.points[1:]
        ):
            commands.append(
                (point.positions, point.time_from_start - previous.time_from_start)
            )
        transition_end = transition_trajectory.points[-1].positions
        if _maximum_delta(transition_end, trajectory.points[0].positions) > 1e-9:
            commands.append((trajectory.points[0].positions, 0.0))
    elif at_end and not at_start:
        for index in range(len(trajectory.points) - 2, -1, -1):
            nominal = (
                trajectory.points[index + 1].time_from_start
                - trajectory.points[index].time_from_start
            )
            commands.append((trajectory.points[index].positions, nominal))
        if start_dwell_sec > 0.0:
            commands.append((trajectory.points[0].positions, start_dwell_sec * speed_scale))
    else:
        # Do not insert a duplicate start point when the robot is already at
        # the recorded start. That artificial segment made every replay begin
        # late even at 1.0x. A small correction is still planned when feedback
        # is only within the endpoint tolerance rather than exactly equal.
        start_delta = _maximum_delta(current, trajectory.points[0].positions)
        if start_delta > 1e-9:
            commands.append((trajectory.points[0].positions, 0.0))

    for index in range(1, len(trajectory.points)):
        nominal = (
            trajectory.points[index].time_from_start
            - trajectory.points[index - 1].time_from_start
        )
        commands.append((trajectory.points[index].positions, nominal))

    output = [RecordedPoint(0.0, current)]
    elapsed = 0.0
    previous = current
    for positions, nominal in commands:
        delta = _maximum_delta(previous, positions)
        duration = max(
            nominal / speed_scale,
            delta / maximum_joint_velocity,
            minimum_segment_sec,
        )
        elapsed += duration
        output.append(RecordedPoint(elapsed, positions))
        previous = positions
    return PlaybackPlan(
        points=tuple(output),
        includes_reverse_return=bool(
            transition_trajectory is None and at_end and not at_start
        ),
    )


def concatenate_trajectories_with_transitions(
    trajectories: Sequence[RecordedTrajectory],
    transitions: Sequence[RecordedTrajectory | None],
    *,
    endpoint_tolerance: float = 0.08,
    maximum_joint_velocity: float = 0.30,
) -> RecordedTrajectory:
    """Join actions with the same continuous boundary plans used by hardware."""
    items = tuple(trajectories)
    boundary_plans = tuple(transitions)
    if not items:
        raise ValueError("at least one trajectory is required")
    if len(boundary_plans) != len(items) - 1:
        raise ValueError("transition count must equal action count minus one")
    first = items[0]
    if len(first.points) < 2 or first.duration <= 0.0:
        raise ValueError("trajectory 0 must contain timed motion")
    output = list(first.points)
    for index, (trajectory, transition) in enumerate(
        zip(items[1:], boundary_plans), start=1
    ):
        if trajectory.joint_names != first.joint_names:
            raise ValueError("all trajectories must use the same joint names")
        if len(trajectory.points) < 2 or trajectory.duration <= 0.0:
            raise ValueError(f"trajectory {index} must contain timed motion")
        plan = build_playback_plan(
            trajectory,
            output[-1].positions,
            transition_trajectory=transition,
            endpoint_tolerance=endpoint_tolerance,
            maximum_joint_velocity=maximum_joint_velocity,
            reverse_return=False,
        )
        offset = output[-1].time_from_start
        output.extend(
            RecordedPoint(offset + point.time_from_start, point.positions)
            for point in plan.points[1:]
        )
    return RecordedTrajectory(
        joint_names=first.joint_names,
        points=tuple(output),
        created_utc=first.created_utc,
    )
