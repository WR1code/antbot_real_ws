"""Arrival window used only by the Piper-H automatic hand-eye workflow."""
from collections import deque
import math


class HandeyeArrivalWindow:
    """Require every target residual and the whole stationary window to be strict."""

    def __init__(self, target, tolerance_deg=0.1, duration=1.0, variation_deg=0.01):
        self.target = tuple(float(value) for value in target)
        if len(self.target) != 6 or not all(math.isfinite(value) for value in self.target):
            raise ValueError("arrival target requires six finite joints")
        for value in (tolerance_deg, duration, variation_deg):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("arrival thresholds must be finite and positive")
        self.tolerance_deg = tolerance_deg
        self.duration = duration
        self.variation_deg = variation_deg
        self.samples = deque()
        self.residual_deg = None
        self.last_stamp = None
        self.last_received = None

    def add(self, positions, stamp, received):
        if (len(positions) != 6 or not all(math.isfinite(value) for value in positions)
                or (self.last_stamp is not None and stamp <= self.last_stamp)):
            return False
        if self.last_received is not None and received - self.last_received > 0.75:
            self.samples.clear()
        self.last_stamp, self.last_received = stamp, received
        self.residual_deg = tuple(math.degrees(a - b) for a, b in zip(positions, self.target))
        if any(abs(value) >= self.tolerance_deg for value in self.residual_deg):
            self.samples.clear()
            return False
        self.samples.append((stamp, received, tuple(positions)))
        # Retain the boundary sample to require a full second, never a shorter window.
        cutoff = received - self.duration
        while len(self.samples) > 1 and self.samples[1][1] <= cutoff:
            self.samples.popleft()
        if (received - self.samples[0][1] < self.duration
                or stamp - self.samples[0][0] < self.duration):
            return False
        variations = [math.degrees(max(values) - min(values))
                      for values in zip(*(item[2] for item in self.samples))]
        return all(value < self.variation_deg for value in variations)


def _joint_values(values):
    if values is None:
        return "unavailable"
    return "[" + ", ".join(f"{float(value):+.9f}" for value in values) + "] rad"


def arrival_timeout_message(
    label, target, positions, feedback_age, command_target=None, planned_final=None,
):
    """Build an actionable timeout report without weakening the arrival gate."""
    command_target = target if command_target is None else command_target
    planned_final = target if planned_final is None else planned_final
    lines = [
        f"Handeye calibration arrival timeout: {label}",
        f"command target: {_joint_values(command_target)}",
        f"planned final joint: {_joint_values(planned_final)}",
        f"current feedback joint: {_joint_values(positions)}",
        "joint error(deg):",
    ]
    if positions is None:
        lines.extend(
            f"J{i} error=unavailable (J{i}: unavailable; no real joint feedback)"
            for i in range(1, 7)
        )
        lines.append("max error=unavailable")
        lines.append("最大误差关节: unavailable")
    else:
        residuals = [math.degrees(a - b) for a, b in zip(positions, planned_final)]
        lines.extend(
            f"J{i} error={value:+.6f} deg (J{i}: {value:+.6f} deg)"
            for i, value in enumerate(residuals, 1)
        )
        worst = max(range(6), key=lambda index: abs(residuals[index]))
        lines.append(f"max error={abs(residuals[worst]):.6f} deg (J{worst + 1})")
        lines.append(f"最大误差关节: J{worst + 1}, absolute error={abs(residuals[worst]):.6f} deg")
    lines.append(f"feedback age: {feedback_age:.3f} s (real feedback age)")
    return "\n".join(lines)
