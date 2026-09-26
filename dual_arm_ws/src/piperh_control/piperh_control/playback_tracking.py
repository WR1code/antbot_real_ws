"""Pure, bounded trajectory preparation and playback error accounting."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import csv
import math
from pathlib import Path
from statistics import mean
from typing import Sequence


def wrap_to_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def bounded_trajectory_step(wall_delta: float, period: float) -> float:
    """Prevent a delayed Python loop from jumping far ahead in the trajectory."""
    if not math.isfinite(wall_delta) or not math.isfinite(period) or period <= 0.0:
        raise ValueError("trajectory timing values must be finite and period must be positive")
    return min(max(wall_delta, 0.0), 2.0 * period)


@dataclass(frozen=True)
class PreparedTrajectory:
    times: tuple[float, ...]
    positions: tuple[tuple[float, ...], ...]
    tangents: tuple[tuple[float, ...], ...]
    scale: float

    @property
    def duration(self) -> float:
        return self.times[-1] * self.scale

    def sample(self, elapsed: float) -> tuple[float, ...]:
        t = min(max(elapsed / self.scale, 0.0), self.times[-1])
        index = min(bisect_right(self.times, t) - 1, len(self.times) - 2)
        width = self.times[index + 1] - self.times[index]
        u = (t - self.times[index]) / width
        a, b = self.positions[index:index + 2]
        va, vb = self.tangents[index:index + 2]
        # Cubic Hermite passes through every recorded point and has continuous
        # velocity.  Time scaling below bounds its sampled acceleration.
        return tuple(
            (2*u**3 - 3*u**2 + 1)*x
            + (u**3 - 2*u**2 + u)*width*vx
            + (-2*u**3 + 3*u**2)*y
            + (u**3 - u**2)*width*vy
            for x, y, vx, vy in zip(a, b, va, vb)
        )


def prepare_trajectory(
    times: Sequence[float], positions: Sequence[Sequence[float]], *,
    velocity_limit: float, acceleration_limit: float, rate_hz: float,
    lower: Sequence[float], upper: Sequence[float],
) -> PreparedTrajectory:
    """Validate, unwrap, interpolate, and conservatively retime a six-joint path."""
    ts = tuple(float(t) for t in times)
    if len(ts) < 2 or len(ts) != len(positions) or not math.isfinite(ts[0]) or ts[0] < 0:
        raise ValueError("trajectory needs matching points and nonnegative time")
    if any(not math.isfinite(t) or t <= previous for previous, t in zip(ts, ts[1:])):
        raise ValueError("trajectory timestamps must be finite and strictly increasing")
    if not all(math.isfinite(v) and v > 0 for v in (velocity_limit, acceleration_limit, rate_hz)):
        raise ValueError("playback limits and rate must be positive and finite")
    if len(lower) != 6 or len(upper) != 6:
        raise ValueError("six joint limits are required")
    qs: list[tuple[float, ...]] = []
    for raw in positions:
        q = tuple(float(x) for x in raw)
        if len(q) != 6 or not all(math.isfinite(x) for x in q):
            raise ValueError("every trajectory point needs six finite joints")
        if qs:
            q = tuple(qs[-1][j] + wrap_to_pi(x - qs[-1][j]) for j, x in enumerate(q))
        if any(x < lo - 1e-8 or x > hi + 1e-8 for x, lo, hi in zip(q, lower, upper)):
            raise ValueError("unwrapped trajectory exceeds Piper-H joint limits")
        qs.append(q)
    slopes = [tuple((b[j] - a[j]) / (tb - ta) for j in range(6))
              for a, b, ta, tb in zip(qs, qs[1:], ts, ts[1:])]
    tangents = [tuple(0.0 for _ in range(6))]
    for left, right, ta, tb, tc in zip(slopes, slopes[1:], ts, ts[1:], ts[2:]):
        tangents.append(tuple(
            0.0 if left[j] * right[j] <= 0 else
            math.copysign(min(abs((left[j]*(tc-tb) + right[j]*(tb-ta))/(tc-ta)),
                              3*abs(left[j]), 3*abs(right[j])), left[j])
            for j in range(6)))
    tangents.append(tuple(0.0 for _ in range(6)))
    prepared = PreparedTrajectory(ts, tuple(qs), tuple(tangents), 1.0)
    # Bound the cubic derivatives analytically. A 100 Hz finite difference
    # would miss peaks inside a short recorded segment.
    max_v = max_a = 0.0
    for i, width in enumerate(b-a for a, b in zip(ts, ts[1:])):
        for j in range(6):
            a, b = qs[i][j], qs[i+1][j]
            va, vb = tangents[i][j], tangents[i+1][j]
            cubic = 2*a - 2*b + width*(va+vb)
            quadratic = -3*a + 3*b - width*(2*va+vb)
            linear = width*va
            candidates = [0.0, 1.0]
            if abs(cubic) > 1e-15:
                vertex = -quadratic / (3*cubic)
                if 0.0 < vertex < 1.0:
                    candidates.append(vertex)
            max_v = max(max_v, *(abs((3*cubic*u*u + 2*quadratic*u + linear)/width)
                                   for u in candidates))
            max_a = max(max_a, abs(2*quadratic/width**2),
                        abs((6*cubic+2*quadratic)/width**2))
    # Inspect positions at the control resolution for any joint-limit overshoot.
    dt = 1.0 / rate_hz
    n = math.ceil(prepared.duration / dt)
    if n > 360000:
        raise ValueError("trajectory exceeds five hours of control samples")
    for i in range(1, n + 1):
        t = min(i * dt, prepared.duration)
        q = prepared.sample(t)
        if any(x < lo - 1e-8 or x > hi + 1e-8 for x, lo, hi in zip(q, lower, upper)):
            raise ValueError("interpolated trajectory exceeds Piper-H joint limits")
    scale = max(1.0, max_v / velocity_limit, math.sqrt(max_a / acceleration_limit))
    return PreparedTrajectory(ts, tuple(qs), tuple(tangents), scale * 1.001)


class TrackingLog:
    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._stream = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._stream)
        self._writer.writerow(["time"] + [f"j{j}_{kind}" for j in range(1, 7)
                                          for kind in ("target", "actual", "error")]
                              + ["feedback_age", "phase"])
        self._errors: list[list[float]] = [[] for _ in range(6)]
        self.peak = (0.0, "J1", 0.0)

    def add(self, t: float, target: Sequence[float], actual: Sequence[float],
            age: float, phase: str) -> tuple[float, ...]:
        errors = tuple(wrap_to_pi(a - b) for a, b in zip(target, actual))
        self._writer.writerow([f"{t:.6f}"] +
                              [f"{x:.8f}" for triple in zip(target, actual, errors)
                               for x in triple] + [f"{age:.6f}", phase])
        for j, error in enumerate(errors):
            value = abs(error)
            self._errors[j].append(value)
            if value > self.peak[0]:
                self.peak = (value, f"J{j+1}", t)
        return errors

    def summary(self) -> str:
        lines = []
        for j, values in enumerate(self._errors):
            if not values:
                lines.append(f"J{j+1}: no valid feedback")
                continue
            ordered = sorted(values)
            p95 = ordered[min(len(ordered)-1, math.ceil(0.95*len(ordered))-1)]
            lines.append(f"J{j+1}: max={max(values):.4f} rms={math.sqrt(mean(x*x for x in values)):.4f} "
                         f"mean={mean(values):.4f} p95={p95:.4f} final={values[-1]:.4f} rad")
        lines.append(f"peak={self.peak[0]:.4f} rad at {self.peak[1]} t={self.peak[2]:.3f}s")
        return "\n".join(lines)

    def close(self) -> None:
        self._stream.close()
