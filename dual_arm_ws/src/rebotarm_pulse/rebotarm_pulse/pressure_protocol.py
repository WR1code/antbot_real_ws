"""Parse the ESP32-S3 / ICP-20100 firmware's serial measurement lines."""

from __future__ import annotations

import math
import re


_MEASUREMENT = re.compile(
    r"^(S[123])\s+Pressure\s*=\s*([-+]?\d+(?:\.\d+)?)\s+hPa"
    r"\s+Temperature\s*=\s*([-+]?\d+(?:\.\d+)?)\s+C\s*$",
    re.IGNORECASE,
)


def parse_pressure_line(line: str) -> tuple[str, float, float] | None:
    """Return channel, absolute pressure in hPa, and Celsius; ignore diagnostics."""
    match = _MEASUREMENT.fullmatch(line.strip())
    if match is None:
        return None
    pressure = float(match.group(2))
    temperature = float(match.group(3))
    if not math.isfinite(pressure) or not math.isfinite(temperature):
        return None
    if not 100.0 <= pressure <= 1500.0 or not -50.0 <= temperature <= 125.0:
        return None
    return match.group(1).upper(), pressure, temperature
