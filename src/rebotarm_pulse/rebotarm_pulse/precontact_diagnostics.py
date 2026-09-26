"""Structured JSONL diagnostics for Piper-H pre-contact attempts."""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import threading
from typing import Any


REASON_CODES = (
    "NONE", "EARLIER_CANDIDATE_SELECTED",
    "WAIT_TARGET", "TARGET_STABILIZING", "TARGET_STALE", "TARGET_LOST", "TARGET_MOVED",
    "TARGET_TEMPORARILY_STALE", "WAIT_TARGET_RECOVERY",
    "TARGET_HARD_JUMP", "TARGET_ALREADY_PASSED", "WAIT_ARM_DIRECTION", "ARM_DIRECTION_STALE",
    "ARM_DIRECTION_UNSTABLE", "JOINT_FEEDBACK_STALE", "WRONG_ARM_SELECTED",
    "XBOX_NOT_LOCKED", "PRESSURE_DUPLICATE_PUBLISHERS", "PRESSURE_NOT_CONNECTED", "PRESSURE_NOT_ZEROED",
    "PRESSURE_STALE", "PRESSURE_UNSTABLE", "PRESSURE_DELTA_ABORT",
    "PRESSURE_EMERGENCY_ABORT",
    "POSITION_CORRECTION_TOO_LARGE", "HOVER_OUT_OF_WORKSPACE",
    "HOVER_IK_FAILED", "HOVER_COLLISION", "PRECONTACT_OUT_OF_WORKSPACE",
    "PRECONTACT_IK_FAILED", "PRECONTACT_COLLISION", "CARTESIAN_PATH_INCOMPLETE",
    "TOOL_ENVELOPE_COLLISION", "NO_VALID_APPROACH_CANDIDATE", "TF_LOOKUP_FAILED",
    "INTERNAL_ERROR",
)


class PlanDiagnostics:
    def __init__(self, directory: str, now: datetime | None = None) -> None:
        stamp = now or datetime.now()
        path = Path(directory).expanduser()
        path.mkdir(parents=True, exist_ok=True)
        self.path = path / f"pulse_precontact_{stamp:%Y%m%d_%H%M%S_%f}.jsonl"
        self._stamp = stamp
        self._sequence = 0
        self.plan_id = ""
        self._lock = threading.Lock()

    def new_plan(self, now: datetime | None = None) -> str:
        stamp = now or datetime.now()
        self._sequence += 1
        self.plan_id = f"PIPERH-{stamp:%Y%m%d-%H%M%S%f}-{self._sequence:03d}"
        return self.plan_id

    def write(self, event_type: str, **fields: Any) -> dict[str, Any]:
        event = {
            "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "event_type": event_type,
            "plan_id": self.plan_id,
            **fields,
        }
        line = json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        with self._lock:
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(line + "\n")
        return event
