"""Pure state machine for safe dual-arm controller ownership handoff."""

from dataclasses import dataclass, field


VALID_ROBOTS = ("rebotarm", "piperh")


@dataclass
class ArmOwnership:
    """Require both Xbox command sources to report locked before handoff."""

    selected: str = "rebotarm"
    pending: str = ""
    armed: dict[str, bool | None] = field(
        default_factory=lambda: {name: None for name in VALID_ROBOTS}
    )

    def request(self, robot: str) -> str:
        target = robot.strip().lower()
        if target not in VALID_ROBOTS:
            raise ValueError("robot must be rebotarm or piperh")
        if target == self.selected and not self.pending:
            return self.selected
        self.pending = target
        self.selected = "none"
        return self.selected

    def report_armed(self, robot: str, armed: bool) -> str:
        if robot not in self.armed:
            raise ValueError(f"unknown robot: {robot}")
        self.armed[robot] = bool(armed)
        if self.pending and all(value is False for value in self.armed.values()):
            self.selected = self.pending
            self.pending = ""
        return self.selected

