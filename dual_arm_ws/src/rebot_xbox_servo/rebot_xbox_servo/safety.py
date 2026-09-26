"""Pure state helpers for the Xbox safety interlock."""

from dataclasses import dataclass


def a_button_rising_edge(current_a: int, previous_a: int) -> bool:
    """Return true exactly once for each physical A-button press."""
    a_pressed = current_a == 1 and previous_a == 0
    return a_pressed


@dataclass
class SafetyState:
    """Latched arm state; recovery from timeout never re-arms motion."""

    armed: bool = False
    joystick_timed_out: bool = False

    def toggle(self, sticks_centered: bool, require_centered: bool) -> str:
        if self.armed:
            self.armed = False
            return "locked"
        if require_centered and not sticks_centered:
            return "center_required"
        self.armed = True
        self.joystick_timed_out = False
        return "armed"

    def emergency_stop(self) -> None:
        self.armed = False

    def timeout(self) -> None:
        self.armed = False
        self.joystick_timed_out = True

    def reconnect(self) -> None:
        # Deliberately preserve armed=False. A fresh rising edge is required.
        self.joystick_timed_out = False


class SpeedSelector:
    """Bounded discrete speed selector, starting at the lowest tier."""

    def __init__(self, levels: list[float]) -> None:
        if not levels or any(level <= 0.0 or level > 1.0 for level in levels):
            raise ValueError("speed levels must be in the interval (0, 1]")
        if levels != sorted(set(levels)):
            raise ValueError("speed levels must be unique and ascending")
        self.levels = levels
        self.index = 0

    @property
    def multiplier(self) -> float:
        return self.levels[self.index]

    @property
    def percentage(self) -> int:
        return round(self.multiplier * 100.0)

    def increase(self) -> bool:
        old_index = self.index
        self.index = min(self.index + 1, len(self.levels) - 1)
        return self.index != old_index

    def decrease(self) -> bool:
        old_index = self.index
        self.index = max(self.index - 1, 0)
        return self.index != old_index


def bounded_tier_speed(
    configured_speed: float,
    multiplier: float,
    minimum_speed: float,
    maximum_speed: float,
) -> float:
    """Apply a tier while respecting both configured minimum and maximum."""
    if minimum_speed < 0.0 or maximum_speed <= 0.0:
        raise ValueError("speed limits must be positive")
    if minimum_speed > maximum_speed:
        raise ValueError("minimum speed cannot exceed maximum speed")
    return min(max(configured_speed * multiplier, minimum_speed), maximum_speed)
