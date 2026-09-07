"""Resolve the dedicated RS00 USB-TTL port without guessing other devices."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping


PREFERRED_PORT = (
    "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE6063665-if00"
)
USB_TTL_GLOB = "usb-1a86_USB_Single_Serial_*-if00"


def resolve_uart_port(
    explicit: str | None = None,
    *,
    environ: Mapping[str, str] | None = None,
    by_id_dir: Path = Path("/dev/serial/by-id"),
    preferred_port: str = PREFERRED_PORT,
) -> str:
    """Resolve an explicit/env port or the only known RS00 USB-TTL adapter.

    Arbitrary ttyUSB/ttyACM devices are intentionally never selected: sending
    chassis commands to a guessed serial device is unsafe. Multiple matching
    adapters require an explicit choice.
    """
    if explicit:
        return explicit

    environment = os.environ if environ is None else environ
    configured = environment.get("RS00_UART_PORT")
    if configured:
        return configured

    preferred = Path(preferred_port)
    if preferred.exists():
        return str(preferred)

    matches = sorted(
        str(path) for path in by_id_dir.glob(USB_TTL_GLOB) if path.exists()
    )
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        choices = ", ".join(matches)
        raise RuntimeError(
            "发现多个 RS00 USB-TTL 候选，请用 --port 或 "
            f"RS00_UART_PORT 明确选择：{choices}"
        )
    return preferred_port
