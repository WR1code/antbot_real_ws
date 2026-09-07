"""Common RGB-D source lifecycle."""

from typing import Protocol

from ..models import RGBDFrame


class RGBDSource(Protocol):
    def start(self) -> None: ...
    def read(self) -> RGBDFrame | None: ...
    def stop(self) -> None: ...
