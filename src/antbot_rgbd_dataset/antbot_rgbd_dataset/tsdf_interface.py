"""Source-independent incremental TSDF interface reserved for online use."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from .models import RGBDFrame


@dataclass(frozen=True)
class IntegrationResult:
    accepted: bool
    reason: str = ""
    integration_time_ms: float = 0.0


class TSDFIntegrator(Protocol):
    def reset(self) -> None: ...
    def integrate(self, frame: RGBDFrame) -> IntegrationResult: ...
    def extract_pointcloud(self) -> Any: ...
    def extract_mesh(self) -> Any: ...
    def save_checkpoint(self, path: Path) -> None: ...
