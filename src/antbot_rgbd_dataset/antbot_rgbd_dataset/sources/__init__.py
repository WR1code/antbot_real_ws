"""RGB-D source adapters. Simulation is optional and never imported by core."""

from .base import RGBDSource
from .dataset import DatasetRGBDSource
from .live_ros2 import LiveROS2RGBDSource

__all__ = ["RGBDSource", "DatasetRGBDSource", "LiveROS2RGBDSource"]
