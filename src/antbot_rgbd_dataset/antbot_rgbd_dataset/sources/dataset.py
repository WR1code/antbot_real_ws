"""Phase 2/3 compatible dataset source producing the unified data contract."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from ..models import CameraIntrinsics, RGBDFrame


class DatasetRGBDSource:
    def __init__(self, dataset_root: str | Path):
        self.root = Path(dataset_root).resolve()
        for name in ("frames.csv", "intrinsics.yaml", "dataset_metadata.yaml"):
            if not (self.root / name).is_file():
                raise FileNotFoundError(self.root / name)
        self.dataset_metadata = yaml.safe_load(
            (self.root / "dataset_metadata.yaml").read_text(encoding="utf-8")
        )
        raw_intrinsics = yaml.safe_load(
            (self.root / "intrinsics.yaml").read_text(encoding="utf-8")
        )
        self.intrinsics = CameraIntrinsics(
            width=int(raw_intrinsics["image_width"]),
            height=int(raw_intrinsics["image_height"]),
            fx=float(raw_intrinsics["fx"]), fy=float(raw_intrinsics["fy"]),
            cx=float(raw_intrinsics["cx"]), cy=float(raw_intrinsics["cy"]),
            distortion_model=str(raw_intrinsics.get("distortion_model", "plumb_bob")),
            distortion_coefficients=tuple(raw_intrinsics.get("D", [])),
            frame_id=str(raw_intrinsics.get("camera_frame", "")),
            rectified=not any(abs(float(v)) > 1e-12 for v in raw_intrinsics.get("D", [])),
        )
        self.intrinsics.validate()
        self.records = self._read_index()
        self._cursor = 0
        self._started = False

    def _resolve(self, relative: str) -> Path:
        path = (self.root / relative).resolve()
        if self.root not in path.parents:
            raise ValueError(f"dataset path escapes root: {relative}")
        return path

    def _read_index(self) -> list[dict]:
        with (self.root / "frames.csv").open(newline="", encoding="utf-8") as stream:
            records = list(csv.DictReader(stream))
        required = {"frame_index", "timestamp_ns", "rgb_path", "depth_npy_path", "pose_path"}
        if not records or not required.issubset(records[0]):
            raise ValueError("frames.csv is empty or missing required fields")
        stamps = [int(row["timestamp_ns"]) for row in records]
        if any(a >= b for a, b in zip(stamps, stamps[1:])):
            raise ValueError("dataset timestamps are not strictly increasing")
        return records

    def start(self) -> None:
        self._cursor = 0
        self._started = True

    def stop(self) -> None:
        self._started = False

    def read(self) -> RGBDFrame | None:
        if not self._started:
            raise RuntimeError("DatasetRGBDSource.start() must be called before read()")
        if self._cursor >= len(self.records):
            return None
        row = self.records[self._cursor]
        self._cursor += 1
        with Image.open(self._resolve(row["rgb_path"])) as image:
            color = np.asarray(image.convert("RGB"))
        depth = np.load(self._resolve(row["depth_npy_path"]), allow_pickle=False)
        pose = json.loads(self._resolve(row["pose_path"]).read_text(encoding="utf-8"))
        world = str(pose.get("fixed_frame", self.dataset_metadata.get("coordinate_frame", "odom")))
        matrix_key = f"T_{world}_camera"
        matrix = np.asarray(pose.get(matrix_key, pose.get("T_odom_camera")), dtype=np.float64)
        stamp = int(row["timestamp_ns"])
        camera_frame = str(pose.get("camera_frame", self.intrinsics.frame_id))
        frame_metadata_path = self.root / "frame_metadata" / f"{int(row['frame_index']):06d}.yaml"
        extra = (
            yaml.safe_load(frame_metadata_path.read_text(encoding="utf-8"))
            if frame_metadata_path.is_file() else {}
        )
        frame = RGBDFrame(
            timestamp_ns=stamp,
            frame_index=int(row["frame_index"]),
            color=color,
            depth_m=depth,
            camera_intrinsics=self.intrinsics,
            T_world_camera=matrix,
            pose_source=str(extra.get("pose_source", "dataset")),
            color_frame_id=camera_frame,
            depth_frame_id=str(extra.get("frame_ids", {}).get("depth", camera_frame)),
            camera_frame_id=camera_frame,
            world_frame_id=world,
            color_timestamp_ns=int(extra.get("color_timestamp_ns", stamp)),
            depth_timestamp_ns=int(extra.get("depth_timestamp_ns", stamp)),
            pose_timestamp_ns=int(extra.get("pose_timestamp_ns", stamp)),
            depth_aligned_to_color=bool(extra.get("depth_aligned_to_color", True)),
            robot_id=str(extra.get("robot_id", "antbot")),
            sensor_id=str(extra.get("sensor_id", "rgbd")),
            metadata={"dataset_row": row, **extra},
        )
        frame.validate()
        return frame

    def __iter__(self):
        self.start()
        try:
            while True:
                frame = self.read()
                if frame is None:
                    break
                yield frame
        finally:
            self.stop()
