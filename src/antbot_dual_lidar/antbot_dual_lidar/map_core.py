"""ROS-independent bounded voxel accumulation used by the RViz 3D map."""

from collections import OrderedDict

import numpy as np


class BoundedVoxelMap:
    """Keep one representative point record per XYZ voxel with a hard bound."""

    def __init__(self, leaf_size: float, max_points: int):
        if not np.isfinite(leaf_size) or leaf_size <= 0.0:
            raise ValueError("leaf_size must be finite and positive")
        if max_points <= 0:
            raise ValueError("max_points must be positive")
        self.leaf_size = float(leaf_size)
        self.max_points = int(max_points)
        self._points = OrderedDict()
        self._dimension = None

    def clear(self) -> None:
        self._points.clear()
        self._dimension = None

    def __len__(self) -> int:
        return len(self._points)

    def update(self, points) -> int:
        xyz = np.asarray(points, dtype=np.float64)
        if xyz.ndim != 2 or xyz.shape[1] < 3:
            raise ValueError("points must have shape (N, M) with M >= 3")
        if self._dimension is not None and xyz.shape[1] != self._dimension:
            raise ValueError("point record dimension changed during accumulation")
        xyz = xyz[np.isfinite(xyz[:, :3]).all(axis=1)]
        if not len(xyz):
            return 0
        self._dimension = xyz.shape[1]

        keys = np.floor(xyz[:, :3] / self.leaf_size).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        for index in np.sort(indices):
            key = tuple(int(value) for value in keys[index])
            self._points[key] = xyz[index].astype(np.float32)
            self._points.move_to_end(key)

        overflow = len(self._points) - self.max_points
        for _ in range(max(0, overflow)):
            self._points.popitem(last=False)
        return len(indices)

    def points(self) -> np.ndarray:
        if not self._points:
            return np.empty((0, self._dimension or 3), dtype=np.float32)
        return np.asarray(list(self._points.values()), dtype=np.float32)
