import numpy as np
import pytest

from antbot_dual_lidar.map_core import BoundedVoxelMap


def test_voxel_map_deduplicates_and_replaces_voxels():
    cloud = BoundedVoxelMap(0.1, 10)
    assert cloud.update([[0.01, 0.02, 0.03], [0.09, 0.08, 0.07]]) == 1
    assert len(cloud) == 1
    cloud.update([[0.15, 0.0, 0.0]])
    assert len(cloud) == 2
    assert cloud.points().shape == (2, 3)


def test_voxel_map_is_bounded_and_clearable():
    cloud = BoundedVoxelMap(0.1, 2)
    cloud.update([[0.0, 0.0, 0.0], [0.2, 0.0, 0.0], [0.4, 0.0, 0.0]])
    assert len(cloud) == 2
    assert np.allclose(cloud.points()[-1], [0.4, 0.0, 0.0])
    cloud.clear()
    assert len(cloud) == 0


def test_voxel_map_rejects_bad_configuration_and_shape():
    with pytest.raises(ValueError):
        BoundedVoxelMap(0.0, 1)
    with pytest.raises(ValueError):
        BoundedVoxelMap(0.1, 0)
    with pytest.raises(ValueError):
        BoundedVoxelMap(0.1, 1).update([1.0, 2.0, 3.0])


def test_voxel_map_preserves_intensity_record_column():
    cloud = BoundedVoxelMap(0.1, 10)
    cloud.update([[1.0, 2.0, 3.0, 42.0]])
    assert cloud.points().shape == (1, 4)
    assert cloud.points()[0, 3] == pytest.approx(42.0)
