from antbot_dual_lidar.phase2a_minimal_manifest import (
    classify_scan_coverage,
    clock_stamp_statistics,
)


def test_complete_scan_coverage_excludes_only_bag_boundaries():
    result = classify_scan_coverage(
        [100, 150, 200, 250, 300],
        [(80, 120), (110, 190), (220, 280), (290, 320)],
    )
    assert result["total_frames"] == 4
    assert result["boundary_dropped_frames"] == 2
    assert result["complete_coverage_frames"] == 2
    assert result["valid_coverage_ratio"] == 1.0
    assert result["valid_frames_100_percent_covered"]


def test_clock_statistics_distinguish_duplicate_types():
    result = clock_stamp_statistics([1, 1, 2, 1, 3, 3, 3])
    assert result == {
        "total_messages": 7,
        "unique_stamps": 3,
        "consecutive_duplicates": 3,
        "nonconsecutive_duplicates": 1,
        "regressions": 1,
        "maximum_duplicate_group_length": 3,
    }
