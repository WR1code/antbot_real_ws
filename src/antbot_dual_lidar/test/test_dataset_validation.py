from antbot_dual_lidar.dataset_validation import REQUIRED_TOPICS, assess_dataset


def test_manifest_rejects_missing_topic_and_regression():
    counts = {topic: 1 for topic in REQUIRED_TOPICS}
    ranges = {
        "/antbot/lidar/front_left/points_raw_native": [100, 200],
        "/antbot/lidar/rear_right/points_raw_native": [100, 200],
        "/antbot/imu/data": [90, 210],
    }
    assert assess_dataset(counts, ranges, {})["passed"]
    counts["/antbot/imu/data"] = 0
    assert not assess_dataset(counts, ranges, {})["passed"]
    counts["/antbot/imu/data"] = 1
    assert not assess_dataset(counts, ranges, {"/antbot/imu/data": 1})["passed"]


def test_manifest_requires_positive_common_imu_lidar_coverage():
    counts = {topic: 1 for topic in REQUIRED_TOPICS}
    ranges = {
        "/antbot/lidar/front_left/points_raw_native": [100, 200],
        "/antbot/lidar/rear_right/points_raw_native": [100, 200],
        "/antbot/imu/data": [300, 400],
    }
    result = assess_dataset(counts, ranges, {})
    assert not result["passed"]
    assert result["imu_lidar_common_coverage"]["common_duration_sec"] == 0.0


def test_reset_case_requires_regression_on_clock_and_every_primary_sensor():
    counts = {topic: 1 for topic in REQUIRED_TOPICS}
    ranges = {
        "/antbot/lidar/front_left/points_raw_native": [0, 200],
        "/antbot/lidar/rear_right/points_raw_native": [0, 200],
        "/antbot/imu/data": [0, 200],
    }
    regressions = {topic: 1 for topic in (
        "/clock",
        "/antbot/lidar/front_left/points_raw_native",
        "/antbot/lidar/rear_right/points_raw_native",
        "/antbot/imu/data_raw",
        "/antbot/imu/data",
        "/antbot/ground_truth/odom",
    )}
    assert assess_dataset(counts, ranges, regressions, expect_reset=True)["passed"]
    regressions.pop("/antbot/imu/data")
    assert not assess_dataset(counts, ranges, regressions, expect_reset=True)["passed"]
