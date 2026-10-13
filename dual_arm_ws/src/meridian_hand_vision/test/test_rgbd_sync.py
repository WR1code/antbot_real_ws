from meridian_hand_vision.rgbd_sync import validate_rgb_depth_stamps


def test_rgb_depth_pair_over_30_ms_is_rejected():
    valid, delta_ms, reason, stale = validate_rgb_depth_stamps(
        1_000_000_000, 1_031_000_000, 30_000_000
    )
    assert not valid
    assert delta_ms == 31.0
    assert reason == "RGB_DEPTH_TIME_MISMATCH"
    assert stale == "rgb"


def test_rgb_depth_pair_at_30_ms_is_accepted():
    valid, delta_ms, reason, stale = validate_rgb_depth_stamps(
        1_000_000_000, 1_030_000_000, 30_000_000
    )
    assert valid
    assert delta_ms == 30.0
    assert reason is None
    assert stale == ""


def test_rgb_depth_pair_with_missing_stamp_is_rejected():
    valid, delta_ms, reason, stale = validate_rgb_depth_stamps(
        0, 1_000_000_000, 30_000_000
    )
    assert not valid
    assert delta_ms is None
    assert reason == "MISSING_SENSOR_TIMESTAMP"
    assert stale == "both"
