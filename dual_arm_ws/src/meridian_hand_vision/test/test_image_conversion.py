from types import SimpleNamespace

import numpy as np

from meridian_hand_vision.image_conversion import (
    color_message_to_bgr,
    depth_message_to_meters,
)


def test_rgb8_with_row_padding() -> None:
    message = SimpleNamespace(
        width=2,
        height=1,
        step=8,
        encoding="rgb8",
        data=bytes([255, 0, 0, 0, 255, 0, 99, 99]),
    )
    frame = color_message_to_bgr(message)
    assert frame.shape == (1, 2, 3)
    assert np.array_equal(frame[0, 0], [0, 0, 255])
    assert np.array_equal(frame[0, 1], [0, 255, 0])


def test_16uc1_depth_to_meters() -> None:
    values = np.asarray([[1000, 2500]], dtype="<u2")
    message = SimpleNamespace(
        width=2,
        height=1,
        step=4,
        encoding="16UC1",
        is_bigendian=False,
        data=values.tobytes(),
    )
    depth = depth_message_to_meters(message)
    assert np.allclose(depth, [[1.0, 2.5]])
