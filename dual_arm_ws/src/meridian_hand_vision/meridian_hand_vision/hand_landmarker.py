from __future__ import annotations

from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np


HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
)


class HandLandmarker:
    def __init__(self, model_path: Path, num_hands: int = 2) -> None:
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=num_hands,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self._detector = mp.tasks.vision.HandLandmarker.create_from_options(options)

    def detect(self, frame_bgr: np.ndarray, timestamp_ms: int):
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=np.ascontiguousarray(rgb),
        )
        return self._detector.detect_for_video(image, timestamp_ms)

    def close(self) -> None:
        self._detector.close()

    def __enter__(self) -> "HandLandmarker":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def landmark_pixels(landmarks, width: int, height: int) -> list[tuple[int, int]]:
    return [
        (
            int(np.clip(point.x, 0.0, 1.0) * (width - 1)),
            int(np.clip(point.y, 0.0, 1.0) * (height - 1)),
        )
        for point in landmarks
    ]


def draw_hand_landmarks(frame: np.ndarray, result) -> None:
    height, width = frame.shape[:2]
    for hand_index, landmarks in enumerate(result.hand_landmarks):
        points = landmark_pixels(landmarks, width, height)
        for start, end in HAND_CONNECTIONS:
            cv2.line(frame, points[start], points[end], (60, 220, 60), 2, cv2.LINE_AA)
        for index, point in enumerate(points):
            color = (0, 80, 255) if index == 0 else (0, 255, 255)
            cv2.circle(frame, point, 8 if index == 0 else 4, color, -1, cv2.LINE_AA)

        label = "Hand"
        score = 0.0
        if hand_index < len(result.handedness) and result.handedness[hand_index]:
            category = result.handedness[hand_index][0]
            label = category.category_name or category.display_name or "Hand"
            score = float(category.score or 0.0)
        wrist_x, wrist_y = points[0]
        cv2.putText(
            frame,
            f"{label} {score:.2f}",
            (max(8, wrist_x - 60), max(25, wrist_y - 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
