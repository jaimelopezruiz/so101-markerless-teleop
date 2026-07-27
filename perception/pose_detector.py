"""MediaPipe pose-detection core.

One home for the SO-101 teleop's use of MediaPipe: model setup, per-frame
detection (IMAGE or VIDEO mode), right-arm landmark extraction, and drawing.
The still-image demo, the video generator, and the perception thread are all
thin clients of this module -- none of them touch mediapipe directly.
"""
from collections import namedtuple
from pathlib import Path

import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.vision import drawing_utils
from mediapipe.tasks.python.vision import drawing_styles

ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = ROOT / "models" / "pose_landmarker_heavy.task"

# Right-arm landmark indices in the MediaPipe pose model.
RIGHT_SHOULDER = 12
RIGHT_WRIST = 16

_RUNNING_MODE = {"image": vision.RunningMode.IMAGE, "video": vision.RunningMode.VIDEO}

# Lightweight, immutable landmark: copied floats, decoupled from MediaPipe's
# per-frame objects (which may be recycled after the next detect() call). Carries
# .x/.y/.z -- the interface bridge.RobotArm consumes -- plus .visibility.
Landmark = namedtuple("Landmark", ["x", "y", "z", "visibility"])


def _copy_landmark(lm) -> Landmark:
    return Landmark(lm.x, lm.y, lm.z, getattr(lm, "visibility", 1.0))


class PoseDetector:
    """Owns a MediaPipe PoseLandmarker. ``mode`` is ``"image"`` (still frames)
    or ``"video"`` (a stream, detection needs a monotonic timestamp)."""

    def __init__(self, mode: str = "image", model_path: Path = MODEL_PATH):
        if mode not in _RUNNING_MODE:
            raise ValueError(f"mode must be one of {list(_RUNNING_MODE)}, got {mode!r}")
        base_options = python.BaseOptions(model_asset_path=str(model_path))
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=_RUNNING_MODE[mode],
            output_segmentation_masks=True,
        )
        self._detector = vision.PoseLandmarker.create_from_options(options)
        self._video = mode == "video"

    def detect(self, rgb_frame: np.ndarray, timestamp_ms: int | None = None):
        """Runs detection on an RGB frame. ``timestamp_ms`` (monotonic, ms) is
        required in VIDEO mode and ignored in IMAGE mode."""
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        if self._video:
            return self._detector.detect_for_video(mp_image, timestamp_ms)
        return self._detector.detect(mp_image)

    @staticmethod
    def extract_arm(detection_result):
        """Returns ``(shoulder, wrist, visibility)`` for the right arm as copied
        ``Landmark``s, or ``None`` if no person was detected. Uses
        ``pose_world_landmarks`` (metric, hip-centred)."""
        world = detection_result.pose_world_landmarks
        if not world:
            return None
        person = world[0]   # assumes a single person in frame
        shoulder = _copy_landmark(person[RIGHT_SHOULDER])
        wrist = _copy_landmark(person[RIGHT_WRIST])
        return shoulder, wrist, min(shoulder.visibility, wrist.visibility)


def draw_landmarks_on_image(rgb_image, detection_result):
    """Draws MediaPipe pose landmarks onto a copy of the image.

    :param rgb_image: RGB image array
    :param detection_result: Result from a MediaPipe PoseLandmarker
    :return: (annotated_image, pose_world_landmarks_list)
    """
    pose_landmarks_list = detection_result.pose_landmarks
    pose_world_landmarks_list = detection_result.pose_world_landmarks
    annotated_image = np.copy(rgb_image)

    pose_landmark_style = drawing_styles.get_default_pose_landmarks_style()
    pose_connection_style = drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2)

    for pose_landmarks in pose_landmarks_list:
        drawing_utils.draw_landmarks(
            image=annotated_image,
            landmark_list=pose_landmarks,
            connections=vision.PoseLandmarksConnections.POSE_LANDMARKS,
            landmark_drawing_spec=pose_landmark_style,
            connection_drawing_spec=pose_connection_style)

    return annotated_image, pose_world_landmarks_list
