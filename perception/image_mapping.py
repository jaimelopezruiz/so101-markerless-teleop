"""Still-image pose detection demo.

Runs MediaPipe Pose on a single image, draws the landmarks, saves the annotated
result to docs/media/ for the DEVLOG, and previews it in a window.

Usage (from the repo root):
    python -m perception.image_mapping path/to/arm_photo.jpg
"""
import sys
from pathlib import Path

import cv2
import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.tasks.python import BaseOptions

from perception.pose_detector import draw_landmarks_on_image

ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = ROOT / "models" / "pose_landmarker_heavy.task"
MEDIA_DIR = ROOT / "docs" / "media"

# Input image: first CLI argument, or a default next to the repo root.
image_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "Pose 2.jpg"
if not image_path.is_file():
    raise FileNotFoundError(
        f"Input image not found: {image_path}\n"
        f"Pass one explicitly:  python -m perception.image_mapping <path-to-photo>"
    )

# Create the PoseLandmarker (IMAGE mode).
base_options = BaseOptions(model_asset_path=str(MODEL_PATH))
options = vision.PoseLandmarkerOptions(
    base_options=base_options,
    output_segmentation_masks=True)
detector = vision.PoseLandmarker.create_from_options(options)

# Detect and annotate.
image = mp.Image.create_from_file(str(image_path))
detection_result = detector.detect(image)
annotated_image, _ = draw_landmarks_on_image(image.numpy_view(), detection_result)
annotated_bgr = cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR)

# Save the annotated image for the DEVLOG.
MEDIA_DIR.mkdir(parents=True, exist_ok=True)
out_path = MEDIA_DIR / "pose_detection.png"
cv2.imwrite(str(out_path), annotated_bgr)
print(f"Saved annotated image to {out_path}")

# Preview (skipped silently if running headless).
try:
    cv2.imshow("Pose Detection", annotated_bgr)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
except cv2.error:
    pass
