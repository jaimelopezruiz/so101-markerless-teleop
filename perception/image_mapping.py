"""Still-image pose detection demo.

Runs MediaPipe Pose on a single image, draws the landmarks, saves the annotated
result to docs/media/ for the DEVLOG, and previews it in a window.

Usage (from the repo root):
    python -m perception.image_mapping path/to/arm_photo.jpg
"""
import sys
from pathlib import Path

import cv2

from perception.pose_detector import PoseDetector, draw_landmarks_on_image

ROOT = Path(__file__).resolve().parent.parent
MEDIA_DIR = ROOT / "docs" / "media"

# Input image: first CLI argument, or a default next to the repo root.
image_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "Pose 2.jpg"
if not image_path.is_file():
    raise FileNotFoundError(
        f"Input image not found: {image_path}\n"
        f"Pass one explicitly:  python -m perception.image_mapping <path-to-photo>"
    )

# Load as RGB (cv2 reads BGR).
bgr = cv2.imread(str(image_path))
if bgr is None:
    raise ValueError(f"Could not read image (unsupported format or corrupt file): {image_path}")
rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

# Detect and annotate.
detector = PoseDetector(mode="image")
detection_result = detector.detect(rgb)
annotated_image, _ = draw_landmarks_on_image(rgb, detection_result)
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
