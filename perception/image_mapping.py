from pathlib import Path

import cv2
import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.tasks.python import BaseOptions

from perception.pose_detector import draw_landmarks_on_image

MODEL_PATH = Path(__file__).resolve().parent.parent / "models" / "pose_landmarker_heavy.task"

## Step 2: Creating PoseLandmarker object:
base_options = BaseOptions(model_asset_path=str(MODEL_PATH))
options = vision.PoseLandmarkerOptions(
    base_options=base_options,
    output_segmentation_masks=True)
detector = vision.PoseLandmarker.create_from_options(options)

# STEP 3: Load the input image.
image = mp.Image.create_from_file("Pose 2.jpg")

# STEP 4: Detect pose landmarks from the input image.
detection_result = detector.detect(image)

# STEP 5: Process the detection result. In this case, visualize it.
annotated_image, _ = draw_landmarks_on_image(image.numpy_view(), detection_result)

cv2.imshow("Pose Detection", cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
cv2.waitKey(0)
cv2.destroyAllWindows()
