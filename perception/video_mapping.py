from pathlib import Path
import time

import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from perception.pose_detector import draw_landmarks_on_image

MODEL_PATH = Path(__file__).resolve().parent.parent / "models" / "pose_landmarker_heavy.task"

# Create PoseLandmarker object for VIDEO mode
base_options = python.BaseOptions(model_asset_path=str(MODEL_PATH))
options = vision.PoseLandmarkerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.VIDEO,
    output_segmentation_masks=True
)

detector = vision.PoseLandmarker.create_from_options(options)

# Start capturing from webcam (0 usually the default camera)
cap = cv2.VideoCapture(0)

start_time = time.time()  # Start measuring time for timestamps for MediaPipe
while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("Ignoring empty camera frame.")
        continue

    # OpenCV uses BGR, but MediaPipe expects RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

    # Calculate the timestamp in milliseconds
    timestamp_ms = int((time.time() - start_time) * 1000)

    # Detect poses using detect_for_video()
    detection_result = detector.detect_for_video(mp_image, timestamp_ms)

    # Process and visualize the result
    annotated_image, world_landmarks_list = draw_landmarks_on_image(mp_image.numpy_view(), detection_result)

    # Convert back to BGR so OpenCV displays the colors correctly
    display_image = cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR)

    cv2.imshow("Live Pose Detection", display_image)

    ## EXTRACTING PARAMETERS:
    # First check there are parameters to extract
    if world_landmarks_list:
        person_landmarks = world_landmarks_list[0]   # <- This assumes only one person in frame

        # Taking the RIGHT arm pose locations (3D world coords in metres):
        shoulder = person_landmarks[12]
        wrist = person_landmarks[16]

    # Press "q" to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up windows and camera when done
cap.release()
cv2.destroyAllWindows()
