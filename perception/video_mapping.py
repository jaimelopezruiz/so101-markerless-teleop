from pathlib import Path
import time

import cv2

from perception.pose_detector import PoseDetector, draw_landmarks_on_image

ROOT = Path(__file__).resolve().parent.parent

# --- DEVLOG capture: set True to record a short GIF of live tracking ---
RECORD_GIF = False
GIF_PATH = ROOT / "docs" / "media" / "live_tracking.gif"
GIF_MAX_FRAMES = 120   # stop collecting after this many kept frames (~ a few seconds)
GIF_STRIDE = 3         # keep every Nth frame to shrink the file
GIF_WIDTH = 360        # downscale width (px) for a lighter GIF
WINDOW = "Live Pose Detection"

# --------- MAIN FUNCTION ------------#

def landmark_stream() -> tuple[object, object, int]:
    detector = PoseDetector(mode="video")

    # Start capturing from webcam (0 usually the default camera)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam (device 0). Is a camera connected and available?")

    try:
        start_time = time.time()
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                raise RuntimeError("Failed to read frame from webcam; the camera may have been disconnected.")

            key = None

            # OpenCV uses BGR, but MediaPipe expects RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Detect poses (VIDEO mode needs a monotonic timestamp in ms)
            timestamp_ms = int((time.time() - start_time) * 1000)
            detection_result = detector.detect(rgb_frame, timestamp_ms)

            # Process and visualize the result
            annotated_image, _ = draw_landmarks_on_image(rgb_frame, detection_result)

            # Convert back to BGR so OpenCV displays the colors correctly
            display_image = cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR)

            cv2.imshow(WINDOW, display_image)

            ## CHECKING FOR INPUT:
            key = cv2.waitKey(1) & 0xFF

            ## EXTRACTING PARAMETERS:
            # Right shoulder/wrist world landmarks (metres), or None if no person.
            arm = detector.extract_arm(detection_result)
            if arm is not None:
                shoulder, wrist, _visibility = arm
                yield shoulder, wrist, key

            # Stop on 'q' or if the window is closed
            if key == ord('q') or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break


    finally:
        # Always release the camera and close windows, even if the loop was
        # interrupted (Ctrl+C, window closed, exception).
        cap.release()
        cv2.destroyAllWindows()
