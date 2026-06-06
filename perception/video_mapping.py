from pathlib import Path
import time

import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from PIL import Image

from perception.pose_detector import draw_landmarks_on_image

ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH = ROOT / "models" / "pose_landmarker_heavy.task"

# --- DEVLOG capture: set True to record a short GIF of live tracking ---
RECORD_GIF = True
GIF_PATH = ROOT / "docs" / "media" / "live_tracking.gif"
GIF_MAX_FRAMES = 120   # stop collecting after this many kept frames (~ a few seconds)
GIF_STRIDE = 3         # keep every Nth frame to shrink the file
GIF_WIDTH = 360        # downscale width (px) for a lighter GIF
WINDOW = "Live Pose Detection"

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
if not cap.isOpened():
    raise RuntimeError("Could not open webcam (device 0). Is a camera connected and available?")

gif_frames = []          # collected RGB frames for the optional GIF
frame_idx = 0
start_time = time.time()  # Start measuring time for timestamps for MediaPipe

if RECORD_GIF:
    print(f"Recording live tracking. Press 'q' in the '{WINDOW}' window to stop and save "
          f"(auto-stops after {GIF_MAX_FRAMES} kept frames).")

try:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            raise RuntimeError("Failed to read frame from webcam; the camera may have been disconnected.")

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

        cv2.imshow(WINDOW, display_image)

        # Optionally collect a downscaled frame for the DEVLOG GIF
        if RECORD_GIF and len(gif_frames) < GIF_MAX_FRAMES and frame_idx % GIF_STRIDE == 0:
            h, w = annotated_image.shape[:2]
            small = cv2.resize(annotated_image, (GIF_WIDTH, int(h * GIF_WIDTH / w)))
            gif_frames.append(Image.fromarray(small))
            if len(gif_frames) >= GIF_MAX_FRAMES:
                break  # collected enough; stop and save
        frame_idx += 1

        ## EXTRACTING PARAMETERS:
        # First check there are parameters to extract
        if world_landmarks_list:
            person_landmarks = world_landmarks_list[0]   # <- This assumes only one person in frame

            # Taking the RIGHT arm pose locations (3D world coords in metres):
            shoulder = person_landmarks[12]
            wrist = person_landmarks[16]

        # Stop on 'q' or if the window is closed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            break
finally:
    # Always release the camera, close windows, and save whatever we collected,
    # even if the loop was interrupted (Ctrl+C, window closed, exception).
    cap.release()
    cv2.destroyAllWindows()
    if RECORD_GIF and gif_frames:
        GIF_PATH.parent.mkdir(parents=True, exist_ok=True)
        gif_frames[0].save(
            GIF_PATH, save_all=True, append_images=gif_frames[1:],
            duration=80, loop=0, optimize=True,
        )
        print(f"Saved {len(gif_frames)} frames to {GIF_PATH}")
    elif RECORD_GIF:
        print("No frames were captured, so nothing was saved.")
