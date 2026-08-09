"""Threaded perception publisher.

Runs the MediaPipe pose loop on a background thread and publishes the latest
result to a shared slot, so a consumer (the teleoperator's get_action) can read
it instantly without blocking on inference. This decouples perception (~camera
rate) from the control/record loop, which must return promptly at its own fps.

One thread owns one camera; stereo (later) is two instances plus a combiner that
pairs their snapshots by timestamp. No cv2 GUI here -- imshow/waitKey are
main-thread affine, so a consumer displays `annotated_frame` on its own thread.
"""
import threading
import time
from collections import namedtuple

import cv2

from perception.pose_detector import PoseDetector, draw_landmarks_on_image

# Immutable snapshot published each frame. shoulder/wrist/visibility are None
# when no person is detected. `timestamp` is a monotonic capture time (seconds)
# for pairing across cameras; `frame_id` lets a consumer spot a stale repeat.
Snapshot = namedtuple(
    "Snapshot",
    ["shoulder", "wrist", "visibility", "frame_id", "timestamp", "annotated_frame"],
)


class PerceptionThread:
    """Background webcam + PoseDetector loop with a latest-value publish slot."""

    def __init__(self, camera_index: int = 0):
        self.camera_index = camera_index
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._latest: Snapshot | None = None   # single-reference slot (atomic swap)
        self._error: Exception | None = None
        self._frame_id = 0

    # --- lifecycle ---------------------------------------------------------
    def start(self) -> None:
        self._stop.clear()
        self._error = None
        self._thread = threading.Thread(target=self._run, name="PerceptionThread", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    # --- consumer API (thread-safe reads) ----------------------------------
    def latest(self) -> Snapshot | None:
        """The most recent snapshot, or None before the first frame. Reading a
        single immutable object is atomic under the GIL, so a consumer never
        sees a half-written snapshot."""
        return self._latest

    @property
    def error(self) -> Exception | None:
        """The exception that stopped the loop, if any. A thread that raises
        dies silently, so we stash it here for the consumer to notice."""
        return self._error

    # --- worker ------------------------------------------------------------
    def _run(self) -> None:
        cap = None
        try:
            detector = PoseDetector(mode="video")
            cap = cv2.VideoCapture(self.camera_index)
            if not cap.isOpened():
                raise RuntimeError(f"Could not open camera index {self.camera_index}.")

            start = time.perf_counter()
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError("Camera read failed (disconnected?).")

                now = time.perf_counter()
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # MediaPipe VIDEO mode needs a monotonic timestamp in ms.
                result = detector.detect(rgb, int((now - start) * 1000))
                annotated, _ = draw_landmarks_on_image(rgb, result)

                arm = detector.extract_arm(result)
                shoulder, wrist, visibility = arm if arm is not None else (None, None, None)

                self._frame_id += 1
                # Build the whole snapshot, then publish with one atomic assignment.
                self._latest = Snapshot(shoulder, wrist, visibility,
                                        self._frame_id, now, annotated)
        except Exception as e:   # a raising thread dies silently -- capture it
            self._error = e
        finally:
            if cap is not None:
                cap.release()


# --- demo: run `python -m perception.perception_thread` (needs a camera) -------
def _demo() -> None:
    pt = PerceptionThread(camera_index=0)
    pt.start()
    last_id = -1
    loops = 0          # consumer iterations this window
    new_frames = 0     # producer frames (frame_id changes) this window
    prod_fps = cons_fps = 0.0
    t0 = time.perf_counter()
    try:
        while True:
            if pt.error is not None:
                raise pt.error
            snap = pt.latest()
            loops += 1
            if snap is not None:
                if snap.frame_id != last_id:
                    last_id = snap.frame_id
                    new_frames += 1
                # Refresh the rate readout ~twice a second. producer = camera/
                # MediaPipe throughput; consumer = how fast this loop can sample
                # latest() -- the gap is the decoupling get_action() relies on.
                dt = time.perf_counter() - t0
                if dt >= 0.5:
                    prod_fps, cons_fps = new_frames / dt, loops / dt
                    loops = new_frames = 0
                    t0 = time.perf_counter()
                bgr = cv2.cvtColor(snap.annotated_frame, cv2.COLOR_RGB2BGR)
                cv2.putText(
                    bgr,
                    f"frame_id {snap.frame_id}  producer {prod_fps:4.1f} Hz  consumer {cons_fps:5.0f} Hz",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA,
                )
                cv2.imshow("PerceptionThread", bgr)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        pt.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    _demo()
