"""HARDWARE live test for the MarkerlessTeleop wrapper (PLAN C4).

Standalone loop: MarkerlessTeleop.get_action() -> SO101Follower.send_action().
Stands in for lerobot-record until C5 wires up the real CLI, and is the harness
C4's acceptance is checked against:
  - no first-tick jump (arm holds PARK until the gate arms)
  - hold-last on occlusion / low visibility
  - tracks the operator once armed

Needs the arm connected AND a webcam. Not part of the offline suite.
Run:  python -m tests.teleop_live      ('q' in the window, or Ctrl+C, to stop)
"""
import time

import cv2
import numpy as np
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from bridge import bridge
from lerobot_teleoperator_markerless import MarkerlessTeleop, MarkerlessTeleopConfig

PORT = "COM3"                 # find with lerobot's find_port utility
ROBOT_ID = "jlo"              # must match the id set when calibrating the arm
CAMERA_INDEX = 0
MAX_RELATIVE_TARGET = 10.0    # float! an int raises TypeError inside lerobot
DT = 1 / 30                   # control tick (T2 will formalise this)
STARTUP_STEP_DEG = 5.0


def ramp_to(follower, target_pose: dict, step_deg: float = STARTUP_STEP_DEG,
            dt: float = DT, tol: float = 1.0, max_steps: int = 400) -> None:
    """Closed-loop synchronized ramp (same policy as main.py): re-read present
    position each step, move the largest-gap joint by step_deg and the rest by
    the same fraction, so they arrive together without tripping the clamp."""
    for _ in range(max_steps):
        present = follower.get_observation()
        gaps = {k: target_pose[k] - present[k] for k in target_pose}
        max_gap = max(abs(g) for g in gaps.values())
        if max_gap <= tol:
            return
        ratio = min(step_deg / max_gap, 1.0)
        follower.send_action({k: present[k] + ratio * gaps[k] for k in target_pose})
        time.sleep(dt)


def _overlay(teleop, snap) -> np.ndarray:
    """Annotated camera frame + gate telemetry. Display stays on THIS (main)
    thread -- the perception thread deliberately owns no cv2 GUI.

    Unarmed, the scalar distance says how far but not which way, so split it
    into the two knobs the operator actually controls: reach (arm extension)
    and angle (hand direction).
    """
    bgr = cv2.cvtColor(snap.annotated_frame, cv2.COLOR_RGB2BGR)
    lines = []
    if teleop._armed:
        lines.append(("ARMED - teleop live", (0, 255, 0)))
    elif teleop._last_xz is None:
        lines.append(("holding PARK | no valid landmarks yet", (0, 165, 255)))
    else:
        dist = float(np.linalg.norm(np.array(teleop._last_xz) - teleop.arm.park_xz))
        r_now, a_now = teleop.arm.polar(teleop._last_xz)
        r_park, a_park = teleop.arm.polar(teleop.arm.park_xz)
        lines.append((
            f"holding PARK | {dist * 100:5.1f} cm from ready "
            f"(need <{teleop.config.gate_eps * 100:.1f}) | "
            f"{teleop._ready_count}/{teleop.config.gate_n_frames}",
            (0, 165, 255),
        ))
        # Actionable deltas: which way to move, not just how far off.
        reach_hint = "extend" if r_now < r_park else "fold"
        angle_hint = "raise" if a_now < a_park else "lower"
        lines.append((
            f"reach {r_now * 100:5.1f} -> {r_park * 100:.1f} cm  ({reach_hint} arm)",
            (255, 255, 255),
        ))
        lines.append((
            f"angle {a_now:+6.1f} -> {a_park:+.1f} deg  ({angle_hint} hand)",
            (255, 255, 255),
        ))
    for i, (text, colour) in enumerate(lines):
        cv2.putText(bgr, text, (10, 30 + 26 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2, cv2.LINE_AA)
    return bgr


def run() -> None:
    follower = SO101Follower(SO101FollowerConfig(
        port=PORT, id=ROBOT_ID, use_degrees=True,
        max_relative_target=MAX_RELATIVE_TARGET,
    ))
    teleop = MarkerlessTeleop(MarkerlessTeleopConfig(camera_index=CAMERA_INDEX))

    follower.connect(calibrate=False)
    try:
        teleop.connect()          # starts perception, then blocks for arm-length calibration
        # Ramp to PARK before any teleop action: theta_prev is seeded to PARK, so
        # the first get_action() then commands the pose the arm is already in.
        ramp_to(follower, teleop.arm.to_action(bridge.THETALIST_PARK))
        print("ready -- bring your hand to the ready pose to arm ('q' to quit)")

        while True:
            follower.send_action(teleop.get_action())
            snap = teleop.perception.latest()
            if snap is not None and snap.annotated_frame is not None:
                cv2.imshow("teleop_live", _overlay(teleop, snap))
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        cv2.destroyAllWindows()
        teleop.disconnect()
        try:
            ramp_to(follower, teleop.arm.to_action(bridge.THETALIST_PARK))
        except ConnectionError:
            print("could not park arm: connection lost.")
        follower.disconnect()


if __name__ == "__main__":
    run()
