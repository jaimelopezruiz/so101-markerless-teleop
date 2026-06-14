"""
Wires perception, bridge, and hardware into the live teleop loop.
"""
import time
import numpy as np
from bridge import bridge
from perception import video_mapping
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


STARTUP_STEP_DEG = 5.0
DT = 1/30
# Per-joint cap (deg) on how far a single action may move from the present
# position. Folding is the largest reconfiguration (~44 deg of elbow travel), so
# too small a cap makes it creep over many frames and never complete during
# normal hand motion; at 10 deg it folds in ~5 frames while still bounding a
# runaway jump. (Targets are already smoothed + radius-bounded + success-gated
# upstream.) Raise toward 20 for snappier folds, or None to remove the cap.
MAX_RELATIVE_TARGET = 10.0

VIS_THRESH = 0.5    # min landmark visibility to trust a frame for driving the arm
CALIB_FRAMES = 20   # frames averaged (median) when measuring the operator's arm length

PORT = "COM3"       # find with lerobot's find_port utility
ROBOT_ID = "jlo"    # must match the id set when calibrating arm


config = SO101FollowerConfig(
    port=PORT,       
    id=ROBOT_ID,      
    use_degrees=True,
    max_relative_target= MAX_RELATIVE_TARGET
)

follower = SO101Follower(config)

def ramp_to(target_pose: dict, step_deg: float = STARTUP_STEP_DEG,
            dt: float = DT, tol: float = 1.0, max_steps: int = 400) -> None:
    for _ in range(max_steps):
        present = follower.get_observation()
        gaps = {k: target_pose[k] - present[k] for k in target_pose}
        max_gap = max(abs(g) for g in gaps.values())
        if max_gap <= tol:
            return
        ratio = min(step_deg / max_gap, 1.0)          # fraction of the gap to close this step
        action = {k: present[k] + ratio * gaps[k] for k in target_pose}
        follower.send_action(action)
        time.sleep(dt)

def run() -> None:
    arm = bridge.RobotArm(bridge.M, bridge.Blist, bridge.limits, bridge.THETALIST_REST,
                          min_cutoff=1.0, beta=1.5)     # one-euro smoothing, tweakable

    follower.connect(calibrate=False)
    try:
        ## STARTUP RAMP
        # Ramp to REST (the teleop neutral) and seed theta_prev from it, so the
        # first step() has no jump: hand-at-neutral maps straight to this pose.
        ramp_to(arm.to_action(bridge.THETALIST_REST))
        arm.theta_prev = bridge.THETALIST_REST.copy()
        
        ## MAIN LOOP
        calibrating = False
        frame = 0
        for shoulder, wrist, key in video_mapping.landmark_stream():
            frame += 1

            # Ignore frames where the tracked joints aren't confidently seen, so
            # low-confidence landmarks never drive the arm.
            if min(getattr(shoulder, "visibility", 1.0),
                   getattr(wrist, "visibility", 1.0)) < VIS_THRESH:
                continue

            # 'c' (re)starts arm-length calibration; average a few frames before use.
            if key == ord('c'):
                calibrating = True
                arm.start_calibration()
            if calibrating:
                if arm.calibrate(shoulder, wrist, n_samples=CALIB_FRAMES):
                    calibrating = False
                    print(f"calibrated: arm_length={arm.arm_length:.3f} m")
                continue
            if arm.arm_length is None:   # hold off teleop until calibrated
                continue

            thetalist, success = arm.step(shoulder, wrist)
            if not success or frame % 15 == 0:   # throttle: heartbeat + every IK hold
                print(f"success={success}  deg={np.degrees(thetalist).round(1)}")
            if success:
                follower.send_action(arm.to_action(thetalist))
        
    finally:
        try:
            ramp_to(arm.to_action(bridge.THETALIST_STOW))
        except ConnectionError:
            print("Could not stow arm: connection lost.")
        follower.disconnect()
    

# Standard calling
if __name__ == "__main__": run()