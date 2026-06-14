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
# Per-joint cap on how far a single action may move from the present position.
# Folding the arm is the largest reconfiguration (~44 deg of elbow travel), so a
# small cap makes it creep over many frames and never complete during normal hand
# motion. 20 deg lets it fold in ~2-3 frames while still bounding a runaway jump.
# (Targets are already smoothed + workspace-clamped + success-gated upstream.)
MAX_RELATIVE_TARGET = 10.0

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
        for shoulder, wrist, key in video_mapping.landmark_stream():
            # Hold off teleop until the operator calibrates their arm length ('c').
            if key == ord('c'):
                arm.calibrate(shoulder, wrist)
                print(f"calibrated: arm_length={arm.arm_length:.2f}")
            elif arm.arm_length is None:
                continue

            thetalist, success = arm.step(shoulder, wrist)
            print(f"success={success}  arm_length={arm.arm_length:.2f}  deg={np.degrees(thetalist).round(1)}")
            if success:
                action = arm.to_action(thetalist)
                follower.send_action(action)
        
    finally:
        try:
            ramp_to(arm.to_action(bridge.THETALIST_STOW))
        except ConnectionError:
            print("Could not stow arm: connection lost.")
        follower.disconnect()
    

# Standard calling
if __name__ == "__main__": run()