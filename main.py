"""
Wires perception, bridge, and hardware into the live teleop loop.
"""
import time
import math
import numpy as np
from bridge import bridge
from perception import video_mapping
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


STARTUP_STEP_DEG = 5
DT = 1/30
MAX_RELATIVE_TARGET = 5.0
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
    arm = bridge.RobotArm(bridge.M, bridge.Blist, bridge.limits, bridge.THETALIST_REST, 0.5)     # alpha = 0.5, tweakable
    
    follower.connect(calibrate=False)
    try:
        ## STARTUP RAMP
        ramp_to(arm.to_action(bridge.THETALIST_CALIB))
        arm.theta_prev = bridge.THETALIST_CALIB.copy()
        
        ## MAIN LOOP
        for shoulder, wrist, key in video_mapping.landmark_stream():
            # Checks calibration initialisation, waits until scale is set
            if key == ord('c'):
                arm.calibrate(shoulder, wrist)
                print(f"calibrated: scale={arm.scale:.2f}")
            elif arm.scale is None:
                continue

            thetalist, success = arm.step(shoulder, wrist)
            print(f"success={success}  scale={arm.scale}  deg={np.degrees(thetalist).round(1)}")
            if success:
                action = arm.to_action(thetalist)
                follower.send_action(action)
        
    finally:
        ramp_to(arm.to_action(bridge.THETALIST_STOW))
        follower.disconnect()
    

# Standard calling
if __name__ == "__main__": run()