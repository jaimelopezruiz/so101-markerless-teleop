"""
Wires perception, bridge, and hardware into the live teleop loop.
"""
import time
import math
from bridge import bridge
from perception import video_mapping
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


STARTUP_STEP_DEG = 1.5
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

def run() -> None:
    arm = bridge.RobotArm(bridge.M, bridge.Blist, bridge.limits, bridge.THETALIST_REST, 0.5)     # alpha = 0.5, tweakable
    
    follower.connect(calibrate=False)
    try:
        ## STARTUP RAMP
        start_pose = follower.get_observation()
        target_pose = arm.to_action(bridge.THETALIST_CALIB)
        deltas = [abs(target_pose[k] - start_pose[k]) for k in target_pose]
        N = max(1, math.ceil(max(deltas) / STARTUP_STEP_DEG))
        for i in range(1, N + 1):
            action = {k: start_pose[k] + (i / N) * (target_pose[k] - start_pose[k]) for k in target_pose}
            follower.send_action(action)
            time.sleep(DT)

        arm.theta_prev = bridge.THETALIST_CALIB.copy()
        
        ## MAIN LOOP
        for shoulder, wrist, key in video_mapping.landmark_stream():
            
            # Checks calibration initialisation, waits until scale is set
            if key == ord('c'):
                arm.calibrate(shoulder, wrist)
            elif arm.scale is None:
                continue

            thetalist, success = arm.step(shoulder, wrist)
            if success:
                action = arm.to_action(thetalist)
                follower.send_action(action)
        
    finally:
        follower.disconnect()
    

# Standard calling
if __name__ == "__main__": run()