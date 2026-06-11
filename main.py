"""
Wires perception, bridge, and hardware into the live teleop loop.
"""
import numpy as np
from bridge import bridge
from perception import video_mapping
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

config = SO101FollowerConfig(
    port="COM3",       # find with lerobot's find_port utility
    id="jlo",      # must match the id set when calibrating arm
    use_degrees=True,
)

robot = SO101Follower(config)
robot.connect(calibrate=False)  # calibrate=False assumes you've already run calibration


def run() -> None:
    arm = bridge.Robot(bridge.M, bridge.Blist, bridge.limits, bridge.THETALIST_REST, 0.5)     # alpha = 0.5, tweakable

    ## MAIN LOOP
    for shoulder, wrist, key in video_mapping.landmark_stream():
        
        # Checks calibration initialisation, waits until scale is set
        if key == ord('c'):
            arm.calibrate(shoulder, wrist)
        elif arm.scale is None:
            continue

        thetalist, success = arm.step(shoulder, wrist)
        deg = np.degrees(thetalist)
        action = {
            "shoulder_pan.pos":  deg[0],
            "shoulder_lift.pos": deg[1],
            "elbow_flex.pos":    deg[2],
            "wrist_flex.pos":    deg[3],
            "wrist_roll.pos":    deg[4],
            "gripper.pos":       0,
        }
        robot.send_action(action)

# Standard calling
if __name__ == "__main__": run()