"""
Wires perception, bridge, and hardware into the live teleop loop.
"""

from bridge import bridge
from perception import video_mapping


def run() -> None:
    robot = bridge.Robot(bridge.M, bridge.Blist, bridge.limits, bridge.THETALIST_REST, 0.5)     # alpha = 0.5, tweakable

    ## MAIN LOOP
    for shoulder, wrist, key in video_mapping.landmark_stream():
        
        # Checks calibration initialisation, waits until scale is set
        if key == ord('c'):
            robot.calibrate(shoulder, wrist)
        elif robot.scale is None:
            continue

        thetalist, success = robot.step(shoulder, wrist)
        print(thetalist, success)

# Standard calling
if __name__ == "__main__": run()