"""Diagnostic: determine the lerobot <-> URDF joint SIGN and OFFSET in isolation.

This is deliberately decoupled from IK, teleop, the rest pose, and the startup
ramp, because all of those confound an eyeball sign test. The only ground truth
used is get_observation(), which returns RAW motor degrees that never pass
through JOINT_SIGN (that only affects to_action OUTPUT). We compare those raw
readings, via FK, against where the gripper physically is.

Joint order / indices (this is the slot each JOINT_SIGN entry controls):
    0 shoulder_pan   1 shoulder_lift   2 elbow_flex   3 wrist_flex   4 wrist_roll

----------------------------------------------------------------------------
PROCEDURE
----------------------------------------------------------------------------
0. In bridge.py start from a clean hypothesis: JOINT_SIGN = [1,1,1,1,1],
   JOINT_OFFSET = [0,0,0,0,0]. (Re-run this script after each change.)

1. OFFSET CHECK. Move the arm by hand to the URDF home pose: arm pointing
   roughly straight forward, horizontal. Every joint should read ~0 and the
   printed FK EE should be ~ (0.39, 0.00, 0.23). Any joint far from 0 at true
   home is a nonzero JOINT_OFFSET for that joint.

2. SIGN CHECK, one joint at a time. The script prints, per joint, the change in
   reading since you pressed Enter ("d=") and the change in the FK-predicted EE.
   Press Enter to zero the baseline, move EXACTLY ONE joint a clear amount, and
   look at that joint:
       - predicted EE moves the SAME way as the real gripper -> sign is correct.
       - predicted EE moves the OPPOSITE way to the real gripper -> set that
         joint's JOINT_SIGN entry to -1.
   Re-press Enter between joints to re-zero. After flipping any sign, re-run and
   confirm every joint now predicts EE in the same direction as reality.

Reference: at home, +angle in the URDF frame moves the gripper like this, so a
correct sign should reproduce these directions:
    shoulder_pan +  -> gripper swings horizontally to one side
    shoulder_lift + -> gripper tips forward and down
    elbow_flex +    -> gripper moves down
    wrist_flex +    -> gripper tips down
    wrist_roll +    -> gripper rolls about its own axis

Usage (arm connected on COM3, id 'jlo'):
    python -m tests.joint_check
Press Enter to re-zero the baseline, Ctrl+C to stop.
"""
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from kinematics.core import FKinBody
from bridge import bridge
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

config = SO101FollowerConfig(
    port="COM3",
    id="jlo",
    use_degrees=True,
    max_relative_target=None,   # diagnostic only: no commands are sent
)


def lerobot_deg_to_urdf_rad(deg: np.ndarray) -> np.ndarray:
    """Inverse of bridge.to_action's sign/offset map, then degrees -> radians."""
    return np.radians((deg - bridge.JOINT_OFFSET) / bridge.JOINT_SIGN)


def fk_ee(deg: np.ndarray) -> np.ndarray:
    return FKinBody(bridge.M, bridge.Blist, lerobot_deg_to_urdf_rad(deg))[:3, 3]


def main() -> None:
    robot = SO101Follower(config)
    robot.connect(calibrate=False)
    robot.bus.disable_torque()
    print(f"Torque OFF. JOINT_SIGN={bridge.JOINT_SIGN.tolist()}  JOINT_OFFSET={bridge.JOINT_OFFSET.tolist()}")
    print("Move ONE joint at a time. Press Enter to re-zero baseline, Ctrl+C to stop.\n")

    # Enter on a background thread re-zeros the baseline without blocking the read loop.
    rezero = threading.Event()
    threading.Thread(target=lambda: [rezero.set() for _ in iter(input, None)], daemon=True).start()

    base_deg = np.array([robot.get_observation()[f"{j}.pos"] for j in ORDER])
    base_ee = fk_ee(base_deg)
    try:
        while True:
            if rezero.is_set():
                base_deg = np.array([robot.get_observation()[f"{j}.pos"] for j in ORDER])
                base_ee = fk_ee(base_deg)
                rezero.clear()
                print("--- baseline re-zeroed ---")
            deg = np.array([robot.get_observation()[f"{j}.pos"] for j in ORDER])
            ddeg = deg - base_deg
            dee = fk_ee(deg) - base_ee
            joints = "  ".join(f"{j}:d={dd:+6.1f}" for j, dd in zip(ORDER, ddeg))
            print(f"{joints}   |  dEE(x,y,z)={dee.round(3)}")
            time.sleep(0.15)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
