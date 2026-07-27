import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from types import SimpleNamespace

from bridge.bridge import RobotArm, WS_CENTER, WS_RMIN, MAP_RMAX, EXT_MIN_FRAC, THETALIST_PARK, THETALIST_NEUTRAL
from kinematics.core import Adjoint, TransInv, FKinBody
from urdf.parser import findMnS

M, Slist, limits = findMnS()
# Body-frame screw axes: Blist = [Ad_{M^-1}] Slist, column by column.
Blist = np.array([Adjoint(TransInv(M)) @ Slist[:, i] for i in range(Slist.shape[1])]).T

robot_test = RobotArm(M, Blist, limits, theta_park=THETALIST_PARK, theta_neutral=THETALIST_NEUTRAL)

ARM = 0.7
shoulder = SimpleNamespace(x=0.0, y=0.0)
wrist_extended = SimpleNamespace(x=ARM, y=0.0)        # full extension (calibration pose)
wrist_folded = SimpleNamespace(x=0.05, y=-0.05)       # wrist near shoulder -> folded
wrist_mid = SimpleNamespace(x=0.3, y=0.2)             # partial extension, below shoulder


def ee_radius(thetalist):
    p = FKinBody(M, Blist, thetalist)[:3, 3]
    return np.linalg.norm([p[0] - WS_CENTER[0], p[2] - WS_CENTER[1]])


# Calibration accumulates samples and sets arm_length (median) once enough arrive.
robot_test.start_calibration()
done = False
for _ in range(20):
    done = robot_test.calibrate(shoulder, wrist_extended, n_samples=20)
assert done is True
assert abs(robot_test.arm_length - ARM) < 1e-4

# Full extension -> reach radius near MAP_RMAX (robot extended).
th_ext, ok_ext = robot_test.step(shoulder, wrist_extended)
assert ok_ext is True
assert abs(ee_radius(th_ext) - MAP_RMAX) < 0.03, ee_radius(th_ext)

# Folded arm -> reach radius near WS_RMIN (robot curled), strictly tighter than extended.
robot_test.prev_filtered = None   # reset the one-euro filter between scripted moves
th_fold, ok_fold = robot_test.step(shoulder, wrist_folded)
assert ok_fold is True
assert ee_radius(th_fold) < ee_radius(th_ext) - 0.1, (ee_radius(th_fold), ee_radius(th_ext))

# Mid extension returns a well-formed result.
robot_test.prev_filtered = None
th_mid, ok_mid = robot_test.step(shoulder, wrist_mid)
assert isinstance(th_mid, np.ndarray) and len(th_mid) == 5
assert isinstance(ok_mid, bool)

print("OK  arm_length:", round(robot_test.arm_length, 3),
      " r_extended:", round(ee_radius(th_ext), 3),
      " r_folded:", round(ee_radius(th_fold), 3))
