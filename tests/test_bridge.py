import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from types import SimpleNamespace

from main import Robot
from kinematics.ik import IKinBodyDLS
from kinematics.core import Adjoint, TransInv, FKinBody
from urdf.parser import findMnS

THETALIST_REST = np.array([0, -1.3, 0, 0, 0])

M, Slist, limits = findMnS()
# Body-frame screw axes: Blist = [Ad_{M^-1}] Slist, column by column.
Blist = np.array([Adjoint(TransInv(M)) @ Slist[:, i] for i in range(Slist.shape[1])]).T

# Determining "reach" of robot for calibration later
thetalist = np.array([0, 1.7, -1.69, 0, 0])   # Robot pointing forward approx full extension
T = FKinBody(M, Blist, thetalist)
reach_xyz = FKinBody(M, Blist, THETALIST_REST)[:3, 3] - T[:3, 3]
REACH = np.linalg.norm(reach_xyz[[0, 2]])    # Magnitude of vector difference between full reach and rest point

robot_test = Robot(M, Blist, limits, THETALIST_REST, 0.5)

shoulder = SimpleNamespace(x= 0.0, y = 0.0)
wrist_rest = SimpleNamespace(x = 0.0, y = 0.0)
wrist_forward = SimpleNamespace(x = 0.7, y = 0.0)
wrist_mid = SimpleNamespace(x = 0.3, y = 0.2)   # slightly below shoulder

robot_test.calibrate(shoulder, wrist_forward)

assert abs(robot_test.scale - REACH / 0.7) < 1e-4

result = robot_test.step(shoulder, wrist_rest)

print("scale:", robot_test.scale)
print("T_rest:", robot_test.T_rest)

assert result[1] is True
assert np.allclose(result[0], THETALIST_REST, atol=0.1)

result = robot_test.step(shoulder, wrist_mid)
assert isinstance(result[0], np.ndarray) and len(result[0]) == 5
assert isinstance(result[1], bool)

