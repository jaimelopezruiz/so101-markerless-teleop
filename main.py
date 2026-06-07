"""
Main pipeline script bridging Mediapipe landmarks, IK solver and hardware.
"""
import numpy as np

from kinematics.ik import IKinBodyDLS
from kinematics.core import Adjoint, TransInv, FKinBody
from urdf.parser import findMnS

M, Slist, limits = findMnS()
# Body-frame screw axes: Blist = [Ad_{M^-1}] Slist, column by column.
Blist = np.array([Adjoint(TransInv(M)) @ Slist[:, i] for i in range(Slist.shape[1])]).T

# Order matches how findMnS builds Slist (reversed URDF order, arm joints only)
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

THETALIST_REST = np.array([0, -1.3, 0, 0, 0])

# Determining "reach" of robot for calibration later
thetalist = np.array([0, 1.7, -1.69, 0, 0])   # Robot pointing forward approx full extension
T = FKinBody(M, Blist, thetalist)
reach_xyz = FKinBody(M, Blist, THETALIST_REST)[:3, 3] - T[:3, 3]
REACH = np.linalg.norm(reach_xyz[[0, 2]])    # Magnitude of vector difference between full reach and rest point

class Bridge:

    def __init__(self, M, Blist, limits, T_rest, theta_prev, alpha):
        self.M = M
        self.Blist = Blist
        self.limits = limits
        self.T_rest = T_rest
        self.theta_prev = theta_prev.copy()
        self.alpha = alpha
        self.scale = None
        self.prev_filtered = None

    def  _smooth(self, raw: np.ndarray) -> np.ndarray:
        """Smoothing function for Mediapipe landmark noise, using EMA."""
        if self.prev_filtered is None: self.prev_filtered = raw.copy() 
        else: self.prev_filtered = self.alpha * raw + (1 - self.alpha) * self.prev_filtered
        return self.prev_filtered
    
    def calibrate(self, shoulder: object, wrist: object):
        """Calibrates the scaling factor used to convert wrist to end-effector movement"""
        dx, dy = wrist.x - shoulder.x, wrist.y - shoulder.y
        extended = np.array([dx, dy])
        arm_length = np.linalg.norm(extended)
        self.scale = REACH / arm_length
    
    def step(self, shoulder: object, wrist: object):
        """Computes relative vector, calls _smooth, maps to robot target position, 
        builds 4x4 matrix, calls IK function and returns objective thetalist and success"""
        
        # Computing raw relative vector, smoothing it and mapping to robot target position
        rel_vector = np.array([wrist.x - shoulder.x, wrist.y - shoulder.y])
        smooth_rel_vector= self._smooth(rel_vector)
        robot_x = self.T_rest[0] + self.scale * smooth_rel_vector[0] 
        robot_z = self.T_rest[2] - self.scale * smooth_rel_vector[1]

        # Forming 4x4 matrix for that position and (for now) fixed orientation
        T_sd = np.eye(4)
        T_sd[:3, :3] = self.M[:3, :3]
        T_sd[:3, 3] = np.array([robot_x, self.T_rest[1], robot_z])

        # Calling IK function -> (thetalist, success)
        if (IK_result:= IKinBodyDLS(self.Blist, self.M, T_sd, self.theta_prev, self.limits))[1] is True:
            self.theta_prev = IK_result[0]
        
        return IK_result