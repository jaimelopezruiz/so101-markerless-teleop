"""
Bridge layer: converts MediaPipe landmarks into IK targets for the SO-101.
"""
import time

import numpy as np

from kinematics.ik import IKinBodyDLS
from kinematics.core import Adjoint, TransInv, FKinBody
from urdf.parser import findMnS

M, Slist, limits = findMnS()
Blist = np.array([Adjoint(TransInv(M)) @ Slist[:, i] for i in range(Slist.shape[1])]).T

# All poses are in the URDF joint frame (radians). The URDF<->motor map is
# identity (JOINT_SIGN all +1, JOINT_OFFSET 0, verified with tests/joint_check.py),
# so these are also the motor commands. Do NOT sign-flip these to fix motor
# behaviour: the only place a motor sign belongs is JOINT_SIGN.
# REST is a central pose (EE ~(0.30, 0.30), mid-radius) so hand motion folds AND
# extends the arm; it drives T_rest, R_fixed and theta_pref.
THETALIST_REST = np.radians([0, -37.7, 11.4, 19.2, 0])
THETALIST_CALIB = np.radians([0, 75, -75, -10, 0])   # Robot pointing forward approx full extension
THETALIST_STOW = np.radians([0, -105, 96, 80, 0])    # Parked
THETALIST_INTERM = np.radians([0, -70, 35, 47, 0])

T = FKinBody(M, Blist, THETALIST_CALIB)
reach_xyz = FKinBody(M, Blist, THETALIST_REST)[:3, 3] - T[:3, 3]
REACH = np.linalg.norm(reach_xyz[[0, 2]])    # Magnitude of vector difference between full reach and rest point

# --- lerobot joint-frame calibration --------------------------------------
# lerobot accepts/reports each joint in TRUE degrees referenced to the motor's
# mechanical mid-range (motors_bus._normalize, DEGREES mode). Our IK angles are
# true degrees in the URDF frame. Both are 1:1 in real degrees, so the only
# corrections are a per-joint sign and a constant offset.
#   JOINT_OFFSET[i]: lerobot degrees read when the arm is physically at URDF
#                    home (theta=0). Measured ~0 for all joints because the URDF
#                    limits are symmetric, so lerobot's mid-range zero coincides
#                    with the URDF zero. Re-measure with tests/joint_check.py.
#   JOINT_SIGN[i]:   +1 if the motor's +deg direction matches the URDF +axis,
#                    else -1. Verify per joint with tests/joint_check.py.
# Verified identity (all +1, offset 0): isolated per-joint test matched FK to the
# physical gripper at home and through the startup ramps.
JOINT_SIGN   = np.array([1.0, 1.0, 1.0, 1.0, 1.0])
JOINT_OFFSET = np.array([0.0, 0.0, 0.0, 0.0, 0.0])   # degrees, lerobot frame

# --- reachable workspace (x-z plane, pan=0) -------------------------------
# Annulus fit by sweeping shoulder_lift & elbow over their limits: centre is the
# shoulder pivot, radii are the folded/extended reach. Targets are clamped into
# this band so unreachable commands no longer stall or saturate the IK.
WS_CENTER = np.array([0.07, 0.18])   # (x, z) metres
WS_RMIN, WS_RMAX = 0.165, 0.42       # RMIN at the physical fold limit so the arm
                                     # can curl as tightly as it reaches; RMAX
                                     # just inside the true max (0.447)

# --- radial (extension) mapping -------------------------------------------
# Map the operator's ARM EXTENSION (|shoulder->wrist|) to the robot's reach
# RADIUS, and the hand DIRECTION to the EE angle about WS_CENTER. Folding the arm
# (wrist toward shoulder) shrinks the radius and curls the robot; extending it
# reaches out. This replaces a hand-position->EE-position map, under which a
# folded robot required reaching the hand ~50 cm across the torso (anatomically
# impossible), so deep folds were never commandable.
# Extension fraction over [EXT_MIN_FRAC, 1] -> reach radius over [WS_RMIN, MAP_RMAX].
EXT_MIN_FRAC = 0.15   # arm this fraction extended -> fully folded robot (WS_RMIN)
MAP_RMAX = 0.40       # radius at full extension; < WS_RMAX so edge targets stay reachable

# Null-space posture bias: how hard IK pulls the redundant joints toward the
# rest posture each iteration. Keeps the elbow from folding into awkward poses
# without disturbing position tracking. 0 disables it.
NULLSPACE_GAIN = 0.3

class RobotArm:

    def __init__(self, M, Blist, limits, theta_prev, min_cutoff=1.0, beta=1.5, d_cutoff=1.0):
        self.M = M
        self.Blist = Blist
        self.limits = limits
        T_rest_full = FKinBody(self.M, self.Blist, theta_prev)
        self.T_rest = T_rest_full[:3, 3]
        self.R_fixed = T_rest_full[:3, :3]
        self.theta_prev = theta_prev.copy()
        self.theta_pref = theta_prev.copy()   # fixed preferred posture for null-space biasing
        self.arm_length = None                # set by calibrate(); gates teleop until then
        self.prev_dir = np.array([0.0, 1.0])  # last EE direction (fallback when hand near shoulder)
        # One-euro filter state/params (see _smooth).
        self.min_cutoff = min_cutoff   # Hz: lower = more smoothing when the hand is still
        self.beta = beta               # responsiveness: higher = less lag when moving fast
        self.d_cutoff = d_cutoff       # Hz: cutoff for the speed estimate
        self.prev_filtered = None
        self.prev_raw = None
        self.dx_filtered = None
        self.prev_time = None

    @staticmethod
    def _ema_alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def _smooth(self, raw: np.ndarray) -> np.ndarray:
        """One-euro filter on the 2D relative vector: heavy smoothing when the
        hand is slow/still (kills jitter), light smoothing when moving fast (low
        lag). Replaces the fixed-alpha EMA, whose single alpha forced a static
        jitter-vs-lag tradeoff."""
        now = time.perf_counter()
        if self.prev_filtered is None:
            self.prev_filtered = raw.copy()
            self.prev_raw = raw.copy()
            self.dx_filtered = np.zeros_like(raw)
            self.prev_time = now
            return self.prev_filtered

        dt = now - self.prev_time
        if dt <= 0.0:
            dt = 1e-3
        self.prev_time = now

        # Low-pass the derivative, then set the cutoff from the smoothed speed.
        dx = (raw - self.prev_raw) / dt
        self.prev_raw = raw.copy()
        a_d = self._ema_alpha(self.d_cutoff, dt)
        self.dx_filtered = a_d * dx + (1.0 - a_d) * self.dx_filtered
        speed = np.linalg.norm(self.dx_filtered)

        cutoff = self.min_cutoff + self.beta * speed
        a = self._ema_alpha(cutoff, dt)
        self.prev_filtered = a * raw + (1.0 - a) * self.prev_filtered
        return self.prev_filtered

    def calibrate(self, shoulder: object, wrist: object):
        """Records the operator's full arm extension (|shoulder->wrist| in the x-y
        plane), used to normalise extension into a robot reach radius in step().

        TODO (Tier 3): average arm_length over several frames to reject landmark
        noise. That needs the caller to sample multiple frames, so it lives in
        main.py's loop, not here."""
        dx, dy = wrist.x - shoulder.x, wrist.y - shoulder.y
        self.arm_length = np.linalg.norm([dx, dy])

    @staticmethod
    def _clamp_to_workspace(x: float, z: float) -> tuple[float, float]:
        """Clamps an (x, z) target into the reachable annulus so out-of-range
        commands are pulled to the nearest reachable point instead of stalling
        or saturating the IK."""
        v = np.array([x, z]) - WS_CENTER
        r = np.linalg.norm(v)
        if r < 1e-9:
            return x, z
        r_clamped = np.clip(r, WS_RMIN, WS_RMAX)
        p = WS_CENTER + v * (r_clamped / r)
        return float(p[0]), float(p[1])

    def step(self, shoulder: object, wrist: object):
        """Smooths the shoulder->wrist vector, maps arm extension->reach radius and
        hand direction->EE angle, builds T_sd, solves IK, returns (thetalist, success)."""

        # Smoothed relative vector (one-euro).
        rel = self._smooth(np.array([wrist.x - shoulder.x, wrist.y - shoulder.y]))

        # Extension -> reach radius. Fraction of full arm extension over
        # [EXT_MIN_FRAC, 1] maps linearly onto [WS_RMIN, MAP_RMAX].
        ext_frac = np.clip(
            (np.linalg.norm(rel) / self.arm_length - EXT_MIN_FRAC) / (1.0 - EXT_MIN_FRAC),
            0.0, 1.0,
        )
        radius = WS_RMIN + ext_frac * (MAP_RMAX - WS_RMIN)

        # Hand direction -> EE direction in robot x-z (+x fwd, +z up). MediaPipe y
        # is down, so flip it. Near the shoulder the direction is undefined; hold
        # the last one to avoid snapping.
        direction = np.array([rel[0], -rel[1]])
        n = np.linalg.norm(direction)
        if n > 1e-6:
            self.prev_dir = direction / n
        robot_x, robot_z = WS_CENTER + radius * self.prev_dir
        robot_x, robot_z = self._clamp_to_workspace(robot_x, robot_z)

        # Forming 4x4 matrix for that position and (for now) fixed orientation
        T_sd = np.eye(4)
        T_sd[:3, :3] = self.R_fixed
        T_sd[:3, 3] = np.array([robot_x, self.T_rest[1], robot_z])

        # Calling IK function -> (thetalist, success)
        # position_only=True for now: locking orientation to a single R_fixed
        # shrinks the reachable set (~72%->55% IK success on a workspace sweep)
        # and would freeze the arm too often. Orientation tracking is Tier 2.
        # theta_pref/k0: bias the redundant joints (elbow especially) toward the
        # rest posture so the arm stays natural instead of folding up.
        IK_result = IKinBodyDLS(self.Blist, self.M, T_sd, self.theta_prev, self.limits, ev=1e-2,
                                position_only=True, theta_pref=self.theta_pref, k0=NULLSPACE_GAIN)
        if IK_result[1] is True:
            self.theta_prev = IK_result[0]     # advance ONLY on success — keeps the redundant joints pinned
        return IK_result

    @staticmethod
    def to_action(theta_urdf: np.ndarray) -> dict[str, float]:
        # URDF radians -> lerobot degrees: per-joint sign and offset (see the
        # JOINT_SIGN / JOINT_OFFSET notes above).
        deg = JOINT_SIGN * np.degrees(theta_urdf) + JOINT_OFFSET
        action = {
            "shoulder_pan.pos":  deg[0],
            "shoulder_lift.pos": deg[1],
            "elbow_flex.pos":    deg[2],
            "wrist_flex.pos":    deg[3],
            "wrist_roll.pos":    deg[4],
            "gripper.pos":       0,
        }
        return action
