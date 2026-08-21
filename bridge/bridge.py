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

# Joint poses in the URDF frame (radians). The URDF<->motor map is identity
# (JOINT_SIGN all +1, JOINT_OFFSET 0, verified with tests/joint_check.py), so
# these are also the motor commands. Never sign-flip these to fix motor
# behaviour -- a motor sign belongs only in JOINT_SIGN.
# NEUTRAL is the mapping reference: it sets the fixed gripper orientation
# R_fixed, the held y-coordinate, and the null-space posture bias. Chosen
# central (EE ~(0.30, 0.30)). PARK is the folded physical start pose (the
# measured torque-off limp reading): it seeds theta_prev / prev_dir and is the
# ramp target at both ends.
THETALIST_NEUTRAL = np.radians([0, -37.7, 11.4, 19.2, 0])
THETALIST_PARK = np.radians([0.57, -97.14, 96.53, 67.65, 1.63])    # folded start / seed / ramp target

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
# Shoulder-pivot centre and tightest reach radius, from an annulus fit (sweeping
# shoulder_lift & elbow over their limits; true reach is ~[0.165, 0.447] m). The
# radial mapping below builds targets within [WS_RMIN, MAP_RMAX], so every target
# is reachable by construction and needs no extra clamping.
WS_CENTER = np.array([0.07, 0.18])   # (x, z) metres, shoulder pivot
WS_RMIN = 0.165                      # tightest fold (physical min reach)

# --- radial (extension) mapping -------------------------------------------
# Map the operator's ARM EXTENSION (|shoulder->wrist|) to the robot's reach
# RADIUS, and the hand DIRECTION to the EE angle about WS_CENTER. Folding the arm
# (wrist toward shoulder) shrinks the radius and curls the robot; extending it
# reaches out. This replaces a hand-position->EE-position map, under which a
# folded robot required reaching the hand ~50 cm across the torso (anatomically
# impossible), so deep folds were never commandable.
# Extension fraction over [EXT_MIN_FRAC, 1] -> reach radius over [WS_RMIN, MAP_RMAX].
EXT_MIN_FRAC = 0.2   # arm this fraction extended -> fully folded robot (WS_RMIN)
MAP_RMAX = 0.32       # radius at full extension; inside the true max so edges stay reachable
EXT_BREAK_FRAC = 0.40      # input fraction (post-floor) where the gentle segment ends
RADIUS_BREAK_FRAC = 0.30   # output fraction reached at that breakpoint

# Null-space posture bias: how hard IK pulls the redundant joints toward the
# rest posture each iteration. Keeps the elbow from folding into awkward poses
# without disturbing position tracking. 0 disables it.
NULLSPACE_GAIN = 0.3

class RobotArm:

    def __init__(self, M, Blist, limits, *, theta_park, theta_neutral, min_cutoff=1.0, beta=1.5, d_cutoff=1.0):
        self.M = M
        self.Blist = Blist
        self.limits = limits
        # Mapping reference (held orientation + y) comes from NEUTRAL, not the
        # PARK start pose -- keeps R_fixed / held-y decoupled from the seed.
        T_rest_full = FKinBody(self.M, self.Blist, theta_neutral)
        self.T_rest = T_rest_full[:3, 3]
        self.R_fixed = T_rest_full[:3, :3]
        self.theta_prev = theta_park.copy()      # IK warm-start / hold-last seed (physical start pose)
        self.theta_pref = theta_neutral.copy()   # fixed preferred posture for null-space biasing
        self.arm_length = None                # set by calibrate(); gates teleop until then
        self._calib_buf = []                  # arm-length samples accumulated during calibration
        # Fallback EE direction when the hand is near the shoulder: unit vector
        # from WS_CENTER to PARK's end-effector, so the first live target (before
        # a hand direction is defined) lands on PARK.
        p = FKinBody(M, Blist, theta_park)[:3, 3]
        d = np.array([p[0], p[2]]) - WS_CENTER
        self.prev_dir = d / np.linalg.norm(d)
        # PARK's (x, z) in task space -- the ready-pose gate's reference point
        # (C4): teleop arms only once the live mapped target is back near here.
        self.park_xz = np.array([p[0], p[2]])
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

    def start_calibration(self) -> None:
        """Begins a fresh arm-length calibration (clears any prior samples)."""
        self._calib_buf = []

    def calibrate(self, shoulder: object, wrist: object, n_samples: int = 20) -> bool:
        """Accumulates one arm-extension sample (|shoulder->wrist| in the x-y plane).
        Once n_samples are collected, sets arm_length to their median (robust to the
        odd bad landmark frame) and returns True; otherwise returns False. The caller
        keeps feeding frames until it returns True (see main.py)."""
        self._calib_buf.append(np.linalg.norm([wrist.x - shoulder.x, wrist.y - shoulder.y]))
        if len(self._calib_buf) >= n_samples:
            self.arm_length = float(np.median(self._calib_buf))
            self._calib_buf = []
            return True
        return False

    @staticmethod
    def polar(xz) -> tuple[float, float]:
        """(reach m, angle deg) of a task-space (x, z) point about WS_CENTER --
        the two quantities the radial mapping is actually built from, and the
        useful way to report how far a target is from a reference pose."""
        d = np.asarray(xz, dtype=float) - WS_CENTER
        return float(np.linalg.norm(d)), float(np.degrees(np.arctan2(d[1], d[0])))

    def map_target(self, shoulder: object, wrist: object) -> tuple[float, float]:
        """Smooths the shoulder->wrist vector and maps arm extension->reach radius,
        hand direction->EE angle. Returns the task-space target (x, z). One call
        per frame -- the one-euro filter must see true camera cadence, not the
        (slower, gated) rate solve() gets called at."""

        # Smoothed relative vector (one-euro).
        rel = self._smooth(np.array([wrist.x - shoulder.x, wrist.y - shoulder.y]))

        # Extension -> reach radius. Fraction of full arm extension over
        # [EXT_MIN_FRAC, 1] maps linearly onto [WS_RMIN, MAP_RMAX].
        ext_frac = np.clip(
            (np.linalg.norm(rel) / self.arm_length - EXT_MIN_FRAC) / (1.0 - EXT_MIN_FRAC),
            0.0, 1.0,)

        ext_frac = np.interp(ext_frac, [0.0, EXT_BREAK_FRAC, 1.0], [0.0, RADIUS_BREAK_FRAC, 1.0])
        radius = WS_RMIN + ext_frac * (MAP_RMAX - WS_RMIN)

        # Hand direction -> EE direction in robot x-z (+x fwd, +z up). MediaPipe y
        # is down, so flip it. Near the shoulder the direction is undefined; hold
        # the last one to avoid snapping.
        direction = np.array([rel[0], -rel[1]])
        n = np.linalg.norm(direction)
        if n > 1e-6:
            self.prev_dir = direction / n
        robot_x, robot_z = WS_CENTER + radius * self.prev_dir
        return float(robot_x), float(robot_z)

    def solve(self, x: float, z: float):
        """Builds T_sd from the task-space target (x, z) plus the fixed
        orientation/held-y, solves IK, advances theta_prev on success. Returns
        (thetalist, success)."""

        T_sd = np.eye(4)
        T_sd[:3, :3] = self.R_fixed
        T_sd[:3, 3] = np.array([x, self.T_rest[1], z])

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

    def step(self, shoulder: object, wrist: object):
        """map_target + solve in one call. Kept for callers (main.py, the offline
        tests) that don't need C4's per-tick ready-pose gate."""
        x, z = self.map_target(shoulder, wrist)
        return self.solve(x, z)

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
