from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("markerless")
@dataclass
class MarkerlessTeleopConfig(TeleoperatorConfig):
    """Config for the markerless (single-webcam pose) teleoperator.

    Registered as choice ``"markerless"`` so lerobot's CLI can select it via
    ``--teleop.type=markerless``; ``config.type`` then returns ``"markerless"``.
    """

    # Webcam device index for the perception thread.
    camera_index: int = 0
    # Frames (median) collected when measuring the operator's arm length.
    n_calib_frames: int = 20
    # Minimum landmark visibility to trust a frame for calibration/driving.
    vis_thresh: float = 0.5
    # One-euro filter params handed to RobotArm (jitter-vs-lag tradeoff).
    min_cutoff: float = 1.0
    beta: float = 1.5
    # Ready-pose gate: teleop stays parked until the mapped target sits within
    # gate_eps metres of PARK's end-effector (x, z) for gate_n_frames
    # CONSECUTIVE frames. Kills the startup task-space lunge ("Jerk B").
    gate_eps: float = 0.035      # m; PLAN suggests 2-3 cm
    gate_n_frames: int = 8       # PLAN suggests 5-10
