# Markerless teleoperation — SO-101

Drive a LeRobot SO-101 5-DOF arm from a single webcam: MediaPipe pose
estimation → frame transforms → from-scratch inverse kinematics → joint
commands. No wearables, no markers. See [DEVLOG.md](DEVLOG.md) for goals,
scope, and status.

## Layout

```
modern_robotics/
├── README.md
├── DEVLOG.md
├── requirements.txt
├── kinematics/
│   ├── core.py        # trimmed Modern Robotics library (PoE FK, Jacobians, SE(3) utils)
│   └── ik.py          # IKinBodyDLS — damped least-squares inverse kinematics
├── urdf/
│   ├── parser.py      # findMnS — extracts M, Slist, joint limits from the URDF
│   └── so101_new_calib.urdf
├── perception/
│   ├── pose_detector.py   # MediaPipe landmark drawing/extraction helper
│   ├── image_mapping.py   # still-image demo
│   └── video_mapping.py   # live-webcam demo
├── models/
│   └── pose_landmarker_heavy.task   # MediaPipe model (not tracked in git)
└── tests/
    ├── robot.py           # shared SO-101 model fixtures (M, Slist, Blist, limits)
    ├── test_fk.py         # FK vs yourdfpy
    ├── test_jacobian.py   # Jacobian vs numerical differentiation
    └── test_ik.py         # IK round-trip, singularity, noise sweep
```

## Setup

```sh
python -m venv .venv
.venv\Scripts\activate          # Windows
pip install -r requirements.txt
```

The MediaPipe model (`models/pose_landmarker_heavy.task`) is not tracked in
git — download `pose_landmarker_heavy.task` from MediaPipe and place it there.

## Running

All scripts use absolute imports and expect to be run as modules from the
repo root:

```sh
python -m tests.test_fk
python -m tests.test_jacobian
python -m tests.test_ik
python -m perception.video_mapping
```

`kinematics/core.py` is a trimmed subset of the
[Modern Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
code library (Weng, Hunt, Schultz, Todes).
