# Markerless teleoperation: SO-101

Drive a LeRobot SO-101 5-DOF arm from a single webcam: MediaPipe pose
estimation → frame transforms → from-scratch inverse kinematics → joint
commands. No wearables, no markers. See [DEVLOG.md](DEVLOG.md) for goals,
scope, and status.

## Layout

```
so101-markerless-teleop/
├── README.md
├── DEVLOG.md
├── main.py            # entry point: wires perception, bridge, and hardware
├── pyproject.toml
├── requirements.txt
├── bridge/
│   └── bridge.py      # Robot class: calibration, smoothing, IK target construction
├── kinematics/
│   ├── core.py        # trimmed Modern Robotics library (PoE FK, Jacobians, SE(3) utils)
│   └── ik.py          # IKinBodyDLS, damped least-squares inverse kinematics
├── urdf/
│   ├── parser.py      # findMnS, extracts M, Slist, joint limits from the URDF
│   └── so101_new_calib.urdf
├── perception/
│   ├── pose_detector.py   # MediaPipe landmark drawing/extraction helper
│   ├── image_mapping.py   # still-image demo
│   └── video_mapping.py   # landmark_stream() generator, yields (shoulder, wrist, key)
├── models/
│   └── pose_landmarker_heavy.task   # MediaPipe model (not tracked in git)
└── tests/
    ├── robot.py           # shared SO-101 model fixtures (M, Slist, Blist, limits)
    ├── test_fk.py         # FK vs yourdfpy
    ├── test_jacobian.py   # Jacobian vs numerical differentiation
    ├── test_ik.py         # IK round-trip, singularity, noise sweep
    └── test_bridge.py     # Robot class: calibration, step, IK gate
```

## Setup

```sh
python -m venv .venv
.venv\Scripts\activate          # Windows
pip install -r requirements.txt
pip install -e .                # makes kinematics/, urdf/, perception/, tests/ importable
```

The editable install (`pip install -e .`) puts the project on the path, so the
imports resolve no matter where a script is run from. The registered packages
are `kinematics`, `urdf`, `perception`, `bridge`, and `tests`.

The MediaPipe model (`models/pose_landmarker_heavy.task`) is not tracked in
git; download `pose_landmarker_heavy.task` from MediaPipe and place it there.

## Running

Run the live pipeline from the repo root:

```sh
python main.py
```

Press `c` with your arm extended forward to calibrate, then move your arm to
drive the solver. Press `q` or close the window to stop.

To run the verification tests:

```sh
python -m tests.test_fk
python -m tests.test_jacobian
python -m tests.test_ik
python -m tests.test_bridge
```

`kinematics/core.py` is a trimmed subset of the
[Modern Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
code library (Weng, Hunt, Schultz, Todes).

@misc{cadene2024lerobot,
    author = {Cadene, Remi and Alibert, Simon and Soare, Alexander and Gallouedec, Quentin and Zouitine, Adil and Palma, Steven and Kooijmans, Pepijn and Aractingi, Michel and Shukor, Mustafa and Aubakirova, Dana and Russi, Martino and Capuano, Francesco and Pascal, Caroline and Choghari, Jade and Moss, Jess and Wolf, Thomas},
    title = {LeRobot: State-of-the-art Machine Learning for Real-World Robotics in Pytorch},
    howpublished = "\url{https://github.com/huggingface/lerobot}",
    year = {2024}
}