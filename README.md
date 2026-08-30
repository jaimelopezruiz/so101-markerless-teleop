# Markerless teleoperation: SO-101

Drive a [LeRobot](https://github.com/huggingface/lerobot) SO-101 5-DOF arm from
a single webcam: MediaPipe pose estimation → a radial operator-to-robot mapping →
from-scratch inverse kinematics → joint commands. No wearables, no markers. See
[DEVLOG.md](DEVLOG.md) for goals, scope, and status.

## Layout

```
so101-markerless-teleop/
├── README.md
├── DEVLOG.md
├── main.py            # standalone orchestrator: camera loop, calibration key, ramps
├── pyproject.toml     # editable install; the distribution name is load-bearing (see below)
├── requirements.txt
├── bridge/
│   └── bridge.py      # RobotArm: calibration, smoothing, radial mapping, IK target, to_action
├── kinematics/
│   ├── core.py        # trimmed Modern Robotics library (PoE FK, Jacobians, SE(3) utils)
│   └── ik.py          # IKinBodyDLS + dls_operator (damped pseudo-inverse, null-space projector)
├── urdf/
│   ├── parser.py      # findMnS, extracts M, Slist, joint limits from the URDF
│   └── so101_new_calib.urdf
├── perception/
│   ├── pose_detector.py     # PoseDetector: the sole MediaPipe home
│   ├── perception_thread.py # background publisher, latest Snapshot, no cv2 GUI
│   ├── image_mapping.py     # still-image demo
│   └── video_mapping.py     # landmark_stream() generator, yields (shoulder, wrist, key)
├── lerobot_teleoperator_markerless/
│   ├── config.py      # MarkerlessTeleopConfig, registers --teleop.type=markerless
│   └── markerless.py  # MarkerlessTeleop, the lerobot Teleoperator plugin
├── docs/media/        # figures and gifs used by the docs
├── models/
│   └── pose_landmarker_heavy.task   # MediaPipe model (not tracked in git)
└── tests/
    ├── robot.py           # shared SO-101 model fixtures (M, Slist, Blist, limits)
    ├── test_fk.py         # FK vs yourdfpy
    ├── test_jacobian.py   # Jacobian vs numerical differentiation
    ├── test_ik.py         # IK round-trip, singularity, convergence radius
    ├── test_bridge.py     # RobotArm: calibration + radial mapping
    ├── teleop_live.py     # hardware: MarkerlessTeleop -> SO101Follower, with gate overlay
    ├── joint_check.py     # hardware: per-joint sign/offset verification
    └── lerobot_tests.py   # hardware: raw observation stream
```

Everything below the orchestrators is free of lerobot and independently testable.

## Setup

```sh
python -m venv .venv
.venv\Scripts\activate          # Windows
pip install -r requirements.txt
pip install -e .
```

The editable install (`pip install -e .`) puts the project on the path, so the
imports resolve no matter where a script is run from. The registered packages
are `kinematics`, `urdf`, `perception`, `bridge`, `lerobot_teleoperator_markerless`,
and `tests`. The install records a **static snapshot** of that list, so re-run
`pip install -e .` whenever it changes in `pyproject.toml` - otherwise the new
package resolves only when a script happens to run from the repo root.

The MediaPipe model (`models/pose_landmarker_heavy.task`) is not tracked in
git; download `pose_landmarker_heavy.task` from MediaPipe and place it there.

### The distribution name is deliberate

`pyproject.toml` sets `name = "lerobot_teleoperator_markerless"`, which looks
wrong for a project this size. It is what makes `--teleop.type=markerless`
resolve: lerobot's `register_third_party_plugins()` scans installed
**distribution** names for a `lerobot_teleoperator_` prefix and imports the
module of that exact name. Renaming the distribution silently breaks the record
path. `lerobot` is pinned to `0.5.1` because this route relies on registration
conventions that are not documented.

### OpenCV GUI conflict (`cv2.imshow` fails)

`lerobot` depends on `opencv-python-headless`; this repo (and mediapipe) need
`opencv-contrib-python`. Both install into the same `cv2/` directory, so
whichever pip writes last wins - and if that's headless, every `cv2.imshow`
raises *"The function is not implemented. Rebuild the library with Windows,
GTK+ 2.x or Cocoa support"*. Any `pip install` that touches lerobot can
re-trigger it. Fix (version pinned to match lerobot's `<4.14` constraint):

```sh
pip install --force-reinstall --no-deps "opencv-contrib-python==4.13.0.92"
```

## Running

Two orchestrators sit on the same core. Both need a webcam and the arm.

**Standalone loop.** Owns its own camera window, calibration keypress, and the
startup/stow ramps, and drives the arm directly:

```sh
python main.py
```

Press `c` with your **right** arm extended forward to calibrate, then move your
arm to drive the solver. Press `q` or close the window to stop.

**lerobot plugin.** Perception runs on a background thread and `lerobot-record`
owns the hardware and the loop rate:

```sh
# standalone harness: get_action() -> send_action(), with the ready-pose overlay
python -m tests.teleop_live

# full record path, writes a LeRobotDataset
lerobot-record --robot.type=so101_follower --robot.port=COM3 --robot.id=<id> \
  --robot.max_relative_target=10.0 \
  --robot.cameras="{ scene: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}}" \
  --teleop.type=markerless --teleop.camera_index=0 \
  --dataset.repo_id=<user>/<name> --dataset.single_task="..." --dataset.push_to_hub=false
```

On the plugin path the arm holds its parked pose until your mapped hand position
reaches it (the ready-pose gate), which is what stops it lunging at handover.
The terminal prints how far off you are and which way to move. Both paths log a
throttled rate line: control rate, new-frame rate, and the measured control
period, which is the evidence that perception and control really are decoupled.

Perception only, no arm needed:

```sh
python -m perception.video_mapping        # camera window with the mapping overlay
python -m perception.perception_thread    # threaded publisher, producer/consumer rates
```

## Tests

Plain assert-scripts, not pytest. Run one by running its module.

```sh
python -m tests.test_fk         # FK vs yourdfpy
python -m tests.test_jacobian   # Jacobian vs numerical differentiation
python -m tests.test_ik         # IK round-trip, singularity, convergence radius
python -m tests.test_bridge     # RobotArm calibration + radial mapping
```

These four need **no camera and no robot**. `tests/teleop_live.py`,
`tests/joint_check.py` and `tests/lerobot_tests.py` are hardware diagnostics and
are not part of the offline suite.

`test_ik` is seeded and asserts a success-rate floor at each noise level. It
takes ~30 s for its full sweep; `--iters 50` is enough while iterating. It also
carries a stricter local gate for refactors:

```sh
python -m tests.test_ik --capture   # save a baseline from known-good code
python -m tests.test_ik --check     # solved angles must come back bit-identical
python -m tests.test_ik --plots     # write figures to docs/media/tests/ (gitignored)
```

## Credits

`kinematics/core.py` is a trimmed subset of the
[Modern Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
code library (Weng, Hunt, Schultz, Todes).

The SO-101 hardware interface (motor calibration, observations, and action
commands) is provided by [LeRobot](https://github.com/huggingface/lerobot)
(Cadene et al., 2024).

```bibtex
@misc{cadene2024lerobot,
    author = {Cadene, Remi and Alibert, Simon and Soare, Alexander and Gallouedec, Quentin and Zouitine, Adil and Palma, Steven and Kooijmans, Pepijn and Aractingi, Michel and Shukor, Mustafa and Aubakirova, Dana and Russi, Martino and Capuano, Francesco and Pascal, Caroline and Choghari, Jade and Moss, Jess and Wolf, Thomas},
    title = {LeRobot: State-of-the-art Machine Learning for Real-World Robotics in Pytorch},
    howpublished = "\url{https://github.com/huggingface/lerobot}",
    year = {2024}
}
```
