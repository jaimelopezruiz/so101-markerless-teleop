# DEVLOG

Development log for markerless teleoperation project using LeRobot's SO-101 arm.

> A note on ordering: this is a journal, but the foundation layer (Entries 1-6) is
> written up after the fact. Perception, URDF parsing, FK, Jacobians, and IK were
> each built and verified on their own before being documented, so these entries
> follow the build order (each depends on the previous one) rather than diary
> chronology. Day-by-day journaling resumes at the bridge layer (Entry 7+), where
> design and writing happen together. Might make a cleaner reference doc once finished.

> Planned future work includes adding perception of depth for the missing axis of movement and extending the current purely kinematic control to model the arm's dynamics (joint torques, inertia, and gravity compensation).
----

## Contents
- [Entry 1: Project Setup & Goals](#entry-1---setup-and-goals)
- [Entry 2: Perception Layer: MediaPipe Pose Tracking](#entry-2---perception-layer-mediapipe-pose-tracking)
- [Entry 3: URDF Parsing & Kinematic Description](#entry-3---urdf-parsing--kinematic-description)
  - [Frames & Conventions (reference)](#frames--conventions-reference)
- [Entry 4: Forward Kinematics](#entry-4---forward-kinematics)
- [Entry 5: Jacobians](#entry-5---jacobians)
- [Entry 6: Inverse Kinematics](#entry-6---inverse-kinematics)
- [Entry 7: Bridge Layer (Perception → IK)](#entry-7---bridge-layer-perception--ik)
- [Entry 8: Video Loop Integration](#entry-8---video-loop-integration)
- [Entry 9: Hardware Bring-up](#entry-9---hardware-bring-up)
- [Entry 10: LeRobot Record Integration](#entry-10---lerobot-record-integration)

----

## Entry 1 - Setup and goals

Lay the foundation and validate perception, kinematics, and IK independently.

### Goal

Build a markerless teleoperation pipeline for the SO-101 5-DOF robot arm. The user's arm motion is captured by a standard webcam, processed via pose estimation, and translated into joint commands that drive the physical robot in real time.
No wearable trackers, no IMUs, no fiducial markers.

### Motivation

Most teleoperation systems rely on expensive motion-capture hardware or wearable trackers.
This project explores how far I can get with a single webcam and from-scratch kinematics, which is useful for low-cost teleop and demonstrations.
It also serves as a learning exercise in robotics fundamentals: kinematics, IK, frame transformations, and real-time control.

### Tech stack

| Component | Tool |
|-----------|------|
| Perception | MediaPipe Pose Landmarker (`pose_world_landmarks`) |
| Kinematics | Custom Python implementation (Product-of-Exponentials formulation) |
| Robot description | URDF (parsed with custom script) |
| Hardware interface | `lerobot` library for SO-101 |
| Visualization / verification | `yourdfpy`, `matplotlib` |
| Language | Python 3.10 |

### Pipeline architecture

```mermaid
flowchart LR
    A[Webcam] --> B[MediaPipe Pose]
    B --> C[Landmark extraction<br/>shoulder, wrist]
    C --> D[One-euro smoothing<br/>+ radial mapping]
    D --> E[Construct T_sd]
    E --> F[IK solver<br/>DLS + Newton-Raphson]
    F --> G[Joint angles θ]
    G --> H[SO-101 via lerobot]
    G -. "warm start: previous θ seeds next frame's solve" .-> F
```

The dotted feedback edge is the warm start: the iterative IK solver needs an initial
guess, so each frame seeds the solve with the previous frame's solution. This is
faster (the target moves little between frames) and keeps the joint trajectory
smooth over time. It is enabled by the solver's `thetalist0` argument and driven by
the bridge layer (`RobotArm.solve`), where `theta_prev` advances only on IK success,
so a failed or low-visibility frame holds the last good pose.

Two orchestrators sit on top of this core, and everything below them is free of
lerobot. `main.py` owns its own camera loop, the calibration keypress and the
startup/stow ramps, and drives the arm directly. `MarkerlessTeleop` exposes the same
core as a lerobot `Teleoperator` plugin, so `lerobot-record` calls `get_action()` at
record fps and owns the hardware itself; perception then runs on its own thread
(`PerceptionThread`) rather than in the orchestrator's loop.

### Repo structure

```text
so101-markerless-teleop/
├── README.md            # layout, setup, how to run
├── DEVLOG.md            # this file
├── main.py              # standalone orchestrator: camera loop, calibration key, ramps
├── pyproject.toml       # editable install; distribution name IS the lerobot plugin name
├── requirements.txt     # numpy, matplotlib, mediapipe, opencv-contrib-python, yourdfpy, lerobot
├── .gitignore
├── kinematics/
│   ├── core.py          # trimmed Modern Robotics lib, 18 functions (was 40+)
│   └── ik.py            # IKinBodyDLS
├── urdf/
│   ├── parser.py        # findMnS + rpyToRot (path-safe, no module globals)
│   └── so101_new_calib.urdf
├── perception/
│   ├── pose_detector.py     # PoseDetector + draw_landmarks_on_image, sole MediaPipe home
│   ├── perception_thread.py # background publisher, latest Snapshot, no cv2 GUI
│   ├── image_mapping.py     # still-image demo
│   └── video_mapping.py     # webcam demo, landmark_stream generator
├── bridge/
│   └── bridge.py        # RobotArm: smoothing, radial mapping, T_sd, IK call, to_action
├── lerobot_teleoperator_markerless/
│   ├── config.py        # MarkerlessTeleopConfig, registers --teleop.type=markerless
│   └── markerless.py    # MarkerlessTeleop, the lerobot Teleoperator plugin
├── docs/
│   └── media/           # gifs and figures used by README and this log
├── models/
│   └── pose_landmarker_heavy.task   # 30 MB, git-ignored
└── tests/
    ├── robot.py         # shared M, Slist, Blist, limits fixtures
    ├── test_fk.py       # verify_fk + driver
    ├── test_jacobian.py # verify_jac + driver
    ├── test_ik.py       # round_trip, verify_singular, noise_sweep + driver
    ├── test_bridge.py   # calibration + radial mapping, no camera or hardware
    ├── teleop_live.py   # hardware: get_action -> send_action with gate overlay
    ├── joint_check.py   # hardware: per-joint sign/offset verification
    └── lerobot_tests.py # hardware: raw observation stream
```

`test_fk`, `test_jacobian`, `test_ik` and `test_bridge` need no camera or robot.
The other three are hardware diagnostics, run with the arm connected.

### Scope and constraints (v1)

- 2D first: target the end-effector position in the (x, y) plane of the robot base; z held constant.
- Fixed orientation: gripper orientation locked to a sensible constant pose, not tracked from the user.
- Single webcam: monocular depth limitations accepted; stereo extension deferred.
- Position-only target: orientation tracking and finger/gripper control deferred to later phases.

### Current status

| Module | State | Verified against |
|--------|-------|------------------|
| URDF parser | Working | Manual inspection, FK consistency |
| FK (space & body) | Working | `yourdfpy` cross-validation, N random configs |
| Jacobians (space & body) | Working | Numerical differentiation |
| IK (DLS, body frame) | Working | Round-trip, convergence radius, unreachable, singularity tests |
| Pose tracker | Working | Live webcam, returns world landmarks |
| Perception thread (background publisher) | Working | Demo loop, producer vs consumer rates |
| Bridge layer (perception to IK input) | Working | `tests/test_bridge.py`, live teleop |
| Hardware execution | Working | `main.py` and `tests/teleop_live.py` on the arm |
| lerobot Teleoperator plugin | Working | Recorded dataset reloaded via `LeRobotDataset`, one episode replayed |

----

## Entry 2 - Perception Layer: MediaPipe Pose Tracking

### Goal
Turn a webcam feed into a stream of 3D arm landmarks usable as an IK target, with no markers or wearables.

### Approach
MediaPipe Pose Landmarker (heavy model, VIDEO running mode) runs on each frame. It returns two landmark sets, and choosing between them is the main decision here (see Decisions below).

### Implementation notes
- `perception/pose_detector.py::draw_landmarks_on_image` overlays the skeleton for visual debugging and returns the `pose_world_landmarks`.
- `perception/image_mapping.py`: single-image smoke test, run on a still photo.
- `perception/video_mapping.py`: live webcam loop in VIDEO mode. It now raises immediately if no camera is available instead of spinning on empty frames.
- The right arm is used: landmark 12 is the shoulder, 16 is the wrist. The shoulder is the anchor, so all target coordinates are expressed relative to it and the mapping does not depend on where the user stands in frame.

### Verification
There's no unit test here. I check perception visually, since the ground truth is whether the skeleton tracks my arm. Two manual checks:
- `image_mapping.py` on a still photo: the annotated skeleton overlay lands on the right joints.
- `video_mapping.py` live: landmarks track the arm in real time, and the shoulder/wrist indices are the expected joints.

Example output from the live tracking script (`video_mapping.py`), saved to `docs/media/`:

![Live pose tracking](docs/media/live_tracking.gif)

Regenerate with:

```sh
python -m perception.video_mapping   # press q to stop; saves docs/media/live_tracking.gif
```

### Decisions & open questions
- Using `pose_world_landmarks`, not `pose_landmarks`. The latter are normalized image coordinates (0 to 1, image-relative), with no metric scale and tied to the camera framing. The world landmarks are in metres, body-relative (origin near the hips), which is what the kinematics need.
- Anchoring on the shoulder makes the signal translation-invariant, so the user can move around the frame.
- Monocular depth is unreliable: a single camera cannot recover absolute depth well. This is the reason for the 2D-first scope (Entry 1): target the base (x, y) plane and hold z constant. Depth via stereo or learned priors is deferred.
- Open: temporal smoothing / jitter filtering on the landmark stream is not implemented yet, and will likely be needed before driving hardware.

----

## Entry 3 - URDF Parsing & Kinematic Description

### Goal
Convert the SO-101 URDF into the Product-of-Exponentials (PoE) quantities the kinematics need: the home pose `M`, the space-frame screw axes `Slist`, and the joint `limits`.

### Approach: why PoE over DH
Denavit-Hartenberg requires per-joint frame assignments with fiddly conventions and is error-prone to derive by hand. PoE needs only, for each joint, a screw axis expressed in the base frame at the home configuration, plus one home transform `M`. It maps directly onto URDF data (axis and origin per joint) and avoids the intermediate frame bookkeeping. Implemented in `urdf/parser.py::findMnS`.

### Implementation notes
- Building `M`: walk the joints and chain their local link transforms (`T = T @ T_local`), where each `T_local` comes from the joint `origin` (`xyz` translation and `rpy` rotation). After the full chain, `T` is the home pose of the end-effector, `M`.
- Building `Slist`: for each revolute joint, the screw axis at home is

```math
\mathcal{S}_i = \begin{bmatrix} \omega_i \\ v_i \end{bmatrix}, \qquad \omega_i = R_{\text{cum}}\,\hat{a}_i, \qquad v_i = -\,\omega_i \times q_i
```

  where `a_i` is the joint's local axis, `R_cum` is the rotation of the chain up to that joint, and `q_i` is a point on the axis (the joint origin in the base frame, `T[:3,3]`).
- The subtle part: ω and q must both be in the base frame, not parent-relative. The local axis is rotated into the base frame by `R_cum`, and q is read from the accumulated `T`, not from the per-joint origin. Getting either one parent-relative produces a wrong-but-plausible model that only the FK cross-validation in Entry 4 catches.
- Joint walk: joints are traversed in reversed URDF order, arm joints only (the `gripper` joint is skipped). Joint limits are read from each `<limit>` tag into `limits` as `[lower, upper]` rows.
- `rpyToRot` convention: fixed-axis roll-pitch-yaw, composed as `R = Rz(y) · Ry(p) · Rx(r)`, matching the URDF/ROS RPY convention.

### Verification
There's no standalone parser test. I verify correctness transitively: if `M` or `Slist` were wrong, FK would not match the independent `yourdfpy` model (Entry 4). So a passing FK test also validates the parser.

### Decisions & open questions
- The gripper joint is excluded on purpose; v1 is position-only with 5 arm DOF.
- `findMnS` is path-safe (it resolves the bundled URDF relative to the source file) and holds no module-level globals.
- Open: the reversed-joint-walk assumes a specific URDF ordering. A more robust parser would build the kinematic tree explicitly rather than relying on document order.

### Frames & Conventions (reference)
The conventions every later entry depends on, in one place.

| Symbol | Meaning |
|--------|---------|
| `{s}` | Space/base frame (URDF `base_link`) |
| `{b}` | Body frame, the end-effector (URDF `gripper_frame_link`) |
| `M` | Home configuration of `{b}` relative to `{s}` (4×4) |
| `Slist` | Space-frame screw axes at home, columns (6×n), from `findMnS` |
| `Blist` | Body-frame screw axes at home, columns (6×n), derived in `tests/robot.py` |
| `T_sd` | Desired end-effector config (the IK target) in `{s}` |
| `T_sb` | Current end-effector config in `{s}` |

- Space-to-body conversion: `Blist = Ad(M⁻¹) · Slist` (`tests/robot.py` builds `Blist` from `Slist` this way).
- FK comes in two equivalent forms: the space form uses `(M, Slist)`, the body form uses `(M, Blist)` (Entry 4).
- IK runs in the body frame (it uses `Blist`), with the error measured as the body twist `log(T_sb⁻¹ · T_sd)` (Entry 6).
- Units: metres and radians. Joint order (matching the `Slist` columns): `[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]`.

----

## Entry 4 - Forward Kinematics

### Goal
Map joint angles θ to the end-effector pose `T ∈ SE(3)`, in both space and body PoE forms, and check it against a reference.

### Approach
Two equivalent Product-of-Exponentials formulations (`kinematics/core.py`). `FKinSpace` composes the joint exponentials in the space frame:

```math
T(\theta) = e^{[\mathcal{S}_1]\theta_1}\,e^{[\mathcal{S}_2]\theta_2}\cdots e^{[\mathcal{S}_n]\theta_n}\,M
```

`FKinBody` composes them in the body frame:

```math
T(\theta) = M\,e^{[\mathcal{B}_1]\theta_1}\,e^{[\mathcal{B}_2]\theta_2}\cdots e^{[\mathcal{B}_n]\theta_n}
```

The two give the same `T` for a given θ because `Blist` is defined as `Ad(M⁻¹) · Slist`. So testing `FKinSpace` against `FKinBody` compares two independent code paths rather than restating one.

### Verification
`tests/test_fk.py` cross-validates against an independent library. For each configuration it computes the pose two ways: my `FKinSpace(M, Slist, θ)`, and `yourdfpy`'s `URDF.get_transform("gripper_frame_link", "base_link")` after setting the same joint config. It reports the max absolute element-wise difference of the two 4×4 matrices, and passes if that is below `1e-4`. This catches any error in the parser (`M`, `Slist`) or in the exponential maps.

```
PASS  max_err=2.22e-16  theta=[0 0 0 0 0]
PASS  max_err=1.11e-16  theta=[-0.393  0.     0.     0.     0.   ]
PASS  max_err=1.53e-16  theta=[0.    0.785 0.    0.    0.   ]
PASS  max_err=3.33e-16  theta=[ 0.     0.    -0.785  0.     0.   ]
PASS  max_err=4.44e-16  theta=[ 0.393 -0.785  0.524 -0.393  1.047]
```

Errors are at the level of floating-point round-off (~1e-16): the two models are identical to machine precision across configs that span all five joints.

### Decisions & open questions
- Both forms are kept: the space form pairs with the space Jacobian (Entry 5), and the body form is what IK iterates on (Entry 6).
- `yourdfpy` is the reference because it is an independent implementation. Agreement to 1e-16 is strong evidence both are correct, rather than sharing the same bug.

----

## Entry 5 - Jacobians

### Goal
Provide the manipulator Jacobian J(θ) that relates joint rates to the end-effector twist: the linearization the IK solver steps along.

### Approach
`JacobianSpace(Slist, θ)` and `JacobianBody(Blist, θ)` (`kinematics/core.py`), where each column is the screw axis transformed by the accumulated exponentials up to that joint.

### Verification
`tests/test_jacobian.py` checks the analytical Jacobian against numerical differentiation. For each joint it perturbs that joint's angle by a small step (`1e-4`), recomputes FK, and forms the spatial twist numerically from the finite difference of `T`, reading ω from the skew-symmetric part and v from the translation. It compares this finite-difference Jacobian to `JacobianSpace`, and passes if the max error is below `1e-4`.

```
PASS  max_err=1.17e-05  theta=[0. 0. 0. 0. 0.]
PASS  max_err=1.77e-05  theta=[-0.393  0.     0.     0.     0.   ]
PASS  max_err=2.50e-05  theta=[0.    0.785 0.    0.    0.   ]
PASS  max_err=2.50e-05  theta=[ 0.     0.    -0.785  0.     0.   ]
PASS  max_err=2.23e-05  theta=[ 0.393 -0.785  0.524 -0.393  1.047]
```

Errors sit at ~1e-5, limited by the finite-difference step rather than the analytical Jacobian (FK itself is exact to 1e-16 from Entry 4), which is what a first-order difference should give.

### Decisions & open questions
- Condition number motivates the IK design. Near singular configurations J becomes ill-conditioned and a naive inverse step blows up. Measuring cond(J) here is what motivates the damped least-squares step in Entry 6: the damping term λ²I is what keeps `J·Jᵀ + λ²I` invertible when J loses rank. Entry 5 measures the problem; Entry 6 is the fix.
- Open: a richer singularity study (sweeping configs and plotting cond(J) over the workspace) would better show where damping matters most.

----

## Entry 6 - Inverse Kinematics

### Goal
Given a desired pose `T_sd`, solve for joint angles θ that reach it, robustly and within joint limits: the core of the teleop loop.

### Approach: Newton-Raphson with damped least-squares
Iterate in the body frame. At each step, compute the body-twist error to the target and take a damped least-squares step:

```math
\mathcal{V}_b = \log\!\left(T_{sb}^{-1}\,T_{sd}\right), \qquad \Delta\theta = J_b^{\mathsf{T}}\left(J_b J_b^{\mathsf{T}} + \lambda^2 I\right)^{-1}\mathcal{V}_b
```

Implemented in `kinematics/ik.py::IKinBodyDLS`. It iterates until both the angular (`eomg`) and linear (`ev`) tolerances are met, or until `maxiters` is hit.

### Decisions & open questions
- DLS over plain pseudoinverse. The damping λ²I regularizes the step near singularities where `J_b` loses rank (the Entry 5 motivation). One caveat from testing: the bundled `verify_singular` config (`shoulder_lift = π/2`) is only mildly ill-conditioned (cond ≈ 30), so DLS and pinv behave identically on it, both converging in 2 iterations. The robustness payoff shows up in the noise sweep below, and the DLS advantage would widen at genuinely rank-deficient configs.

  ```
  Jacobian condition number at config: 29.6
  DLS  (lam=0.05): converged in 2 iters, max|theta|=1.67
  Pinv           : 2 iters,            max|theta|=1.67
  ```

- In-loop joint clamping, decided empirically. Joint limits are re-clamped every iteration, not just on the final result. An earlier docstring claimed in-loop clamping "corrupts the gradient and prevents convergence." The measurements show the opposite. Re-running the round-trip success sweep both ways (200 trials per noise level, identical seeds):

  ```
    noise |  in-loop clamp |  end-only clamp
  ------------------------------------------
     0.00 |         100.0% |          100.0%
     0.01 |         100.0% |          100.0%
     0.10 |         100.0% |          100.0%
     0.50 |          98.5% |           98.0%
     0.75 |          96.5% |           95.0%
     1.00 |          95.0% |           86.0%
     1.25 |          90.0% |           70.0%
     1.50 |          85.5% |           59.0%
     2.00 |          83.0% |           35.0%
     5.00 |          72.5% |            1.0%
  ```

  The two are equivalent for a small initial-guess error but diverge sharply as the guess worsens: at ~2 rad off, 83% vs 35%; at 5 rad, 72.5% vs 1%. Keeping each iterate inside the reachable joint space stops the solver wandering into unreachable regions it cannot recover from. Decision: clamp in-loop (docstring corrected to match).
- NaN/Inf guard. If a step produces non-finite values (a degenerate Jacobian solve), the solver bails and returns the last good θ with `success=False`, rather than propagating NaNs into joint commands.
- The solver itself doesn't implement warm starting; `IKinBodyDLS` just exposes an initial-guess argument (`thetalist0`). The warm start happens in the bridge layer (Entry 7+), which will seed each frame's solve with the previous frame's solution. I note it here only as the affordance the signature provides; the round-trip test already exercises it by perturbing the true θ as the seed.

### Verification
`tests/test_ik.py` has three checks:
- `round_trip`, the core correctness test: sample a random valid θ, FK it to a target `T_sd`, perturb θ by noise to make the initial guess, run IK, and confirm the solution reproduces the target pose within tolerance. Non-converged solves count as failures. Reported as a success rate.
- `verify_singular`: steps DLS against a plain pseudoinverse near a near-singular config and plots the joint trajectories (it prints the condition number; see the caveat above).
- `noise_sweep`: round-trip success rate across a range of initial-guess noise levels, which characterizes the convergence radius (the in-loop-clamp table above is this sweep, run for both clamping modes).

Default driver output (round-trip, noise = 1 rad, 100 trials; stochastic, no fixed seed):

```
round-trip success (noise=1): 0.90
```

**Update (before the velocity-IK refactor).** Both problems were in the test, not the solver. The pass gate had drifted out of sync: `bb65ea1` loosened the solver default from `ev=1e-3` to `5e-3` and left `round_trip` checking against `1e-3`, so the test scored the solver against a tolerance five times tighter than the one it was handed. At noise = 1 rad, 44.6% of converged solves land in `[1e-3, 5e-3)` and only 1.6% genuinely miss. That accounts for the whole gap between the 95.0% in the table above and the ~52% the driver was reporting by the time I came back to it. The gate now reuses whatever `eomg`/`ev` go to the solver, the sweep reproduces the table, and the in-loop clamping decision stands unchanged.

The test was also unseeded, wandering 0.43-0.53 across four runs of untouched code, which makes it worthless as an oracle for a refactor. It now seeds `default_rng` per noise level and asserts a floor at each, and `--capture`/`--check` compares the solved angles bit-for-bit against a saved baseline. The bit-exact comparison is the part that matters: swapping `Jt.T @ inv(A)` for the algebraically identical `solve(A, Jt).T` leaves every success rate passing but moves solutions by up to 5.6 rad, because a 1e-15 rounding difference in the pseudo-inverse iterates into a different basin. Since `theta_prev` is both the warm-start seed and the hold-last pose, a different solution means the arm holds a different posture.

### Open questions
- Tune `λ`, `eomg`, and `ev` against the real control loop's rate and noise once hardware is connected.
- Add an explicit reachability/limit pre-check so unreachable teleop targets are reported, rather than silently clamped to the nearest feasible pose.

----

## Entry 7 - Bridge Layer: Perception → IK

### Goal
Connect the perception output (MediaPipe shoulder and wrist world landmarks) to the IK solver input (a 4×4 target pose `T_sd`). This means: anchor the wrist vector on the shoulder, smooth it, map it into robot base-frame coordinates, build `T_sd`, and call `IKinBodyDLS` with a warm start from the previous frame.

### Approach
I worked through six design decisions before writing any code. Each is recorded here with its rationale, because the choices interact and the wrong combination produces motion that is subtly wrong in ways that are hard to diagnose (axis reflections, scale mismatch, IK divergence at rest).

**1. Frame alignment.**
MediaPipe world landmarks use x-right, y-down, z-toward-camera (confirmed experimentally: y is negative above the hip and becomes more negative as the wrist rises). The robot base frame has x-forward, z-up (established from `shoulder_pan` rotating around the vertical axis and the home-config translation `M[:3,3] ≈ [0.39, 0, 0.23]`). With the operator and robot both facing camera-right, the mapping is:

| MediaPipe | Robot base |
|---|---|
| `+x` (right) | `+x` (forward) |
| `−y` (up) | `+z` (up) |
| `+z` (depth, deferred) | `+y` (deferred) |

**2. Calibration and scale.**
A single scalar `scale = REACH / arm_length` maps the MediaPipe-space relative vector to robot-space displacement. `REACH` is the x-z Euclidean distance from the rest position to a FK-computed fully-extended configuration (`[0, 1.7, −1.69, 0, 0]` → ≈ [0.47, 0, 0.07] m), stored as a module constant ≈ 0.57 m. `arm_length` is measured once per session: the operator holds their arm fully extended forward (MediaPipe `+x`), and the x-y norm of the wrist-minus-shoulder vector is recorded. I considered normalising by torso length (continuous, no explicit calibration pose) but rejected it: it requires a hardcoded population-average arm/torso ratio and fallback logic for hip occlusion, adding two new failure modes for a problem that MediaPipe's metric world coordinates already largely handle.

**3. Rest position and Cartesian offset.**
The operator's wrist at their shoulder (zero relative displacement) maps to a natural upright robot pose. I chose `THETALIST_REST = [0, −1.3, 0, 0, 0]` through FK exploration: `shoulder_lift = -1.3` rad places the end-effector at T_rest ≈ [0.05, 0, 0.46] m with the arm pointing upward and ≈ 0.44 rad of margin from the joint limit on both sides (I rejected the first candidate, `shoulder_lift = −1.7`, for being within 0.05 rad of the lower limit). The rest position is the origin of robot motion:

```
robot_x = T_rest[0] + scale * rel_x
robot_z = T_rest[2] − scale * rel_y
```

**4. Fixed orientation.**
Gripper orientation is locked to the end-effector rotation at the rest configuration: `R_fixed = FKinBody(M, Blist, THETALIST_REST)[:3, :3]`. I tried `M[:3, :3]` (the home-config orientation, all joints zero) first; it caused IK to fail silently at the rest target because the rest position and home orientation are not co-reachable, so the solver could not converge. The rest-config orientation is always co-reachable with T_rest by construction. Full orientation tracking is deferred.

**5. IK failure handling.**
When `IKinBodyDLS` returns `success=False`, `theta_prev` is not updated. The caller receives the unconverged joint array and `False`; the hardware layer re-sends the last valid angles, holding position until the target re-enters the workspace. This falls out naturally from the warm-start design: `theta_prev` only advances on convergence.

**6. Smoothing.**
An exponential moving average (`alpha=0.5`) filters the raw 2D relative vector before mapping. The filter state `prev_filtered` is initialised to the first observation on frame zero (`if prev_filtered is None`, not `if not prev_filtered` — the latter raises `ValueError` on numpy arrays). A one-euro filter (adaptive alpha: heavy smoothing at low velocity, light at high) would better handle the jitter-vs-lag tradeoff for interactive control, but I'd need a live pipeline to tune the two extra parameters; I'll revisit it once that's in place.

### Implementation notes
`main.py` holds a `Robot` class:
- `__init__(M, Blist, limits, theta_prev, alpha)`: stores the robot model; derives `T_rest` and `R_fixed` from `FKinBody(M, Blist, theta_prev)` so the caller only needs to pass joint angles, not a pre-computed Cartesian pose.
- `_smooth(raw)`: EMA on the 2D relative vector `[rel_x, rel_y]`.
- `calibrate(shoulder, wrist)`: measures arm length from x-y components only (depth excluded), sets `self.scale`.
- `step(shoulder, wrist)`: computes relative vector → smooth → map → build `T_sd` → IK → update `theta_prev` on success → return `(thetalist, success)`.

Module-level constants at load time: `THETALIST_REST`; `REACH` computed from FK at the fully-extended configuration.

### Verification
`tests/test_bridge.py` uses `SimpleNamespace` to fake MediaPipe landmark objects (no camera or robot needed) and checks:
- After calibrating with a 0.7 m simulated arm extension, `scale ≈ REACH / 0.7` within `1e-4`.
- `step` with wrist at shoulder (zero relative displacement) returns `success=True` and `thetalist` within 0.1 rad of `THETALIST_REST`.
- `step` with a mid-range wrist position returns a 5-element array and a bool without crashing.

### Open questions
- **One-euro filter.** Drop-in upgrade to `_smooth` once the live loop provides tuning data.
- **Depth axis.** MediaPipe z → robot y held fixed at `T_rest[1]`. Stereo or learned-depth extension would unlock 3D teleop.
- **Wrist orientation tracking.** `R_fixed` locks to the rest orientation. Mapping wrist roll to `wrist_roll` is a natural v2 extension.
- **Wiring into the video loop.** `video_mapping.py` needs a calibration trigger (`c` keypress) and a per-frame `step()` call. Entry 8.
- **Hardware execution.** Joint angles from `step()` need to drive the physical SO-101 via lerobot.

----

## Entry 8 - Video Loop Integration

### Goal
Bring all pipeline modules together in one runnable entry point. This is the last software step before hardware: `main.py` connects MediaPipe landmark output, the `Robot` bridge class, and the IK solver into a live loop that computes joint angles from webcam input in real time.

### Approach
I worked through three design decisions before writing any code.

**1. Generator vs. passing the robot into the video loop.**
The original `video_mapping.py` was a standalone script: all setup, detection, and display in one blocking `while` loop. Passing the `Robot` instance in and calling `robot.step()` inside that loop would have coupled perception to the bridge layer, making the two modules impossible to test or reuse independently. Instead, `video_mapping.py` was refactored into a generator `landmark_stream()`. A generator solves the blocking problem cleanly: `yield` suspends the loop at each frame and returns control to `main.py`, which processes the result and requests the next frame. The camera opens once before the loop and stays open for the session.

**2. What the generator yields.**
The generator yields `(shoulder, wrist, key)` per frame, where `key = cv2.waitKey(1) & 0xFF`. Keypresses are detected inside the generator (that is where `cv2.imshow` and `cv2.waitKey` live), but the decision of what to do with a keypress belongs to the orchestrator. Yielding `key` keeps that decision in `main.py` without requiring the perception module to know about calibration or the robot. `'q'` is the one exception handled inside the generator (it breaks the loop), since quitting the window is a perception concern, not a control one.

**3. Calibration gate.**
`robot.scale` is `None` until `robot.calibrate()` is called. `main.py` checks `robot.scale is not None` before calling `robot.step()`, skipping frames with `continue` until the user calibrates. This is `main.py`'s responsibility, not `Robot`'s: the `Robot` class is a data and logic object that should not know about the video loop or user interaction.

### Implementation notes
Before this entry I moved `Robot` and its module-level constants (`M`, `Blist`, `THETALIST_REST`, `REACH`) out of `main.py` and into `bridge/bridge.py`. `main.py` was always meant to be the orchestrator, not a module definition file.

`perception/video_mapping.py` refactored into `landmark_stream()`:
- Setup (once): `PoseLandmarkerOptions` (VIDEO mode, segmentation masks), `PoseLandmarker.create_from_options`, `cv2.VideoCapture(0)`, and `start_time = time.time()` are all initialised before the loop. Previously these were module-level side effects; moving them inside the function means importing the module no longer opens the camera.
- Per-frame: read frame, RGB convert, MediaPipe detect, annotate and display, then `key = cv2.waitKey(1) & 0xFF`. `waitKey` is called once per frame, after `imshow`, and the result stored; all key checks use the stored value.
- Yield and break: if landmarks are detected, `yield shoulder, wrist, key`. Break on `key == ord('q')` or window close. Camera release and `cv2.destroyAllWindows()` run in a `finally` block.

`main.py` wires it together:
- Robot constructed as `bridge.Robot(bridge.M, bridge.Blist, bridge.limits, bridge.THETALIST_REST, alpha=0.5)`, importing constants directly from `bridge.bridge` to avoid duplication.
- Loop: `for shoulder, wrist, key in video_mapping.landmark_stream()`, branching on `key == ord('c')` to calibrate, `robot.scale is None` to skip, or `robot.step(shoulder, wrist)` otherwise.
- Wrapped in `run()` and guarded by `if __name__ == "__main__": run()` so the module is importable without side effects.

### Verification
I ran `python main.py`, pressed `c` with my arm extended forward to calibrate, then moved my arm freely. To sanity-check the output I fed the joint angles from `robot.step()` into `FKinSpace` to recover the end-effector pose. With my wrist held approximately at my shoulder (near-zero relative displacement), the FK result was:

```
T ≈ [[0.682, -0.033,  0.730, 0.091],
     [0.049,  0.999,  0.000, 0.000],
     [-0.730, 0.036,  0.683, 0.362],
     [0,      0,      0,     1    ]]
```

Translation `[0.091, ~0, 0.362]` m versus `T_rest = [0.050, ~0, 0.456]` m. The small offset is consistent with the wrist not being perfectly at the shoulder and the EMA filter not having fully settled from the calibration pose. The y coordinate sits at machine-precision zero throughout, confirming depth is correctly held fixed. The pipeline runs without errors end-to-end.

### Open questions
- **Hardware execution.** Joint angles from `robot.step()` need to drive the physical SO-101 via lerobot. Entry 9.
- **One-euro filter.** Drop-in upgrade to `_smooth` once the live loop provides tuning data for the adaptive parameters.
- **Depth axis.** MediaPipe z → robot y is held fixed at `T_rest[1]`. Stereo or learned-depth extension would unlock full 3D teleop.
- **Wrist orientation tracking.** `R_fixed` locks to the rest orientation. Mapping wrist roll to `wrist_roll` is a natural v2 extension.

----

## Entry 9 - Hardware Bring-up

### Goal
Close the loop and drive the physical SO-101 from the live pipeline through lerobot, with teleop that genuinely reproduces my arm pose rather than vaguely twitching toward it. This is the long entry. Bring-up surfaced a chain of issues across every layer (joint frame, IK, workspace, mapping, smoothing), and each one masked the next until I fixed it.

### The hardware interface
`lerobot` (0.5.x) is the hardware layer: `SO101Follower` + `SO101FollowerConfig` from `lerobot.robots.so_follower`, with `use_degrees=True`. The five arm joints are driven by IK; the gripper is a sixth motor the IK never sees, held closed (`gripper.pos = 0`) for v1.

I made `RobotArm.to_action(thetalist)` a pure staticmethod that returns the `{joint.pos: deg}` action dict. It never calls `send_action`, so `RobotArm` stays free of lerobot and unit-testable, and `main` owns the I/O and the `success=False` policy: skip the send and let the servos hold their last goal. Since `step()` always targets the live wrist, recovery has no stale lurch. The teleop loop is paced by the camera (the generator blocks on `cap.read()`), so only the ramps need an explicit `sleep`. I first wrote `to_action` without `@staticmethod`, which would have bound the instance to `thetalist`, and fixed that early.

Startup and stow use closed-loop ramps. My first attempt was open-loop, interpolating from a fixed start over N steps paced by `sleep`, which assumes instant servo tracking. Under gravity the servo lags, the commanded goal outruns the present position, the `max_relative_target` clamp fires every step, and the loop ends before the arm arrives. The fix re-reads `get_observation()` each step and moves from the present position until within tolerance. A second pass made it synchronized, so the largest-gap joint moves a full step and the rest move the same fraction; that way all joints arrive together instead of the elbow settling last and dragging the gripper through the table.

Two lerobot gotchas are worth recording. `max_relative_target` must be a `float`, because `ensure_safe_goal_position` does an `isinstance(x, float)` check and re-raises the value otherwise, so an `int` `5` surfaced as `TypeError: 5`. And that clamp triggers an extra `Present_Position` read on every `send_action`, so it costs loop rate as well as bounding motion.

### Joint frame: URDF to lerobot
lerobot reports and accepts each joint in true degrees referenced to the motor's mechanical mid-range (`motors_bus._normalize`, DEGREES mode), while the URDF frame has its own zero and sign. Both are real degrees, so the map between them can only be a per-joint sign and offset.

I pinned it down with an isolated diagnostic (`tests/joint_check.py`, torque off), deliberately decoupled from IK and teleop. The key lever is that `get_observation()` returns raw motor degrees that never pass through my correction, so I can pose the arm by hand and compare the FK-predicted end-effector against the real gripper for ground truth. Two findings came out of it: the URDF home coincides with the mechanical mid-range (the URDF limits are symmetric, so their midpoint is the URDF zero), which makes every offset 0; and all five signs are +1. The map reduces to `deg_lerobot = degrees(theta_urdf)`, the identity. I still keep it as an explicit `JOINT_SIGN` and `JOINT_OFFSET` layer in `to_action`, so the convention is documented and re-derivable rather than implicit.

One trap cost me real time, so I want it on the record. Eyeballing teleop, shoulder_lift looked sign-flipped, and flipping it seemed to help, so I flipped its sign in the pose constants to match. Both were wrong. The isolated test proves the frame is identity, and what actually looked better was a coincidence inside a pipeline that was broken elsewhere. Worse, flipping the sign of the pose constants quietly relocated the whole teleop workspace, because it moved `T_rest` (see the rest-pose fix below). The lesson I took away is to determine joint signs in isolation, since teleop confounds too many errors at once to read a single sign off it.

### Why teleop was bad
The arm moved, but it reproduced my pose poorly. I peeled the causes back roughly in the order below.

- Orientation over-constraint. The bridge fixes the full rest orientation `R_fixed`, which makes every target six constraints (position and orientation) on a five-joint arm. Only the rest point itself sits on the reachable manifold, so every translated target failed and the joints saturated. The fix is position-only IK: I added a `position_only` flag to `IKinBodyDLS` that steps on the linear rows only (`Jv = J[3:6,:]`, `vb = Vb[3:6]`, damping `eye(3)`) and tests success on `v` alone. Orientation floats, which is acceptable for v1.

- Convergence. Position-only still returned mostly `False`, for two reasons. `ev = 1e-3` (1 mm) was far tighter than cm-level MediaPipe input warrants, and there was a warm-start chicken-and-egg: `theta_prev` advances only on success but was stuck at the far park seed while the solutions lived elsewhere, so it never converged to bootstrap itself. Loosening to `ev = 1e-2` lets near-misses register as success and walks the seed into the working region.

- Redundant-joint drift. Position-only leaves the arm redundant (five joints on a 2D target), and the leftover freedom was being resolved arbitrarily. The elbow sat wherever the seed left it, and the park seed pinned it near -75 degrees, so the arm stayed folded and unnatural. An early hack, advancing `theta_prev` every frame, backfired: with only the seed pinning the redundant pan and wrist_roll, chasing the previous unconstrained result let them drift to their limits and flail (around 110 and 157 degrees either way). The proper fix keeps advance-on-success, which gives a stable seed that pins the redundant joints, and adds a null-space secondary task to `IKinBodyDLS`, `dtheta += (I - Jv_pinv @ Jv) * k0 * (theta_pref - theta)`, which biases the redundant joints toward a preferred posture without disturbing position tracking. With `k0 = 0.3` and `theta_pref = REST`, the elbow relaxes from a seed-stuck -74 degrees to about -5 across the workspace, with IK success essentially unchanged.

- Workspace mismatch. The scale mapped my full reach onto `REACH ≈ 0.53 m`, but that is the workspace diagonal, applied per-axis, so it overshot the reachable set by roughly four times. Large fractions of my arm motion mapped to targets the arm cannot reach, and IK either froze (held position) or saturated. I fit the actual reachable region in the x-z plane by sweeping shoulder_lift and elbow over their limits: an annulus about the shoulder pivot `(0.07, 0.18)` with radius about `[0.165, 0.447]`. Targets are now built inside this band by construction.

- Rest at the top of the workspace. The original rest `[0, -1.3, 0, 0, 0]` placed the end-effector near the top of reach (z about 0.46), so almost every hand direction pushed the arm outward. It extended readily but barely folded. I moved `THETALIST_REST` to a central pose `[0, -37.7, 11.4, 19.2, 0]` degrees, with the end-effector around `(0.30, 0.30)` at mid-radius. Because `T_rest`, `R_fixed` and `theta_pref` all derive from this one constant, the single change recenters the mapping, the held orientation and the posture bias together. I also dropped the old park-at-calibration-pose startup, which left the arm a full reach from the first teleop target so `step()` lurched on frame one, in favour of ramping to REST and seeding `theta_prev = REST`. Hand-at-neutral now maps straight to the pose the ramp already parked at, so there is no first-frame jump, and I checked that the first step lands exactly on REST.

- Jitter. The fixed-alpha EMA forced a single jitter-versus-lag tradeoff, and the servos buzzed at rest. I replaced `_smooth` with a one-euro filter, whose cutoff adapts to the smoothed hand speed, so it filters heavily when my hand is still and lightly when it moves. It exposes two knobs, `min_cutoff` and `beta`, that I can tune live. This closes the Entry 8 open item, which only became actionable once there was a real signal to tune against.

- Command throttle. Folding is the largest reconfiguration in the workspace, about 44 degrees of elbow travel from rest, and the conservative 5-degrees-per-frame cap made it creep over roughly nine frames and never complete during normal motion. Extending, a smaller move, kept up, which is exactly why extend worked and fold did not. I raised the cap to 10 to 20 degrees, which lets a fold finish in a few frames while still bounding a runaway. The targets are already smoothed, radius-bounded and success-gated upstream, so the cap is a backstop rather than the safety story.

- The mapping itself. This was the deepest issue and the one that finally made folding work. The bridge mapped hand position to end-effector position, a translation anchored at rest. Under that map, folding the robot required moving my hand 30 to 55 cm leftward across my torso, which is anatomically impossible, so deep folds were never commandable no matter the scale. The metric scale was never the culprit: MediaPipe `pose_world_landmarks` are a learned, hip-centred metric estimate, designed to be invariant to camera distance, and per-session calibration cancels any body-size error. I replaced the map with a radial one that matches what I actually mean when I move my arm: extension `|shoulder to wrist|` sets the robot reach radius, and hand direction sets the end-effector angle about the workspace centre. Folding my elbow now shrinks the radius and curls the robot, extending reaches out, and direction tracking (up to up, forward to forward) is preserved. Extension over `[EXT_MIN_FRAC, 1]` maps onto radius `[WS_RMIN, MAP_RMAX]`, and near the shoulder, where direction is undefined, I hold the last direction to avoid snapping. This supersedes the Cartesian mapping of Entry 7 and the `scale = REACH/arm_length` calibration, since calibration now just records arm length.

### Robustness and housekeeping
- Calibration averages a window of frames (median of ~20) rather than trusting one noisy landmark frame; `start_calibration()` resets the buffer and `calibrate()` returns `True` once it fills.
- Frames whose shoulder/wrist landmark `visibility < 0.5` are skipped, so low-confidence tracking never drives the arm.
- The per-frame debug print is throttled to a heartbeat plus a line on every IK hold.
- I removed the code the rework made dead: `REACH`, the park and intermediate pose constants, and the workspace clamp (the radial map is in-band by construction).

### Verification
I confirmed the following on the physical arm: the URDF-to-lerobot frame is identity (FK matches the gripper at home and through the ramps), startup parks at the central rest with no jump into teleop, the arm both folds (I curl my elbow) and extends with the elbow holding a natural posture instead of collapsing to the seed, jitter is visibly down with the one-euro filter, and the synchronized ramps reach their targets without tripping the clamp. In sim, full extension reaches a radius of about 0.40 and a fully folded arm about 0.16 (the physical fold limit), varying monotonically with extension.

### Open questions
- **Orientation tracking.** Position-only floats the gripper orientation. Mapping my forearm and wrist to `wrist_flex`/`wrist_roll` (and dropping the `R_fixed` float) is the next fidelity step.
- **Depth.** The depth axis is still dropped (MediaPipe z → robot y held fixed). Stereo or learned depth would unlock the third dimension.
- **One-euro tuning.** `min_cutoff`/`beta` sit at sane defaults; they want a proper sweep against the live loop.

----

## Entry 10 - LeRobot Record Integration

### Goal
Record datasets through `lerobot-record` before spending any more time on motion quality. The point was to prove the whole data path end to end (teleop into record into a versioned `LeRobotDataset` that reloads and replays) rather than polish a pipeline that might not survive contact with the recording API. It went first because it is the only piece of this phase gated on an external, fast-moving library, and the one that kills the data-collection sprint if it slips.

### Registering as a third-party teleoperator
I expected to write a custom record script and ended up not needing one. lerobot's `register_third_party_plugins()` scans installed **distribution** names for a `lerobot_teleoperator_` prefix and imports the module of that exact name, so the whole hookup is a naming exercise. `[project] name` in `pyproject.toml` is now `lerobot_teleoperator_markerless`, which looks absurd for a project this size but is the thing that makes `--teleop.type=markerless` resolve in the stock CLI. Underscores have to survive into the metadata, and they do.

The rest falls out by convention. `@TeleoperatorConfig.register_subclass("markerless")` on the config class registers the type string, then construction falls through to `make_device_from_device_class()`, which finds the class by name (`MarkerlessTeleopConfig` gives `MarkerlessTeleop`) via the package `__init__.py` re-export. Keep that export or the lookup fails.

None of this is documented, which is the argument for pinning: `lerobot[feetech]==0.5.1` is the version this was actually proven against, and a convention that is not in the docs is a convention nobody promised to keep.

### Two startup jerks
The interesting part of this entry. A leader-follower rig never has a startup jerk because the leader reads its present position every frame, so the first command is wherever the leader already is. A camera has no such luxury, and it turns out there are two separate jerks hiding behind that, in two different spaces. I only made progress once I stopped treating them as one problem.

**Jerk A is in joint space:** the pose seeding `theta_prev` versus where the arm physically is at power-on. I killed it by making `THETALIST_PARK` the *measured torque-off limp reading*, `[0.57, -97.14, 96.53, 67.65, 1.63]` deg. On position-controlled servos only a low folded pose is a gravity equilibrium, so a powered-on arm is already sitting at the pose tick 0 commands. That is why the record CLI needs no startup ramp at all, while `main.py` still ramps.

That forced splitting the old single `THETALIST_REST` in two, because the seed pose and the mapping reference had been the same constant doing two unrelated jobs. `THETALIST_NEUTRAL` stays central and remains the mapping reference: it sets `R_fixed`, the held `y`, and the null-space `theta_pref`. `THETALIST_PARK` is the physical start pose and seeds `theta_prev` and `prev_dir`. `RobotArm` now takes both as keyword-only arguments, which keeps it dependency-injected (so `test_bridge` stays hardware-free) and stops two same-typed pose vectors being swapped positionally, which is a bug I would never have found by reading.

**Jerk B is in task space:** the robot's starting end-effector versus wherever my hand happens to be mapping when teleop starts. This is the one the leader-follower setup punts on by snapping the follower to the leader, and it is worse here because the arm can be a full reach away. The fix is a ready-pose gate: hold `PARK` and refuse to drive until the mapped `(x, z)` sits within `gate_eps` of `FK(PARK)`'s end-effector for `gate_n_frames` consecutive frames.

The gate matches in **task space, not on "is the arm folded"**, and the difference matters. Fully folded maps to `r = 0.165 m` but `PARK` sits at `r = 0.206 m`, so a folded check would still hand over with a 4 cm jump. Matching the mapped target means the IK, seeded at `PARK`, returns approximately `PARK`, so the handover is a no-op. It also stays correct if I ever re-measure `PARK`, which a hardcoded posture check would not. `gate_eps = 0.025` and `gate_n_frames = 8` were right first try.

One practical thing: under `lerobot-record` there is no window of mine on screen, so hunting a 2.5 cm target is blind. A bare distance says how far but not which way, so the gate prints the two knobs I actually control, arm extension and hand direction, with the direction to move each.

### Perception on its own thread
`get_action()` is called by the record loop at record fps and has to return promptly, so MediaPipe inference cannot sit in that path. `PerceptionThread` runs capture, detect and publish on a background thread and hands the consumer the latest result through a single-reference slot. Reading one immutable `Snapshot` is atomic under the GIL, so a consumer never sees a half-written one, and there is no lock to contend.

Deliberately **no cv2 GUI in the thread**. `imshow` and `waitKey` are main-thread affine and flaky off-thread on Windows, so the thread publishes an `annotated_frame` and whoever owns the main thread displays it. A raising thread dies silently, so exceptions are stashed on `.error` for the consumer to notice instead of the loop just going quiet. I extracted `PoseDetector` first so the thread and `video_mapping` are both thin clients of one MediaPipe home rather than two copies drifting apart.

The subtle rule is the sampling one, and it bit before I wrote it down. `map_target` runs on every valid new frame whether or not the gate has armed, because the one-euro filter derives its `dt` from wall-clock between calls and so has to see true camera cadence. For the same reason `get_action()` must skip **stale** frames, spotted by an unchanged `frame_id`: record fps and camera fps are different numbers, and sampling the filter at the wrong one corrupts the speed estimate and therefore the adaptive cutoff. Skipping it while unarmed, or calling it twice in a tick, both break it in the same way.

`camera_index` on the constructor and a capture `timestamp` on the snapshot are the two cheap hooks for a second camera later. Two instances plus a combiner that pairs snapshots by timestamp, no triangulation, and none of it built now.

### Remapping extension to radius
Rehearsing a recording showed the extension mapping was not good enough to produce data worth keeping, so it got fixed before recording rather than after. The original map was linear from `EXT_MIN_FRAC = 0.15` to `MAP_RMAX = 0.40` with one constant gain end to end, and that gain was wrong at both ends. Close-in reaching, table height for instance, needed my wrist uncomfortably folded (within 15% of `arm_length` of my shoulder to hit `WS_RMIN`), while ordinary motion pushed the robot close enough to its true maximum reach of about 0.447 m that IK went visibly jerky near the ill-conditioned Jacobian.

My first fix was a global `ext_frac**k` power curve, and it over-corrected in an instructive way. It concentrates resolution at full fold, but the precision I actually want is centred near `PARK`'s own radius of about 0.206 m, not at the extreme. It pushed the raw arm extension needed to reach `PARK` from roughly 30% to 61%, and floored the gate distance around 4 cm, which is outside the 2.5 cm `gate_eps`, so the gate stopped arming at all. `PARK`'s radius is a fixed hardware constant, it is just `FK(THETALIST_PARK)`, so any curve that concentrates resolution below where `PARK` sits can only ever make `PARK` harder to reach. That is worth remembering before reshaping this again.

What landed is two-segment piecewise-linear through `np.interp`, with `EXT_BREAK_FRAC = 0.40` and `RADIUS_BREAK_FRAC = 0.30`: a gentle 0.75 slope from the floor through the fold-to-table-reach band where I actually work, and a steep segment past it for the rare full-stretch gesture. `EXT_MIN_FRAC` went up to 0.2 as a genuine comfort floor. `PARK` now needs about 48% raw extension, worse than the original 30% but comfortably inside the gentle segment, and the working band has roughly 47% less gain than before. `gate_eps` went to 0.035 to match the reshaped map. `tests/test_bridge.py` stayed green through all of it because it imports `MAP_RMAX` and `EXT_MIN_FRAC` live instead of hardcoding expected radii, which is the only reason retuning constants was cheap.

### Housekeeping
- `lerobot` depends on `opencv-python-headless` while this repo needs `opencv-contrib-python`, and they share one `cv2/` directory. Any pip run that touches lerobot can silently strip GUI support, and `cv2.imshow` then raises "The function is not implemented". Fix is `pip install --force-reinstall --no-deps "opencv-contrib-python==4.13.0.92"`.
- Editable installs record a **static** snapshot of the packages list, so adding a package to `pyproject.toml` leaves it unresolvable outside the repo root until `pip install -e .` is re-run. This cost me time twice before I wrote it down.
- Right arm only, landmarks 12 and 16. Calibrating with the left arm silently measures the *resting* right arm and quietly corrupts `arm_length`, with no error anywhere. The prompt now says which arm.
- `send_feedback` raises `NotImplementedError` rather than no-oping, so it fails loudly if the record loop ever starts calling it. It does not.
- `PerceptionThread.start()` returns before the camera and MediaPipe finish initialising, so the calibration prompt can print before the camera light comes on. That is thread scheduling, not a race worth fixing.

### Verification
Recorded `JaimeLR/markerless_c6`, 5 episodes and 2995 frames, and checked it against the acceptance criteria rather than eyeballing it. It reloads through `LeRobotDataset`; timestamps are uniform at 1/30 s with standard deviation 0.0 in every episode; `observation.images.scene` is present and genuinely decodes (I checked for non-trivial pixel values rather than trusting that a stream exists, since a black frame would also "be present"). Actions show continuous discontinuity-free motion in `shoulder_lift`, `elbow_flex` and `wrist_flex`. Episode 1 replayed through `lerobot-replay` without incident.

Two things in that dataset look wrong and are not. `shoulder_pan` and `wrist_roll` sit near-flat and `gripper.pos` is exactly 0 throughout, both expected: the radial mapping never drives pan or roll, and there is no gripper channel yet. Episode 0 also holds flat at `PARK` for about 17.6 s before the gate arms, because `configure()` re-arms once per `connect()` rather than once per episode. Episodes 1 to 4 are live from frame 1.

### Open questions
- **Gate arming is per connect, not per episode.** Episode 0 carries a long parked prefix as a result. Re-arming per episode would cost a re-pose at every episode start, so this is a trade rather than a bug, but it should be a decision rather than an accident.
- **Fixed-rate control and velocity-level IK.** The commanded position only changes when perception delivers, at roughly 15 to 25 Hz and irregularly, so the action stream is hold, hold, step, hold. Pacing the loop does not fix that; integrating toward the latest target every tick does. That is the next piece of work.
- **Gripper channel.** Needed before any manipulation demo, and it is the one obviously empty column in the recorded data.
- **Depth.** Still the missing axis, and still the largest single fidelity gain available.
