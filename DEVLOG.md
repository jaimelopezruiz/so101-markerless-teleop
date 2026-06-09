# DEVLOG

Development log for markerless teleoperation project using LeRobot's SO-101 arm.

> A note on ordering: this is a journal, but the foundation layer (Entries 1-6) is
> written up after the fact. Perception, URDF parsing, FK, Jacobians, and IK were
> each built and verified on their own before being documented, so these entries
> follow the build order (each depends on the previous one) rather than diary
> chronology. Day-by-day journaling resumes at the bridge layer (Entry 7+), where
> design and writing happen together. Might make a cleaner reference doc once finished.

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
    C --> D[Frame transform<br/>+ scaling]
    D --> E[Construct T_sd]
    E --> F[IK solver<br/>DLS + Newton-Raphson]
    F --> G[Joint angles θ]
    G --> H[SO-101 via lerobot]
    G -. "warm start: previous θ seeds next frame's solve" .-> F
```

The dotted feedback edge is the warm start: the iterative IK solver needs an initial
guess, so each frame seeds the solve with the previous frame's solution. This is
faster (the target moves little between frames) and keeps the joint trajectory
smooth over time. It is enabled by the solver's `thetalist0` argument and will be
driven by the bridge layer (Entry 7+).

### Repo structure

```text
so101-markerless-teleop/
├── README.md            # layout, setup, how to run
├── DEVLOG.md            # this file
├── pyproject.toml       # editable-install packaging (kinematics, urdf, perception, tests)
├── requirements.txt     # numpy, matplotlib, mediapipe, opencv-python, yourdfpy
├── .gitignore
├── kinematics/
│   ├── core.py          # trimmed Modern Robotics lib, 18 functions (was 40+)
│   └── ik.py            # IKinBodyDLS
├── urdf/
│   ├── parser.py        # findMnS + rpyToRot (path-safe, no module globals)
│   └── so101_new_calib.urdf
├── perception/
│   ├── pose_detector.py # draw_landmarks_on_image
│   ├── image_mapping.py # still-image demo
│   └── video_mapping.py # webcam demo
├── models/
│   └── pose_landmarker_heavy.task   # 30 MB, git-ignored
└── tests/
    ├── robot.py         # shared M, Slist, Blist, limits fixtures
    ├── test_fk.py       # verify_fk + driver
    ├── test_jacobian.py # verify_jac + driver
    └── test_ik.py       # round_trip, verify_singular, noise_sweep + driver
```

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
| Bridge layer (perception to IK input) | Next | n/a |
| Hardware execution | Pending | n/a |

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
