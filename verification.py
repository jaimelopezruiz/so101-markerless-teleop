import yourdfpy
import numpy as np
from core import *
from kinematics import *
from urdfpy import findMnS
import matplotlib.pyplot as plt

M, Slist, limits = findMnS()
Blist = np.array([Adjoint(TransInv(M)) @ Slist[:, i] for i in range(Slist.shape[1])]).T
theta_test = np.zeros(5)
T_space = FKinSpace(M, Slist, theta_test)
T_body  = FKinBody(M, Blist, theta_test)
# Order matches how findMnS builds Slist (reversed URDF order, arm joints only)
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

_ref = yourdfpy.URDF.load("so101_new_calib.urdf", load_meshes=False)
_zero_cfg = {j: 0.0 for j in _ref.actuated_joint_names}

def verify_fk(thetalist):
    cfg = {**_zero_cfg, **{j: float(v) for j, v in zip(JOINTS, thetalist)}}
    _ref.update_cfg(cfg)
    T_ref  = _ref.get_transform("gripper_frame_link", "base_link")
    T_mine = FKinSpace(M, Slist, np.array(thetalist))
    err = np.max(np.abs(T_ref - T_mine))
    print(f"{'PASS' if err < 1e-4 else 'FAIL'}  max_err={err:.2e}  theta={np.round(thetalist, 3)}")

def verify_jac(thetalist):
    step = 1e-4  # must be > NearZero threshold (1e-6) in core.MatrixExp6
    J_num = np.zeros((6,len(thetalist)))

    for i in range(len(thetalist)):
        theta_plus = thetalist.copy()
        theta_plus[i] += step

        T_plus = FKinSpace(M, Slist, theta_plus)
        T_base = FKinSpace(M, Slist, thetalist)

        dX = (T_plus @ np.linalg.inv(T_base)) / step

        omega = [dX[2,1], dX[0,2], dX[1,0]]
        v = dX[:3, 3]

        J_num[:, i] = np.concatenate([omega, v])

    J_analytical = JacobianSpace(Slist, thetalist)

    err = np.max(np.abs(J_analytical - J_num))
    print(f"{'PASS' if err < 1e-4 else 'FAIL'}  max_err={err:.2e}  theta={np.round(thetalist, 3)}")
    
def round_trip(joint_limits, iters, noise):
    passed, ik_failed = 0, 0

    for _ in range(iters):
        thetalist = np.random.uniform(low=joint_limits[:, 0], high=joint_limits[:, 1])
        T_target = FKinBody(M, Blist, thetalist)
        theta_init = thetalist + np.random.uniform(-noise, noise, size=thetalist.shape)
        theta_solved, success = IKinBodyDLS(Blist, M, T_target, theta_init, joint_limits)

        if not success:
            ik_failed += 1
            continue

        Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, theta_solved)) @ T_target))
        if np.linalg.norm(Vb[:3]) < 1e-2 and np.linalg.norm(Vb[3:]) < 1e-3:
            passed += 1

    return passed / iters  # success rate including non-converged as failures

print(round_trip(limits, 100, 1))
# # Uncomment for forward kinematics verification
# verify_fk([0, 0, 0, 0, 0])
# verify_fk([-np.pi/8, 0, 0, 0, 0])
# verify_fk([0, np.pi/4, 0, 0, 0])
# verify_fk([0, 0, -np.pi/4, 0, 0])
# verify_fk([np.pi/8, -np.pi/4, np.pi/6, -np.pi/8, np.pi/3])

# # Uncomment for jacobean verification
# verify_jac([0, 0, 0, 0, 0])
# verify_jac([-np.pi/8, 0, 0, 0, 0])
# verify_jac([0, np.pi/4, 0, 0, 0])
# verify_jac([0, 0, -np.pi/4, 0, 0])
# verify_jac([np.pi/8, -np.pi/4, np.pi/6, -np.pi/8, np.pi/3])


def verify_singular(noise=0.05, maxiters=50):
    theta_singular = np.array([0.0, np.pi/2, 0.0, 0.0, 0.0])
    theta_init = theta_singular.copy()
    T_target = FKinBody(M, Blist, theta_singular + np.array([0.0, 0.1, 0.0, 0.1, 0.0]))


    J_sing = JacobianBody(Blist, theta_singular)
    print(f"Jacobian condition number at singular config: {np.linalg.cond(J_sing):.1f}")

    def run_ik(use_dls, lam=0.05):
        theta = theta_init.copy()
        history = [theta.copy()]
        for _ in range(maxiters):
            Tsb = FKinBody(M, Blist, theta)
            Vb = se3ToVec(MatrixLog6(TransInv(Tsb) @ T_target))
            if np.linalg.norm(Vb[:3]) < 1e-2 and np.linalg.norm(Vb[3:]) < 1e-3:
                break
            J = JacobianBody(Blist, theta)
            if use_dls:
                delta = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), Vb)
            else:
                delta = np.linalg.pinv(J) @ Vb
            if not np.all(np.isfinite(delta)):
                break
            theta = theta + delta
            history.append(theta.copy())
        return np.array(history)

    hist_dls  = run_ik(use_dls=True,  lam=0.05)
    hist_pinv = run_ik(use_dls=False)
    print(f"DLS:  {len(hist_dls)-1} iters   Pinv: {len(hist_pinv)-1} iters")

    labels = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
    _, axes = plt.subplots(1, 2, figsize=(12, 4), sharey=False)
    for i, label in enumerate(labels):
        axes[0].plot(hist_dls[:, i],  label=label)
        axes[1].plot(hist_pinv[:, i], label=label)
    axes[0].set_title('DLS (λ=0.05)')
    axes[1].set_title('Pseudoinverse')
    for ax in axes:
        ax.set_xlabel('Iteration')
        ax.set_ylabel('Joint angle (rad)')
        ax.legend(fontsize=8)
        ax.grid(True)
    plt.suptitle(f'IK near elbow singularity (elbow_flex=π/2), noise={noise}')
    plt.tight_layout()
    plt.show()

noise_levels = [0, 0.01, 0.1, 0.5, 0.75, 1, 1.25, 1.5, 2, 5]
iters = 200

# success_rates = [round_trip(limits, iters, n) for n in noise_levels]

# for n, r in zip(noise_levels, success_rates):
#     print(f"noise={n:5.2f}  success={r*100:.1f}%")

# plt.figure()
# plt.plot(noise_levels, [r * 100 for r in success_rates], marker='o')
# plt.xscale('symlog', linthresh=0.01)
# plt.xlabel("Noise magnitude (rad)")
# plt.ylabel("Success rate (%)")
# plt.title("IK round-trip success vs initial guess noise")
# plt.ylim(0, 105)
# plt.grid(True)
# plt.tight_layout()
# plt.show()

# verify_singular()