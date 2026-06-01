"""Inverse-kinematics verification: round-trip success and singularity behaviour.

Run from the repo root:  python -m tests.test_ik
"""
import numpy as np
import matplotlib.pyplot as plt

from kinematics.core import FKinBody, JacobianBody, TransInv, MatrixLog6, se3ToVec
from kinematics.ik import IKinBodyDLS
from tests.robot import M, Blist, limits


def round_trip(joint_limits, iters, noise):
    """FK(theta) -> IK -> compare. Returns success rate (non-converged count as failures)."""
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

    return passed / iters


def verify_singular(noise=0.05, maxiters=50):
    """Compares DLS vs plain pseudoinverse stepping near an elbow singularity."""
    theta_singular = np.array([0.0, np.pi / 2, 0.0, 0.0, 0.0])
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

    hist_dls = run_ik(use_dls=True, lam=0.05)
    hist_pinv = run_ik(use_dls=False)
    print(f"DLS:  {len(hist_dls)-1} iters   Pinv: {len(hist_pinv)-1} iters")

    labels = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
    _, axes = plt.subplots(1, 2, figsize=(12, 4), sharey=False)
    for i, label in enumerate(labels):
        axes[0].plot(hist_dls[:, i], label=label)
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


def noise_sweep():
    """Plots IK round-trip success rate vs the magnitude of initial-guess noise."""
    noise_levels = [0, 0.01, 0.1, 0.5, 0.75, 1, 1.25, 1.5, 2, 5]
    iters = 200
    success_rates = [round_trip(limits, iters, n) for n in noise_levels]

    for n, r in zip(noise_levels, success_rates):
        print(f"noise={n:5.2f}  success={r*100:.1f}%")

    plt.figure()
    plt.plot(noise_levels, [r * 100 for r in success_rates], marker='o')
    plt.xscale('symlog', linthresh=0.01)
    plt.xlabel("Noise magnitude (rad)")
    plt.ylabel("Success rate (%)")
    plt.title("IK round-trip success vs initial guess noise")
    plt.ylim(0, 105)
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    print(f"round-trip success (noise=1): {round_trip(limits, 100, 1):.2f}")
    # noise_sweep()      # uncomment for the success-rate-vs-noise plot
    # verify_singular()  # uncomment for the DLS-vs-pseudoinverse singularity plot
