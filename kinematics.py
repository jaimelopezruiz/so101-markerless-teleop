import numpy as np
from core import *
from urdfpy import *



def IKinBodyDLS(Blist, M, T, thetalist0, joints_limits, eomg=1e-2, ev=1e-3, lam=0.01, maxiters=200):
    """Computes inverse kinematics in the body frame for an open chain robot.

    Uses damped least-squares (DLS) Newton-Raphson iteration. Joint limit
    clamping is applied to the final result only — clamping inside the loop
    corrupts the gradient and prevents convergence.

    :param Blist: Screw axes in the end-effector (body) frame at home, columns
    :param M: Home configuration of the end-effector (4x4)
    :param T: Desired end-effector configuration Tsd (4x4)
    :param thetalist0: Initial joint angle guess (n,)
    :param joints_limits: Joint limits array (n, 2), columns [lower, upper]
    :param eomg: Angular error tolerance (rad)
    :param ev: Linear error tolerance (m)
    :param lam: DLS damping factor — higher = more stable but slower
    :param maxiters: Maximum Newton-Raphson iterations
    :return: (thetalist, success) — clamped joint angles and convergence flag
    """
    thetalist = np.array(thetalist0).copy()
    i = 0

    Tsb = FKinBody(M, Blist, thetalist)
    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(Tsb), T)))  # Body twist to desired pose

    omega_b_mag = np.linalg.norm(Vb[0:3])  # Angular error magnitude
    v_b_mag     = np.linalg.norm(Vb[3:6])  # Linear error magnitude
    err = omega_b_mag > eomg or v_b_mag > ev

    while err and i < maxiters:
        thetalist_previous = thetalist.copy()

        J = JacobianBody(Blist, thetalist)
        delta_theta = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), Vb)  # DLS step

        if not np.all(np.isfinite(delta_theta)):  # NaN/Inf guard — bail with last good theta
            return (thetalist_previous, False)

        thetalist = thetalist + delta_theta
        i += 1

        Tsb = FKinBody(M, Blist, thetalist)
        Vb = se3ToVec(MatrixLog6(np.dot(TransInv(Tsb), T)))
        omega_b_mag = np.linalg.norm(Vb[0:3])
        v_b_mag     = np.linalg.norm(Vb[3:6])
        err = omega_b_mag > eomg or v_b_mag > ev

        # Clamp to joint limits WITHIN the while loop
        thetalist = np.clip(thetalist, joints_limits[:, 0], joints_limits[:, 1])

    return (thetalist, not err)

M, Slist, limits = findMnS()
thetalist = np.array([-np.pi / 8, 0, 0, 0, 0])   # Test theta
