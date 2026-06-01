"""Forward-kinematics verification against yourdfpy.

Run from the repo root:  python -m tests.test_fk
"""
import numpy as np
import yourdfpy

from kinematics.core import FKinSpace
from urdf.parser import DEFAULT_URDF
from tests.robot import M, Slist, JOINTS

_ref = yourdfpy.URDF.load(str(DEFAULT_URDF), load_meshes=False)
_zero_cfg = {j: 0.0 for j in _ref.actuated_joint_names}


def verify_fk(thetalist):
    cfg = {**_zero_cfg, **{j: float(v) for j, v in zip(JOINTS, thetalist)}}
    _ref.update_cfg(cfg)
    T_ref = _ref.get_transform("gripper_frame_link", "base_link")
    T_mine = FKinSpace(M, Slist, np.array(thetalist))
    err = np.max(np.abs(T_ref - T_mine))
    print(f"{'PASS' if err < 1e-4 else 'FAIL'}  max_err={err:.2e}  theta={np.round(thetalist, 3)}")


if __name__ == "__main__":
    verify_fk([0, 0, 0, 0, 0])
    verify_fk([-np.pi / 8, 0, 0, 0, 0])
    verify_fk([0, np.pi / 4, 0, 0, 0])
    verify_fk([0, 0, -np.pi / 4, 0, 0])
    verify_fk([np.pi / 8, -np.pi / 4, np.pi / 6, -np.pi / 8, np.pi / 3])
