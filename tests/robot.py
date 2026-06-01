"""Shared SO-101 model fixtures for the verification scripts."""
import numpy as np

from kinematics.core import Adjoint, TransInv
from urdf.parser import findMnS

M, Slist, limits = findMnS()
# Body-frame screw axes: Blist = [Ad_{M^-1}] Slist, column by column.
Blist = np.array([Adjoint(TransInv(M)) @ Slist[:, i] for i in range(Slist.shape[1])]).T

# Order matches how findMnS builds Slist (reversed URDF order, arm joints only)
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]