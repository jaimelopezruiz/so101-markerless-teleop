import xml.etree.ElementTree as ET
import numpy as np
from types import SimpleNamespace

tree = ET.parse('so101_new_calib.urdf')
root = tree.getroot()

def rpyToRot(rpy):
   r, p, y = rpy
   Rx = np.array([[1, 0,        0       ],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r),  np.cos(r)]])
   Ry = np.array([[ np.cos(p), 0, np.sin(p)],
                   [ 0,         1, 0        ],
                   [-np.sin(p), 0, np.cos(p)]])
   Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y),  np.cos(y), 0],
                   [0,          0,         1]])
   return Rz @ Ry @ Rx

robot = SimpleNamespace()

def findMnS():
    T = np.eye(4)
    Slist = np.empty((6, 0))
    limits = []
    for joint in reversed(root.findall('joint')):
        origin = joint.find('origin')
        axis = joint.find('axis')
        name = joint.get('name')

        T_local = np.zeros((4,4))
        R_cumulative = np.zeros((3, 3))

        if origin is not None and name != 'gripper':
            setattr(robot, name, SimpleNamespace(
                xyz=np.array([float(v) for v in origin.get('xyz').split()]),
                rpy=np.array([float(v) for v in origin.get('rpy').split()]),
                axis = np.array([float(v) for v in axis.get('xyz').split()])
            ))

            ## FOR M:
            T_local[:3, :3] = rpyToRot(getattr(robot, name).rpy)
            T_local[:3, 3] = getattr(robot, name).xyz
            T_local[3, 3] = 1

            T = T @ T_local
            R_cumulative = T[:3, :3]
            

            ## FOR Slist and limits:
            if axis is not None and joint.get('type') != 'fixed' and name != 'gripper':
                omega = R_cumulative @ getattr(robot, name).axis
                q = T[:3, 3]
                v = -np.cross(omega, q)
                S_local = np.concatenate([omega, v]).reshape(6, 1)
                Slist = np.hstack([Slist, S_local])

                lim = joint.find('limit')
                limits.append([float(lim.get('lower')), float(lim.get('upper'))])

    return T, Slist, np.array(limits)
