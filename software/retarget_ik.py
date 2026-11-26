"""UMI pose -> Sourccey arm joints (5 joints: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll).
Own FK/IK on top of the official URDF (vulcan-forge/sourccey-hardware, URDF/Arms/ArmLeft/ArmLeft.urdf). Only numpy and scipy.

WARNING: with 5 joints the arm cannot reach an arbitrary 6D pose. The functions return the position error (mm) and the
orientation error (deg) per frame. Tool frame = finger pivot axis, x forward (jaw tips), y = hinge axis, z up
(the same frame used by the CAD in this project).
"""
import re, numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
# finger pivot in the Gripper-Base frame (m) and tool-frame -> Gripper-Base rotation (from the real STL/URDF)
T_GB_TOOL = np.eye(4); T_GB_TOOL[:3, :3] = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float); T_GB_TOOL[:3, 3] = [0.000375, -0.030375, 0.015]

def _T(xyz, rpy):
    T = np.eye(4); T[:3, :3] = Rot.from_euler("xyz", rpy).as_matrix(); T[:3, 3] = xyz; return T

class SourcceyArm:
    def __init__(self, urdf_path, root="base_link", tip="Gripper-Base"):
        t = open(urdf_path).read(); self.joints = {}
        children = {}
        for m in re.finditer(r'<joint name="([^"]*)" type="([^"]*)">(.*?)</joint>', t, re.S):
            n, ty, b = m.groups()
            par = re.search(r'parent link="([^"]*)"', b).group(1); ch = re.search(r'child link="([^"]*)"', b).group(1)
            o = re.search(r'<origin xyz="([^"]*)" rpy="([^"]*)"', b)
            xyz = [float(v) for v in o.group(1).split()] if o else [0, 0, 0]; rpy = [float(v) for v in o.group(2).split()] if o else [0, 0, 0]
            ax = re.search(r'<axis xyz="([^"]*)"', b); axis = np.array([float(v) for v in ax.group(1).split()]) if ax else None
            self.joints[n] = dict(type=ty, parent=par, child=ch, T=_T(xyz, rpy), axis=axis); children.setdefault(par, []).append(n)
        path, stack = None, [(root, [])]                      # search the path root -> tip
        while stack:
            link, p = stack.pop()
            if link == tip: path = p; break
            for j in children.get(link, []): stack.append((self.joints[j]["child"], p + [j]))
        if path is None: raise ValueError("no kinematic path from base_link to Gripper-Base")
        self.path = path
        self.active = [j for j in path if self.joints[j]["type"] == "revolute"]
        assert self.active == ARM_JOINTS, self.active

    def fk(self, q):
        T = np.eye(4); k = 0
        for j in self.path:
            J = self.joints[j]; T = T @ J["T"]
            if J["type"] == "revolute":
                a = J["axis"] / np.linalg.norm(J["axis"]); R = np.eye(4); R[:3, :3] = Rot.from_rotvec(a * q[k]).as_matrix(); T = T @ R; k += 1
        return T @ T_GB_TOOL                                    # tool (pivot) pose in the base_link frame

    def ik(self, T_target, q0, w_ori=0.15, bounds=(-np.pi, np.pi)):
        def res(q):
            T = self.fk(q); dp = T[:3, 3] - T_target[:3, 3]
            dr = Rot.from_matrix(T[:3, :3].T @ T_target[:3, :3]).as_rotvec()
            return np.r_[dp, w_ori * dr]
        s = least_squares(res, q0, bounds=bounds, xtol=1e-10, ftol=1e-10)
        T = self.fk(s.x); err_p = np.linalg.norm(T[:3, 3] - T_target[:3, 3]) * 1000
        err_r = np.degrees(np.linalg.norm(Rot.from_matrix(T[:3, :3].T @ T_target[:3, :3]).as_rotvec()))
        return s.x, err_p, err_r

    def retarget(self, poses_world, T_base_world, T_tracker_tool, q0=None):
        """poses_world: list of 4x4 TRACKER poses in the tracker world. Returns (q[N,5], err_pos_mm[N], err_ori_deg[N])."""
        q = np.zeros(5) if q0 is None else np.asarray(q0, float); Q, EP, ER = [], [], []
        for Tw in poses_world:
            T_tool = T_base_world @ Tw @ T_tracker_tool        # tool in the robot's base_link frame
            q, ep, er = self.ik(T_tool, q); Q.append(q.copy()); EP.append(ep); ER.append(er)
        return np.array(Q), np.array(EP), np.array(ER)
