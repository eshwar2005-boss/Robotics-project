"""
robot_arm.py
------------
A 6-DOF articulated robot arm model (FANUC LR Mate style) built on
standard Denavit-Hartenberg (DH) parameters.

Provides:
    - Forward Kinematics (FK)   : joint angles -> end-effector pose
    - Inverse Kinematics (IK)   : end-effector pose -> joint angles
                                   (numerical, Damped Least Squares Jacobian method)
    - Jacobian computation
    - Joint limit checking

Author: <your name>
"""

import numpy as np


class RobotArm:
    """
    6-DOF serial manipulator defined by DH parameters.

    DH convention used: [theta, d, a, alpha]  (theta is the variable joint angle)
    Values below approximate a small industrial arm (similar class to a
    FANUC LR Mate 200iD) - link lengths in millimeters.
    """

    def __init__(self):
        # DH table: [d (mm), a (mm), alpha (rad)]  -- theta is variable (the joint angle)
        self.dh_d = np.array([330.0, 0.0, 0.0, 320.0, 0.0, 80.0])
        self.dh_a = np.array([75.0, 300.0, 75.0, 0.0, 0.0, 0.0])
        self.dh_alpha = np.array([-np.pi / 2, 0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, 0.0])

        self.n_joints = 6

        # Joint limits in radians (approximate, symmetric for simplicity)
        self.joint_limits = np.array([
            [-np.radians(170), np.radians(170)],
            [-np.radians(100), np.radians(145)],
            [-np.radians(170), np.radians(170)],
            [-np.radians(190), np.radians(190)],
            [-np.radians(120), np.radians(120)],
            [-np.radians(360), np.radians(360)],
        ])

    # ------------------------------------------------------------------ #
    #  Forward Kinematics
    # ------------------------------------------------------------------ #
    @staticmethod
    def _dh_matrix(theta, d, a, alpha):
        """Standard DH homogeneous transformation matrix for one joint."""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,        sa,       ca,      d],
            [0,         0,        0,      1],
        ])

    def forward_kinematics(self, joint_angles, return_all_frames=False):
        """
        Compute the end-effector pose given joint angles (radians).

        Parameters
        ----------
        joint_angles : array-like, shape (6,)
        return_all_frames : bool
            If True, also return the transform of every intermediate joint
            (useful for plotting the arm's links).

        Returns
        -------
        T : (4,4) ndarray          -- end-effector homogeneous transform
        frames : list[(4,4)]       -- only if return_all_frames=True
        """
        joint_angles = np.asarray(joint_angles, dtype=float)
        assert joint_angles.shape[0] == self.n_joints

        T = np.eye(4)
        frames = [T.copy()]
        for i in range(self.n_joints):
            Ti = self._dh_matrix(joint_angles[i], self.dh_d[i], self.dh_a[i], self.dh_alpha[i])
            T = T @ Ti
            frames.append(T.copy())

        if return_all_frames:
            return T, frames
        return T

    def get_joint_positions(self, joint_angles):
        """Return the xyz position (mm) of the base + every joint, for plotting."""
        _, frames = self.forward_kinematics(joint_angles, return_all_frames=True)
        return np.array([f[:3, 3] for f in frames])

    # ------------------------------------------------------------------ #
    #  Jacobian
    # ------------------------------------------------------------------ #
    def jacobian(self, joint_angles):
        """
        Geometric Jacobian (6x6): relates joint velocities to end-effector
        linear + angular velocity.
        """
        _, frames = self.forward_kinematics(joint_angles, return_all_frames=True)
        o_n = frames[-1][:3, 3]

        J = np.zeros((6, self.n_joints))
        for i in range(self.n_joints):
            z_i = frames[i][:3, 2]      # z-axis of joint i frame
            o_i = frames[i][:3, 3]      # origin of joint i frame
            J[:3, i] = np.cross(z_i, (o_n - o_i))
            J[3:, i] = z_i
        return J

    # ------------------------------------------------------------------ #
    #  Inverse Kinematics  (Damped Least Squares / Levenberg-Marquardt)
    # ------------------------------------------------------------------ #
    def inverse_kinematics(self, target_pos, target_rot=None, q_init=None,
                            max_iters=200, tol=1e-3, damping=0.05):
        """
        Numerically solve for joint angles that place the end-effector at
        `target_pos` (xyz, mm) with optional orientation `target_rot` (3x3).

        Uses the Damped Least Squares (DLS) method, which is numerically
        stable near singularities (an issue real robot controllers must
        also handle).

        Returns
        -------
        q : (6,) ndarray of joint angles (radians)
        success : bool
        iterations : int
        """
        if q_init is None:
            q = np.zeros(self.n_joints)
        else:
            q = np.array(q_init, dtype=float)

        target_pos = np.asarray(target_pos, dtype=float)

        for it in range(max_iters):
            T, frames = self.forward_kinematics(q, return_all_frames=True)
            cur_pos = T[:3, 3]
            pos_err = target_pos - cur_pos

            if target_rot is not None:
                cur_rot = T[:3, :3]
                rot_err_mat = target_rot @ cur_rot.T
                rot_err = 0.5 * np.array([
                    rot_err_mat[2, 1] - rot_err_mat[1, 2],
                    rot_err_mat[0, 2] - rot_err_mat[2, 0],
                    rot_err_mat[1, 0] - rot_err_mat[0, 1],
                ])
                err = np.concatenate([pos_err, rot_err])
            else:
                err = pos_err

            if np.linalg.norm(pos_err) < tol:
                return self._wrap_to_limits(q), True, it

            J = self.jacobian(q)
            if target_rot is None:
                J = J[:3, :]

            # Damped least squares pseudo-inverse
            JJt = J @ J.T
            lam2 = damping ** 2
            dq = J.T @ np.linalg.solve(JJt + lam2 * np.eye(JJt.shape[0]), err)

            q = q + dq
            q = self._clip_to_limits(q)

        return self._wrap_to_limits(q), False, max_iters

    def _clip_to_limits(self, q):
        return np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])

    def _wrap_to_limits(self, q):
        return np.array([np.clip(qi, lo, hi) for qi, (lo, hi) in zip(q, self.joint_limits)])

    def in_limits(self, q):
        return np.all(q >= self.joint_limits[:, 0]) and np.all(q <= self.joint_limits[:, 1])
