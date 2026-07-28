"""
trajectory.py
-------------
Joint-space trajectory generation for smooth point-to-point robot motion.

Implements quintic (5th-order) polynomial interpolation, which guarantees
continuous position, velocity, and acceleration at the start and end of
motion - important for reducing mechanical shock on real actuators
(the same principle used in FANUC's own motion controllers).
"""

import numpy as np


def quintic_coefficients(q0, qf, T, v0=0.0, vf=0.0, a0=0.0, af=0.0):
    """
    Compute quintic polynomial coefficients for a single joint moving
    from q0 to qf in time T, with given boundary velocity/acceleration.

    q(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
    """
    A = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 2, 0, 0, 0],
        [1, T, T**2, T**3, T**4, T**5],
        [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
        [0, 0, 2, 6*T, 12*T**2, 20*T**3],
    ])
    b = np.array([q0, v0, a0, qf, vf, af])
    return np.linalg.solve(A, b)


def quintic_trajectory(q_start, q_end, duration, n_steps=100):
    """
    Generate a smooth multi-joint trajectory between two joint configurations.

    Parameters
    ----------
    q_start, q_end : array-like, shape (n_joints,)
    duration : float (seconds)
    n_steps : int, number of samples along the trajectory

    Returns
    -------
    t       : (n_steps,) time vector
    Q       : (n_steps, n_joints) joint positions
    Qd      : (n_steps, n_joints) joint velocities
    Qdd     : (n_steps, n_joints) joint accelerations
    """
    q_start = np.asarray(q_start, dtype=float)
    q_end = np.asarray(q_end, dtype=float)
    n_joints = q_start.shape[0]

    t = np.linspace(0, duration, n_steps)
    Q = np.zeros((n_steps, n_joints))
    Qd = np.zeros((n_steps, n_joints))
    Qdd = np.zeros((n_steps, n_joints))

    for j in range(n_joints):
        c = quintic_coefficients(q_start[j], q_end[j], duration)
        Q[:, j] = c[0] + c[1]*t + c[2]*t**2 + c[3]*t**3 + c[4]*t**4 + c[5]*t**5
        Qd[:, j] = c[1] + 2*c[2]*t + 3*c[3]*t**2 + 4*c[4]*t**3 + 5*c[5]*t**4
        Qdd[:, j] = 2*c[2] + 6*c[3]*t + 12*c[4]*t**2 + 20*c[5]*t**3

    return t, Q, Qd, Qdd


def multi_waypoint_trajectory(waypoints, seg_duration=2.0, n_steps_per_seg=50):
    """
    Chain several quintic segments together to pass through a list of
    joint-space waypoints (e.g. for a pick-and-place cycle:
    home -> approach -> pick -> lift -> place -> home).

    Parameters
    ----------
    waypoints : list of array-like, each shape (n_joints,)
    seg_duration : float, seconds per segment
    n_steps_per_seg : int

    Returns
    -------
    t_full  : (N,) concatenated time vector
    Q_full  : (N, n_joints) concatenated joint positions
    """
    all_t, all_Q = [], []
    t_offset = 0.0
    for i in range(len(waypoints) - 1):
        t, Q, _, _ = quintic_trajectory(waypoints[i], waypoints[i + 1],
                                         seg_duration, n_steps_per_seg)
        all_t.append(t + t_offset)
        all_Q.append(Q)
        t_offset += seg_duration

    return np.concatenate(all_t), np.concatenate(all_Q, axis=0)
