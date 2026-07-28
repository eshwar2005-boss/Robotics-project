"""
visualize.py
------------
3D visualization utilities for the robot arm: static pose plots and
animated GIFs of a trajectory being executed.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless backend, safe for servers / CI
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def plot_pose(arm, joint_angles, title="Robot Arm Pose", save_path=None, ax=None):
    """Plot a single static pose of the arm in 3D."""
    positions = arm.get_joint_positions(joint_angles)

    created_fig = False
    if ax is None:
        fig = plt.figure(figsize=(7, 7))
        ax = fig.add_subplot(111, projection="3d")
        created_fig = True

    ax.clear()
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
            "-o", color="tab:blue", linewidth=3, markersize=6, label="Links")
    ax.scatter(*positions[0], color="black", s=60, label="Base")
    ax.scatter(*positions[-1], color="red", s=80, label="End-Effector")

    lim = 500
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(0, lim * 1.4)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title(title)
    ax.legend(loc="upper left")

    if created_fig and save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Saved: {save_path}")
    return ax


def animate_trajectory(arm, Q, save_path="trajectory.gif", fps=20, title="Robot Arm Trajectory"):
    """
    Animate the arm moving through a sequence of joint configurations Q
    (shape: n_steps x n_joints) and save it as a GIF.
    """
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")

    def update(frame_idx):
        plot_pose(arm, Q[frame_idx], title=f"{title} (frame {frame_idx+1}/{len(Q)})", ax=ax)
        return ax,

    anim = FuncAnimation(fig, update, frames=len(Q), interval=1000 / fps)
    anim.save(save_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    print(f"Saved animation: {save_path}")


def plot_joint_profiles(t, Q, Qd=None, Qdd=None, save_path=None):
    """Plot joint position (and optionally velocity/acceleration) vs time."""
    n_plots = 1 + (Qd is not None) + (Qdd is not None)
    fig, axes = plt.subplots(n_plots, 1, figsize=(9, 3 * n_plots), sharex=True)
    if n_plots == 1:
        axes = [axes]

    n_joints = Q.shape[1]
    labels = [f"J{i+1}" for i in range(n_joints)]

    idx = 0
    axes[idx].plot(t, np.degrees(Q), linewidth=1.5)
    axes[idx].set_ylabel("Position (deg)")
    axes[idx].legend(labels, loc="upper right", ncol=3, fontsize=8)
    axes[idx].set_title("Joint-Space Trajectory Profiles")
    idx += 1

    if Qd is not None:
        axes[idx].plot(t, np.degrees(Qd), linewidth=1.5)
        axes[idx].set_ylabel("Velocity (deg/s)")
        idx += 1

    if Qdd is not None:
        axes[idx].plot(t, np.degrees(Qdd), linewidth=1.5)
        axes[idx].set_ylabel("Accel (deg/s^2)")

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Saved: {save_path}")
    plt.close(fig)
