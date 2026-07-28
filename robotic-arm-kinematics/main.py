"""
main.py
-------
Demo: simulate a 6-DOF industrial robot arm performing a pick-and-place
cycle - the most common task profile in a real robot work-cell.

Pipeline demonstrated:
    1. Forward Kinematics       - verify a home pose
    2. Inverse Kinematics       - solve joint angles for pick/place points
    3. Trajectory Planning      - smooth quintic joint-space motion
    4. Visualization            - static pose plots + animated GIF

Run:
    python main.py
Outputs are written to the `outputs/` folder.
"""

import os
import numpy as np

from robot_arm import RobotArm
from trajectory import multi_waypoint_trajectory
from visualize import plot_pose, animate_trajectory, plot_joint_profiles

OUT_DIR = "outputs"


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    arm = RobotArm()

    # ---------------------------------------------------------------- #
    # 1. Forward Kinematics check at HOME position
    # ---------------------------------------------------------------- #
    q_home = np.zeros(6)
    T_home = arm.forward_kinematics(q_home)
    print("=== Forward Kinematics ===")
    print("Home joint angles (deg):", np.degrees(q_home))
    print("End-effector position at home (mm):", np.round(T_home[:3, 3], 2))
    plot_pose(arm, q_home, title="Home Pose", save_path=f"{OUT_DIR}/pose_home.png")

    # ---------------------------------------------------------------- #
    # 2. Inverse Kinematics for PICK and PLACE targets
    # ---------------------------------------------------------------- #
    pick_target = np.array([300.0, 200.0, 150.0])
    place_target = np.array([300.0, -250.0, 150.0])

    print("\n=== Inverse Kinematics ===")
    q_pick, ok_pick, iters_pick = arm.inverse_kinematics(pick_target, q_init=q_home)
    print(f"Pick target {pick_target} -> converged={ok_pick} in {iters_pick} iters")
    print("  Joint angles (deg):", np.round(np.degrees(q_pick), 2))
    achieved = arm.forward_kinematics(q_pick)[:3, 3]
    print("  Achieved position (mm):", np.round(achieved, 2))

    q_place, ok_place, iters_place = arm.inverse_kinematics(place_target, q_init=q_pick)
    print(f"Place target {place_target} -> converged={ok_place} in {iters_place} iters")
    print("  Joint angles (deg):", np.round(np.degrees(q_place), 2))

    plot_pose(arm, q_pick, title="Pick Pose", save_path=f"{OUT_DIR}/pose_pick.png")
    plot_pose(arm, q_place, title="Place Pose", save_path=f"{OUT_DIR}/pose_place.png")

    # ---------------------------------------------------------------- #
    # 3. Trajectory Planning: Home -> Pick -> Lift -> Place -> Home
    # ---------------------------------------------------------------- #
    q_lift = q_pick.copy()
    q_lift[1] -= np.radians(15)  # simple "lift" by adjusting shoulder joint

    waypoints = [q_home, q_pick, q_lift, q_place, q_home]
    print("\n=== Trajectory Planning ===")
    t_full, Q_full = multi_waypoint_trajectory(waypoints, seg_duration=1.5, n_steps_per_seg=40)
    print(f"Generated trajectory with {len(t_full)} samples over {t_full[-1]:.2f} s")

    plot_joint_profiles(t_full, Q_full, save_path=f"{OUT_DIR}/joint_profiles.png")

    # ---------------------------------------------------------------- #
    # 4. Animate full pick-and-place cycle
    # ---------------------------------------------------------------- #
    print("\n=== Rendering Animation (this may take a few seconds) ===")
    animate_trajectory(arm, Q_full, save_path=f"{OUT_DIR}/pick_and_place.gif",
                        fps=15, title="Pick & Place Cycle")

    print("\nAll outputs saved in the 'outputs/' folder.")


if __name__ == "__main__":
    main()
