Project Title**
6-DOF Robotic Arm Kinematics, Inverse Kinematics & Trajectory Planning Simulator
Designed and implemented a full motion-control simulation pipeline for a
6-axis articulated industrial robot arm, entirely from first principles in
Python. The project models the arm using standard Denavit-Hartenberg (DH)
parameters and implements Forward Kinematics to compute end-effector pose
from joint angles, and numerical Inverse Kinematics (Damped Least Squares
Jacobian method) to solve joint angles for a desired end-effector position,
including handling of joint limits and near-singularity stability. Built a
quintic-polynomial trajectory planner that generates smooth, jerk-limited
joint-space motion between multiple waypoints, and used it to simulate a
complete pick-and-place cycle. Visualized the robot's links, joint angle/
velocity/acceleration profiles, and full motion as an animated 3D sequence
using Matplotlib. The goal was to build hands-on, verifiable understanding
of the core algorithms that run inside real industrial robot controllers.


