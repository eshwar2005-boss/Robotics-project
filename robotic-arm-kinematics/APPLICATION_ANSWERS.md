# Application Form Answers — Major Project

**28. Project Title**
6-DOF Robotic Arm Kinematics, Inverse Kinematics & Trajectory Planning Simulator

**29. Project Description**
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

**30. Technologies/Tools Used**
Python 3, NumPy (linear algebra, DH transforms, Jacobian computation),
Matplotlib (3D visualization, animation), Pillow (GIF export), Git/GitHub
for version control.

**31. Your Role in the Project**
Sole developer — designed the kinematic model and DH parameter table,
derived and implemented the Forward/Inverse Kinematics and Jacobian
computations, implemented the quintic trajectory planner, built the 3D
visualization and animation pipeline, and validated the full pipeline
end-to-end with a simulated pick-and-place task.
