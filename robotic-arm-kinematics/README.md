# 6-DOF Robotic Arm — Kinematics, Inverse Kinematics & Trajectory Planning Simulator

A from-scratch Python simulation of a 6-axis articulated industrial robot arm
(kinematic structure comparable to a small industrial manipulator, e.g. a
FANUC LR Mate–class robot). The project implements the core motion-control
math that runs inside every industrial robot controller — Forward Kinematics,
Inverse Kinematics, and smooth trajectory generation — and visualizes a full
**pick-and-place cycle** in 3D.

![Pick and Place Animation](outputs/pick_and_place.gif)

## Why this project

Understanding how a robot converts joint angles into end-effector position
(and back again) is fundamental to working with industrial robots — whether
programming, integrating, or maintaining them. This project builds that
pipeline from first principles instead of relying on a black-box simulator,
so every transformation is explicit and verifiable.

## Features

| Module | Description |
|---|---|
| `robot_arm.py` | 6-DOF manipulator model using standard **Denavit-Hartenberg (DH) parameters**. Implements Forward Kinematics, the geometric **Jacobian**, and numerical **Inverse Kinematics** via the Damped Least Squares method (robust near singularities). Includes joint-limit enforcement. |
| `trajectory.py` | **Quintic (5th-order) polynomial** joint-space trajectory generation, guaranteeing continuous position, velocity, and acceleration — the same smoothness principle used in real robot motion controllers. Supports chaining multiple waypoints (e.g. home → pick → lift → place → home). |
| `visualize.py` | 3D pose plotting and animated GIF rendering of a full trajectory using `matplotlib`. |
| `main.py` | End-to-end demo: FK check → IK solve for pick/place targets → multi-waypoint trajectory → rendered animation. |

## Example output

The demo script solves inverse kinematics for a pick point and a place
point, plans a smooth 5-segment trajectory between them, and renders:

- `outputs/pose_home.png`, `pose_pick.png`, `pose_place.png` — static 3D poses
- `outputs/joint_profiles.png` — joint angle/velocity/acceleration vs. time
- `outputs/pick_and_place.gif` — full animated motion cycle

## Getting started

```bash
git clone https://github.com/<your-username>/robotic-arm-kinematics.git
cd robotic-arm-kinematics
pip install -r requirements.txt
python main.py
```

Console output looks like:

```
=== Forward Kinematics ===
Home joint angles (deg): [0. 0. 0. 0. 0. 0.]
End-effector position at home (mm): [450.  0. -70.]

=== Inverse Kinematics ===
Pick target [300. 200. 150.] -> converged=True in 5 iters
  Joint angles (deg): [ 33.62 -45.67  42.87   1.36  13.36   0.  ]
  Achieved position (mm): [300. 200. 150.]
...

=== Trajectory Planning ===
Generated trajectory with 160 samples over 6.00 s

=== Rendering Animation ===
Saved animation: outputs/pick_and_place.gif
```

## Technical highlights

- **Forward Kinematics** — chained homogeneous transforms from DH parameters
  give the end-effector pose for any joint configuration.
- **Inverse Kinematics** — Damped Least Squares (Levenberg-Marquardt style)
  Jacobian inversion converges reliably even near kinematic singularities,
  which a naive Jacobian-transpose or pure pseudo-inverse method can struggle
  with.
- **Trajectory Planning** — quintic polynomials remove velocity/acceleration
  discontinuities at waypoints, reducing mechanical jerk — directly relevant
  to protecting real actuators and gearboxes from stress.
- **Joint limits** — configurable per-axis limits are enforced during IK,
  mirroring real robot controller safety constraints.

## Possible extensions

- Add collision/workspace-boundary checking
- Swap the DH table to match a specific FANUC model's real datasheet
  parameters for a 1:1 digital twin
- Add Cartesian (straight-line) trajectory interpolation in addition to
  joint-space
- Export generated trajectories as G-code / robot teach-pendant style
  waypoint lists

## Tech stack

Python 3, NumPy, Matplotlib, Pillow

## Author

<Your Name> — built as part of a robotics internship application.
