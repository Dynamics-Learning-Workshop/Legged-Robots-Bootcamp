# Passive Walker
## Tasks
### A. Forward Kinematics
Robotics 101 revision at [here](/bootcamp_scripts/3_passive_walker/a_forward_kinematics/forward_kinematics.py), where a robot arm is written.

### B. Euler-Lagrange
Please refer to [this](/bootcamp_scripts/1_hopper_dynamics/README.md#a-euler-lagrange).

### C. Jacobians
- Derive Jacobians with $\texttt{sympy}$.
- Understand the mathematical meaning of Jacobians.
- Scripts are as shown [here](/bootcamp_scripts/4_walker_control/c_jacobians/).

### D. Hopper
Please refer to [this](/bootcamp_scripts/1_hopper_dynamics/README.md#c-hopper).

### E. Walker
Please refer to [this](/bootcamp_scripts/3_passive_walker/d_walker/passive_walker.py).

### F. Trajectory

### G. Control

### H. Walker Control

### I. Foot Placement Control
- Try the following control law
  - A linear P controller with naive tuning ([here](/bootcamp_scripts/4_walker_control/i_foot_placement/walker_P_control.py)).
  - A lookup table.
  - A linear polynomial regression.
  - A quadratic polynomial regression.
  - A neural network.
  - A Gaussian process regression.
- Add the above control laws to the [Walker](/bootcamp_scripts/4_walker_control/i_foot_placement/walker_control.py).
- Observe the dynamics via [integration](/dynamics.py) (RK4 or Euler forward).
