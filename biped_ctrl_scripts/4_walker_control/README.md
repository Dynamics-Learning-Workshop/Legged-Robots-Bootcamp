# Passive Walker
## Tasks
### A. Forward Kinematics
Robotics 101 revision at [here](/biped_ctrl_scripts/3_passive_walker/a_forward_kinematics/forward_kinematics.py), where a robot arm is written.

### B. Euler-Lagrange
Please refer to [this](/biped_ctrl_scripts/1_hopper_dynamics/README.md#a-euler-lagrange).

### C. Jacobians
- Derive Jacobians with $\texttt{sympy}$.
- Understand the mathematical meaning of Jacobians.
- Scripts are as shown [here](/biped_ctrl_scripts/4_walker_control/c_jacobians/).

### D. Hopper
Please refer to [this](/biped_ctrl_scripts/1_hopper_dynamics/README.md#c-hopper).

### E. Walker
Please refer to [this](/biped_ctrl_scripts/3_passive_walker/d_walker/passive_walker.py).

### F. Trajectory
- Derive the linear system to get the coefficient of the polynomials.
- Refer to the scripts [here](/biped_ctrl_scripts/4_walker_control/f_trajectory/).

### G. Control Partitioning
- Here we are using one-arm & two-arm robot arms to present the control partitioning.
  
  $\textbf{M}(\boldsymbol{\theta})\ddot{\boldsymbol{\theta}} + \textbf{C}(\boldsymbol{\theta}, \dot{\boldsymbol{\theta}})\dot{\boldsymbol{\theta}}+\textbf{G}(\boldsymbol{\theta})=\boldsymbol{\tau}$
  
  let

  $\boldsymbol{\tau} = \hat{\textbf{M}}(\boldsymbol{\theta})[\ddot{\boldsymbol{\theta}}_{ref}-\textbf{K}_p (\boldsymbol{\theta} - \boldsymbol{\theta}_{ref}) - \textbf{K}_d (\dot{\boldsymbol{\theta}} - \dot{\boldsymbol{\theta}}_{ref})] + \hat{\textbf{C}}(\boldsymbol{\theta}, \dot{\boldsymbol{\theta}})\dot{\boldsymbol{\theta}}+ \hat{\textbf{G}}(\boldsymbol{\theta})$
- First derive the spring-mass damper system, with both uncontrolled and controlled [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/spring_mass_damper/).
- Then, we first attempt control partitioning on single pendulum (one-arm):
  - Derive the dynamics of a single pendulum (one-arm) [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/single_pendulum/dynamics/single_pendulum_dynamics.py).
  - Try simple PD-control of the single pendulum [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/single_pendulum/single_pendulum_pd_control.py).
  - Attempt our first control partitioning on a fictitious system [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/single_pendulum/fictitious_system_control_partitioning.py).
  - Eventually on the single pendulum [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/single_pendulum/single_pendulum_control_partitioning.py).
- Third, we apply the above to double pendulum (two-arm):
  - Derive the dynamics [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/double_pendulum/dynamics/double_pendulum_dynamics.py).
  - Attempt control partitioning on double pendulum [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/double_pendulum/double_pendulum_control_partitioning.py).
  - Generate trajectories (in joint space) for the double pendulum to track [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/double_pendulum/traj_gen/traj_1.py) and [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/double_pendulum/traj_gen/traj_2.py)
  - Attempt control partitioning on double pendulum to track the trajectory [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/double_pendulum/double_pendulum_traj_tracking.py).
  - Derive the inverse kinematics (forward kinematics refer to [this](/biped_ctrl_scripts/4_walker_control/a_forward_kinematics/)) at [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/double_pendulum/inverse_kinematics/).
  - Derive trajectories (in work space) for the double pendulum to track [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/double_pendulum/traj_gen/lemniscate.py) and [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/double_pendulum/traj_gen/lemniscate_kinematics.py).
  - Attempt control partitioning on double pendulum to track the trajectory [here](/biped_ctrl_scripts/4_walker_control/g_control_partitioning/double_pendulum/double_pendulum_cartesian_control.py).

### H. Walker Control
- Revisit walker with fixed point [here](/biped_ctrl_scripts/4_walker_control/h_walker_control/passive_walker_fixpoint.py).
- Do control partitioning [here](/biped_ctrl_scripts/4_walker_control/h_walker_control/walker_control_partitioning.py). In which, we generate the trajectory based on the derived fixed point. Also, we change the dynamics of single stance with control input. Note that the control dynamics is based on an underactuated configuration (walker is indeed underactuated!). 

### I. Foot Placement Control with Data Fitting (simplified model will lead to better results)
- Generate the control data [here](/biped_ctrl_scripts/4_walker_control/i_foot_placement/walker_data_gen.py).
- Try the following control law
  - A linear P controller with naive tuning ([here](/biped_ctrl_scripts/4_walker_control/i_foot_placement/walker_P_control.py)).
  - A lookup table (not suitable for input dimension > 2, here dim = 4).
  - A data-fitted control with PCA ([here](/biped_ctrl_scripts/4_walker_control/i_foot_placement/walker_pca_control.py) and [here](/biped_ctrl_scripts/4_walker_control/i_foot_placement/walker_data_regress_pca.py)).
  - A neural network (TBD).
  - A Gaussian process regression (TBD).
- Add the above control laws to the [Walker](/biped_ctrl_scripts/4_walker_control/i_foot_placement/walker_data_regress_pca.py).
- Observe the dynamics via [integration](/dynamics_bootcamp.py) (RK4 or Euler forward).
