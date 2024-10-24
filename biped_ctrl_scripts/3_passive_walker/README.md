# Passive Walker
## Tasks
### A. Forward Kinematics
- Robotics 101 revision at [here](/biped_ctrl_scripts/3_passive_walker/a_forward_kinematics/forward_kinematics.py), where a robot arm is written.

### B. Euler-Lagrange
Please refer to [this](/biped_ctrl_scripts/1_hopper_dynamics/README.md#a-euler-lagrange).

### C. Bounce
Please refer to [this](/biped_ctrl_scripts/1_hopper_dynamics/README.md#b-bounce).

### D. Passive Walker
- Derive it with $\texttt{sympy}$ with [this](/biped_ctrl_scripts/3_passive_walker/d_walker/dynamics/).
- Learn hybrid system and how to integrate it with $\texttt{FSM}$.
- Applied it on a [passive walker](/biped_ctrl_scripts/3_passive_walker/d_walker/passive_walker.py).
- Observe the dynamics via [integration](/dynamics_bootcamp.py) (RK4 or Euler forward).
- Solve the fixed point for the Poincare map [here](/biped_ctrl_scripts/3_passive_walker/d_walker/passive_walker_fixpoint.py).