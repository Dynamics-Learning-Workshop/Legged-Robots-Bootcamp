# Hopper Dynamics
## Tasks
### A. Euler-Lagrange
- Derive the dynamic equations via getting the Lagrangian.
- Get $\textbf{M(q)}\ddot{\textbf{q}} + \textbf{B}=\boldsymbol{\tau}$ (treat Coriolis and gravitational force as a lump $\textbf{B}$).
- Learning how to use $\texttt{sympy}$ for auto differentiation.
- Applied it on a [projectile](/biped_ctrl_scripts/1_hopper_dynamics/a_euler-lagrange/projectile.py) motion and [double pendulum](/biped_ctrl_scripts/1_hopper_dynamics/a_euler-lagrange/double_pendulum.py).
- Observe the dynamics via [integration](/dynamics_bootcamp.py) (RK4 or Euler forward).

### B. Bounce
- Learn hybrid system and how to integrate it with $\texttt{FSM}$.
- Applied it on a [bouncing ball](/biped_ctrl_scripts/1_hopper_dynamics/b_bounce/bounce.py)
- Observe the dynamics via [integration](/dynamics_bootcamp.py) (RK4 or Euler forward).

### C. Hopper
- Learn hybrid system and how to integrate it with $\texttt{FSM}$.
- Applied it on a [passive hopper](/biped_ctrl_scripts/1_hopper_dynamics/c_hopper/hopper.py).
- Observe the dynamics via [integration](/dynamics_bootcamp.py) (RK4 or Euler forward).