# Hopper Control
## Tasks
### A. Euler-Lagrange
Please refer to [this](/bootcamp_scripts/1_hopper_dynamics/README.md#a-euler-lagrange)

### B. Hopper Control
- Learn $\texttt{Raibert Control}$ for single legged hopper.
  
    $\phi = sin^{-1}(\frac{\dot{x_0}\pi}{2l} \sqrt{\frac{m}{k}}) + k_p \cdot (\dot{x_0} - \dot{x_d})$
- Add the control law with [Raibert Control](/bootcamp_scripts/2_hopper_control/b_hopper-control/raibert_hopper.py).
- Observe the dynamics via [integration](/dynamics_bootcamp.py) (RK4 or Euler forward).