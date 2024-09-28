import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

def cubicpolytraj(q_pts, t_pts, t):
    # Cubic spline interpolation for trajectory generation
    cs = CubicSpline(t_pts, q_pts, bc_type=((1, 0.0), (1, 0.0)))  # Constrain velocity at endpoints
    q = cs(t)  # Position
    qdot = cs(t, 1)  # Velocity (first derivative)
    qddot = cs(t, 2)  # Acceleration (second derivative)
    return q, qdot, qddot, cs

def traj_polytraj():

    t0 = 0
    tf = 8
    q_pts = [1, 5, 6, 3, 2, 7]  # Waypoints
    t_pts = np.linspace(t0, tf, len(q_pts))  # Time points corresponding to waypoints

    t = np.linspace(t0, tf, 10 * len(q_pts))  # Fine time vector for plotting

    q, qdot, qddot, _ = cubicpolytraj(q_pts, t_pts, t)

    # Plotting
    fig, axs = plt.subplots(2, 2, figsize=(10, 6))

    axs[0, 0].plot(t, q)
    axs[0, 0].set_ylabel('$q$', fontsize=12)
    axs[0, 0].set_xlabel('t', fontsize=12)

    axs[0, 1].plot(t, qdot)
    axs[0, 1].set_ylabel('$\dot{q}$', fontsize=12)
    axs[0, 1].set_xlabel('t', fontsize=12)

    axs[1, 0].plot(t, qddot)
    axs[1, 0].set_ylabel('$\ddot{q}$', fontsize=12)
    axs[1, 0].set_xlabel('t', fontsize=12)

    fig.suptitle('Cubic Spline', fontsize=14)
    plt.tight_layout()
    plt.show()

# Run the trajectory plotting
traj_polytraj()
