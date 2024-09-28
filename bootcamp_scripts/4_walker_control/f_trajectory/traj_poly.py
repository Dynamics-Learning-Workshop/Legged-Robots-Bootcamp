import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

from sklearn.linear_model import LinearRegression

# print('lala')
# exit()




def cubicpolytraj(q_pts, t_pts, t):
    # Cubic spline interpolation for trajectory generation
    cs = CubicSpline(t_pts, q_pts, bc_type=((1, 0.0), (1, 0.0)))  # Constrain velocity at endpoints
    q = cs(t)  # Position
    qdot = cs(t, 1)  # Velocity (first derivative)
    qddot = cs(t, 2)  # Acceleration (second derivative)
    return q, qdot, qddot, cs

def quinticpolytraj(q_pts, t_pts, t):
    # Generate a quintic polynomial trajectory (manually solving as scipy doesn't provide quintic splines)
    # Construct the coefficients of the quintic polynomials for each segment
    raise NotImplementedError("Quintic polynomial not implemented yet.")

def traj_polytraj():
    flag = 1  # input 1 for cubicpolytraj or other numeric for quinticpolytraj

    if flag == 1:
        string = 'Using cubicpolytraj'
    else:
        string = 'Using quinticpolytraj'

    t0 = 0
    tf = 8
    q_pts = [1, 5, 6, 3, 2, 7]  # Waypoints
    t_pts = np.linspace(t0, tf, len(q_pts))  # Time points corresponding to waypoints

    t = np.linspace(t0, tf, 10 * len(q_pts))  # Fine time vector for plotting

    if flag == 1:
        q, qdot, qddot, _ = cubicpolytraj(q_pts, t_pts, t)
    else:
        q, qdot, qddot, _ = quinticpolytraj(q_pts, t_pts, t)
    
    print(cubicpolytraj)

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

    fig.suptitle(string, fontsize=14)
    plt.tight_layout()
    plt.show()

# Run the trajectory plotting
traj_polytraj()
