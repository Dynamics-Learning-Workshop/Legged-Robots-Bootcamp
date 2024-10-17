import matplotlib.pyplot as plt
import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from dynamics_bootcamp import Integrator as inte, Simulation3D as sim3D, RobotUtils as util

l = 1
# q_all
rod_start = np.array([0,0,0])

# q = [x, y, z, roll, pitch, yaw, 
#      roll_lh, pitch_lh, yaw_lh, ..., pitch_lh
#      roll_rh, pitch_rh, yaw_rh, ..., pitch_rh]

q = np.array([])

rod_start_x0_all = []
rod_start_x1_all = []
rod_start_x2_all = []
q1_all = []
q2_all = []
t_all = []

sample_factor = 10

l = 1
t_step = 1e-3

# single dot
rod_start_x0_all.append(rod_start[0])
rod_start_x1_all.append(rod_start[1])
rod_start_x2_all.append(rod_start[2])

pitch = np.deg2rad(180)
yaw = np.deg2rad(180)
q1_all.append(pitch)
q2_all.append(yaw)
t_all.append(0)


sim3D().anime(
    t=t_all[::sample_factor], 
    x_states=[
        rod_start_x0_all[::sample_factor], 
        rod_start_x1_all[::sample_factor], 
        rod_start_x2_all[::sample_factor], 
        q1_all[::sample_factor], 
        q2_all[::sample_factor]
    ], 
    ms=1000 * t_step * sample_factor,
    mission="Rotate", 
    sim_object="rod",
    sim_info={'rod_radius': 0.2, 'rod_length': l}, 
    save=True,
    save_name='rotate_3D'
)

exit()