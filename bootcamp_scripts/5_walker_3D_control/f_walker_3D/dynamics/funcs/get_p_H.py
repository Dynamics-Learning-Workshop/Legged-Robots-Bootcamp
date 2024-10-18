import numpy as np
from sympy import *
# AUTO-GENERATE @ 2024-10-18 17:26:29

def get_p_H(x, y, z, roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, roll_rh, pitch_rh, yaw_rh, pitch_lk, pitch_rk, w, l0, l1, l2):

    get_p_Hip_R_init = np.array([l0*sin(pitch) + x, -l0*sin(roll)*cos(pitch) + y, l0*cos(pitch)*cos(roll) + z, 1])  

    return get_p_Hip_R_init