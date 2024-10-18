import numpy as np
from sympy import *
# AUTO-GENERATE @ 2024-10-18 17:30:32

def get_p_B(x, y, z, roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, roll_rh, pitch_rh, yaw_rh, pitch_lk, pitch_rk, w, l0, l1, l2):

    get_p_Hip_R_init = np.array([x, y, z])  

    return get_p_Hip_R_init