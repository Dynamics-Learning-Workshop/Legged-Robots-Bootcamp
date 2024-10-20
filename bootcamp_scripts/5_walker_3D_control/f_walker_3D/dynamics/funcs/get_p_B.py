import numpy as np
from sympy import *
# AUTO-GENERATE @ 2024-10-20 19:26:07

def get_p_B(x, y, z, roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l0, l1, l2):

    p_B = np.array([x, y, z])  

    return p_B