import numpy as np
from sympy import *
# AUTO-GENERATE @ 2024-10-20 13:50:04

def get_p_H(x, y, z, roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l0, l1, l2):

    p_H = np.array([l0*sin(pitch) + x, -l0*sin(roll)*cos(pitch) + y, l0*cos(pitch)*cos(roll) + z, ])  

    return p_H