import numpy as np
from sympy import *
# AUTO-GENERATE @ 2024-10-19 16:24:49

def get_p_Torso(x, y, z, roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, roll_rh, pitch_rh, yaw_rh, pitch_lk, pitch_rk, w, l0, l1, l2):

    p_Torso = np.array([l0*sin(pitch)/2 + x, -l0*sin(roll)*cos(pitch)/2 + y, l0*cos(pitch)*cos(roll)/2 + z, ])  

    return p_Torso