import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_0.h':
    void autofunc(double roll, double pitch, double yaw, double roll_rh, double pitch_rh, double yaw_rh, double pitch_rk, double droll, double dpitch, double dyaw, double droll_rh, double dpitch_rh, double dyaw_rh, double dpitch_rk, double w, double l1, double l2, double *out_1796491314173546443)

def autofunc_c(double roll, double pitch, double yaw, double roll_rh, double pitch_rh, double yaw_rh, double pitch_rk, double droll, double dpitch, double dyaw, double droll_rh, double dpitch_rh, double dyaw_rh, double dpitch_rk, double w, double l1, double l2):

    cdef np.ndarray[np.double_t, ndim=2] out_1796491314173546443 = np.empty((3,14))
    autofunc(roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_rk, droll, dpitch, dyaw, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk, w, l1, l2, <double*> out_1796491314173546443.data)
    return out_1796491314173546443