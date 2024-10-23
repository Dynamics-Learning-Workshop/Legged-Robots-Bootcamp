import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_0.h':
    void autofunc(double roll, double pitch, double yaw, double roll_rh, double pitch_rh, double yaw_rh, double pitch_rk, double w, double l1, double l2, double *out_4488021216560093543)

def autofunc_c(double roll, double pitch, double yaw, double roll_rh, double pitch_rh, double yaw_rh, double pitch_rk, double w, double l1, double l2):

    cdef np.ndarray[np.double_t, ndim=2] out_4488021216560093543 = np.empty((3,14))
    autofunc(roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l1, l2, <double*> out_4488021216560093543.data)
    return out_4488021216560093543