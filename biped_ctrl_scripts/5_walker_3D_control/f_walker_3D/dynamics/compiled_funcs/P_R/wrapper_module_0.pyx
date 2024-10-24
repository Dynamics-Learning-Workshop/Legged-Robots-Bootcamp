import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_0.h':
    void autofunc(double roll, double pitch, double yaw, double roll_rh, double pitch_rh, double yaw_rh, double pitch_rk, double l1, double l2, double F, double *out_4162681260375458509)

def autofunc_c(double roll, double pitch, double yaw, double roll_rh, double pitch_rh, double yaw_rh, double pitch_rk, double l1, double l2, double F):

    cdef np.ndarray[np.double_t, ndim=2] out_4162681260375458509 = np.empty((3,1))
    autofunc(roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_rk, l1, l2, F, <double*> out_4162681260375458509.data)
    return out_4162681260375458509