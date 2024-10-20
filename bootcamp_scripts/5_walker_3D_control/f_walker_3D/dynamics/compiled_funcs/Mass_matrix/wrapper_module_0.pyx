import numpy as np
cimport numpy as np

cdef extern from 'wrapped_code_0.h':
    void autofunc(double roll, double pitch, double yaw, double roll_lh, double pitch_lh, double yaw_lh, double pitch_lk, double roll_rh, double pitch_rh, double yaw_rh, double pitch_rk, double w, double l0, double l1, double l2, double mb, double mt, double mc, double Ibx, double Iby, double Ibz, double Itx, double Ity, double Itz, double Icx, double Icy, double Icz, double *out_5462704511010425555)

def autofunc_c(double roll, double pitch, double yaw, double roll_lh, double pitch_lh, double yaw_lh, double pitch_lk, double roll_rh, double pitch_rh, double yaw_rh, double pitch_rk, double w, double l0, double l1, double l2, double mb, double mt, double mc, double Ibx, double Iby, double Ibz, double Itx, double Ity, double Itz, double Icx, double Icy, double Icz):

    cdef np.ndarray[np.double_t, ndim=2] out_5462704511010425555 = np.empty((14,14))
    autofunc(roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l0, l1, l2, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz, <double*> out_5462704511010425555.data)
    return out_5462704511010425555