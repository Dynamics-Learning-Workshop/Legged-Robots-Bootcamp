# T_ZRM.pyx
import sympy as sp
cimport cython

cdef class cython_acc_func:
    def __init__(self):
        pass

    @cython.boundscheck(False)
    @cython.wraparound(False)
    def T_ZRM_F(self, u_, r_, phi_):
        cdef object ux, uy, uz, rx, ry, rz
        ux, uy, uz = u_
        rx, ry, rz = r_

        # Create symbolic variables
        cphi = sp.cos(phi_)
        sphi = sp.sin(phi_)
        vphi = 1 - cphi

        # Define rotation matrix
        r00 = ux**2 * vphi + cphi
        r01 = ux * uy * vphi - uz * sphi
        r02 = ux * uz * vphi + uy * sphi
        r10 = ux * uy * vphi + uz * sphi
        r11 = uy**2 * vphi + cphi
        r12 = uy * uz * vphi - ux * sphi
        r20 = ux * uz * vphi - uy * sphi
        r21 = uy * uz * vphi + ux * sphi
        r22 = uz**2 * vphi + cphi
        R_ZRM = sp.Matrix([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])

        # Create identity matrix
        I = sp.eye(3)
        r = sp.Matrix([rx, ry, rz])
        
        # Calculate translation
        t_ZRM = (I - R_ZRM) * r
        
        # Create the transformation matrix
        T_ZRM = sp.Matrix([[R_ZRM, t_ZRM],
                            [0, 0, 0, 1]])
        
        return T_ZRM

    cdef get_EOM(self, q_, qdot_, qddot_, L_):
        cdef list EOM = []
        
        cdef int dof = len(q_)
        cdef int i, j
        cdef object q, qdot, qddot, L
        q = q_
        qdot = qdot_
        qddot = qddot_
        L = L_

        for i in range(dof):
            dLdqdot = sp.diff(L, qdot[i])
            ddt_dLdqdot = 0  # Initialize to zero
            
            # Compute ddt_dLdqdot using a sum
            for j in range(dof):
                ddt_dLdqdot += sp.diff(dLdqdot, q[j]) * qdot[j] + sp.diff(dLdqdot, qdot[j]) * qddot[j]
            
            dLdq = sp.diff(L, q[i])
            EOM.append(ddt_dLdqdot - dLdq)
        
        return EOM

    def lala(self):
        print('gan')