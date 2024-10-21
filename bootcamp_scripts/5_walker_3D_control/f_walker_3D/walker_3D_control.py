import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import importlib
import time as time
import pickle
from sympy.utilities.autowrap import autowrap
import sympy as sp

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './dynamics/compiled_funcs')))

# IMPORT FROM CYTHON
from B_matrix.wrapper_module_0 import autofunc_c as get_B_matrix_cy
from Mass_matrix.wrapper_module_0 import autofunc_c as get_Mass_matrix_cy
from J_l_ss.wrapper_module_0 import autofunc_c as get_J_l_ss_cy
from J_r_ss.wrapper_module_0 import autofunc_c as get_J_r_ss_cy
from Jdot_l_ss.wrapper_module_0 import autofunc_c as get_Jdot_l_ss_cy
from Jdot_r_ss.wrapper_module_0 import autofunc_c as get_Jdot_r_ss_cy

# SAVE DATA FOR VISUALIZATION
t_all = []
x0_all_rk4 = []
x1_all_rk4 = []
x2_all_rk4 = []
x3_all_rk4 = []
x4_all_rk4 = []
x5_all_rk4 = []
x6_all_rk4 = []
x7_all_rk4 = []
x8_all_rk4 = []
x9_all_rk4 = []
x10_all_rk4 = []
x11_all_rk4 = []
x12_all_rk4 = []
x13_all_rk4 = []

# FOR VISUALIZATION
def save_data(q):
    x0_all_rk4.append(q[0])
    x1_all_rk4.append(q[1])
    x2_all_rk4.append(q[2])
    x3_all_rk4.append(q[3])
    x4_all_rk4.append(q[4])
    x5_all_rk4.append(q[5])
    x6_all_rk4.append(q[6])
    x7_all_rk4.append(q[7])
    x8_all_rk4.append(q[8])
    x9_all_rk4.append(q[9])
    x10_all_rk4.append(q[10])
    x11_all_rk4.append(q[11])
    x12_all_rk4.append(q[12])
    x13_all_rk4.append(q[13])
    
    return

def f_single_stance(x,u,param_kine, param_dyna, which_leg):
    q = x[0:14]
    qdot = x[14:28]
    
    x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh,pitch_rk = q
    dx,dy,dz,droll,dpitch,dyaw, droll_lh,dpitch_lh,dyaw_lh,dpitch_lk, droll_rh,dpitch_rh,dyaw_rh,dpitch_rk = qdot
    
    ddx,ddy,ddz,ddroll,ddpitch,ddyaw, ddroll_lh,ddpitch_lh,ddyaw_lh,ddpitch_lk, ddroll_rh,ddpitch_rh,ddyaw_rh,ddpitch_rk = np.zeros(14)
    
    w, l0, l1, l2 = param_kine
    g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz = param_dyna
    
    mass_matrix_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l0, l1, l2, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]
    A = get_Mass_matrix_cy(*mass_matrix_argu)
    
    B_matrix_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, dx, dy, dz, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk, ddx, ddy, ddz, ddroll, ddpitch, ddyaw, ddroll_lh, ddpitch_lh, ddyaw_lh, ddpitch_lk, ddroll_rh, ddpitch_rh, ddyaw_rh, ddpitch_rk, w, l0, l1, l2, g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]
    b = get_B_matrix_cy(*B_matrix_argu)
    
    B = np.zeros((17,8))
    B[6:14,0:8] = np.identity(8)
    
    if which_leg == 'l':
        J_l_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, w, l1, l2]
        Jdot_l_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, w, l1, l2]
        
        J_l_ss = get_J_l_ss_cy(*J_l_argu)
        Jdot_l_ss = get_Jdot_l_ss_cy(*Jdot_l_argu)
        
        AA = np.zeros((17,17))
        bb = np.zeros((17,1))
        
        AA[0:14, 0:14] = A
        AA[0:14, 14:17] = -J_l_ss.transpose()
        AA[14:17, 0:14] = -J_l_ss
        
        bb[0:14] = b
        bb[14:17] = -np.array([Jdot_l_ss @ qdot]).transpose() 
        bb = bb + B @ u
        
        x_new = np.linalg.solve(AA,bb)
        
    elif which_leg == 'r':
        J_r_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, w, l1, l2]
        Jdot_r_argu = [roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_rk, droll, dpitch, dyaw, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk, w, l1, l2]    
        J_r_ss = get_J_r_ss_cy(*J_r_argu)
        Jdot_r_ss = get_Jdot_r_ss_cy(*Jdot_r_argu)
        
        AA = np.zeros((17,17))
        bb = np.zeros((17,1))
        
        AA[0:14, 0:14] = A
        AA[0:14, 14:17] = -J_r_ss.transpose()
        AA[14:17, 0:14] = -J_r_ss
        
        bb[0:14] = b
        bb[14:17] = -np.array([Jdot_r_ss @ qdot]).transpose() + B @ u
        bb = bb + B @ u
        
        x_new = np.linalg.solve(AA,bb)
    else:   
        print("CHECK WHICH_LEG")
        
    xdot = np.array([*qdot, *x_new[0:14].flatten()])
    
    return xdot

l0 = 1;
l1 = 0.5;
l2 = 0.5;
w = 0.1;

g = 9.8
mb = 70
mt = 10
mc = 5
Ibx = 5
Iby = 3
Ibz = 2
Itx = 1
Ity = 0.3
Itz = 2
Icx = 0.5
Icy = 0.15
Icz = 1

param_kine = [l0, l1, l2, w]
param_dyna = [g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]

which_leg = 'l' # we start with left leg

dof = 14
x = np.zeros(14*2)

# TESTING
t_now = time.time()
u = np.zeros((8,1))
f_single_stance(x, u, param_kine, param_dyna, which_leg)
t_cy = time.time() - t_now
print("COMPUTATION TIME, ", t_cy)

if t_cy > 1.0:
    print('CYTHON NOT USED, CHECK COMPILATION FILES!')
    exit()





