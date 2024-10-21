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

def traj_setting(t0, tf, p0, pf, v0, vf, a0, af):
    
    A = np.array([
        [1, t0, t0**2, t0**3, t0**4, t0**5],
        [1, tf, tf**2, tf**3, tf**4, tf**5],
        [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
        [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
        [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
        ])
    b = np.array([p0, pf, v0, vf, a0, a0])
    
    return np.linalg.solve(A,b)

def get_ref(pp, tt, tf_one_step):
    if tt>tf_one_step:
        tt = tf_one_step
    
    x_ref_return = np.zeros((8,3))
    for i in range(8):
        p0 = pp[i,0]
        p1 = pp[i,1]
        p2 = pp[i,2]
        p3 = pp[i,3]
        p4 = pp[i,4]
        p5 = pp[i,5]
        
        x_ref_return[i,0] = p5*tt**5 + p4*tt**4 + p3*tt**3 + p2*tt**2 + p1*tt + p0
        x_ref_return[i,1] = 5*p5*tt**4 + 4*p4*tt**3 + 3*p3*tt**2 + 2*p2*tt + p1
        x_ref_return[i,2] = 20*p5*tt**3 + 12*p4*tt**2 + 6*p3*tt + 2*p2
        
        x_ref_return[i,0]

    print(x_ref_return.shape)
    
    return x_ref_return

def f_single_stance(x,u,param_kine, param_dyna, which_leg):
    q = x[0:14]
    qdot = x[14:28]
    
    x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh,pitch_rk = q
    dx,dy,dz,droll,dpitch,dyaw, droll_lh,dpitch_lh,dyaw_lh,dpitch_lk, droll_rh,dpitch_rh,dyaw_rh,dpitch_rk = qdot
    
    ddx,ddy,ddz,ddroll,ddpitch,ddyaw, ddroll_lh,ddpitch_lh,ddyaw_lh,ddpitch_lk, ddroll_rh,ddpitch_rh,ddyaw_rh,ddpitch_rk = np.zeros(14)
    
    w, l0, l1, l2 = param_kine
    g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz = param_dyna
    
    mass_matrix_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l0, l1, l2, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]
    M_mat = get_Mass_matrix_cy(*mass_matrix_argu)
    
    N_matrix_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, dx, dy, dz, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk, ddx, ddy, ddz, ddroll, ddpitch, ddyaw, ddroll_lh, ddpitch_lh, ddyaw_lh, ddpitch_lk, ddroll_rh, ddpitch_rh, ddyaw_rh, ddpitch_rk, w, l0, l1, l2, g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]
    N_vec = get_B_matrix_cy(*N_matrix_argu)
    
    B_ctrl_mat = np.array([
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0],
        
        [1,0,0,0,0,0,0,0],
        [0,1,0,0,0,0,0,0],
        [0,0,1,0,0,0,0,0],
        [0,0,0,1,0,0,0,0],
        [0,0,0,0,1,0,0,0],
        [0,0,0,0,0,1,0,0],
        [0,0,0,0,0,0,1,0],
        [0,0,0,0,0,0,0,1],
        
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0]
    ])
    
    if which_leg == 'l':
        J_l_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, w, l1, l2]
        Jdot_l_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, w, l1, l2]
        
        J_l_ss = get_J_l_ss_cy(*J_l_argu)
        Jdot_l_ss = get_Jdot_l_ss_cy(*Jdot_l_argu)
        
        AA = np.zeros((17,17))
        bb = np.zeros((17,1))
        
        AA[0:14, 0:14] = M_mat
        AA[0:14, 14:17] = -J_l_ss.transpose()
        AA[14:17, 0:14] = -J_l_ss
        
        bb[0:14] = N_vec
        bb[14:17] = -np.array([Jdot_l_ss @ qdot]).transpose() 
        bb = bb + B_ctrl_mat @ u
        
        x_new = np.linalg.solve(AA,bb)
        
    elif which_leg == 'r':
        J_r_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, w, l1, l2]
        Jdot_r_argu = [roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_rk, droll, dpitch, dyaw, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk, w, l1, l2]    
        J_r_ss = get_J_r_ss_cy(*J_r_argu)
        Jdot_r_ss = get_Jdot_r_ss_cy(*Jdot_r_argu)
        
        AA = np.zeros((17,17))
        bb = np.zeros((17,1))
        
        AA[0:14, 0:14] = M_mat
        AA[0:14, 14:17] = -J_r_ss.transpose()
        AA[14:17, 0:14] = -J_r_ss
        
        bb[0:14] = N_vec
        bb[14:17] = -np.array([Jdot_r_ss @ qdot]).transpose()
        bb = bb + B_ctrl_mat @ u
        
        x_new = np.linalg.solve(AA,bb)
    else:   
        print("CHECK WHICH_LEG")
        exit()
        
    xdot = np.array([*qdot, *x_new[0:14].flatten()])
    
    return xdot

def gen_control_partitioning(x, traj_coeff, t_now, tf_one_step, param_kine, param_dyna, which_leg):
    q = x[0:14]
    qdot = x[14:28]
    
    x_ref = get_ref(pp=traj_coeff, tt=t_now, tf_one_step=tf_one_step)
    s_ref = x_ref[:,0]
    v_ref = x_ref[:,1]
    a_ref = x_ref[:,2]
    
    x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh,pitch_rk = q
    dx,dy,dz,droll,dpitch,dyaw, droll_lh,dpitch_lh,dyaw_lh,dpitch_lk, droll_rh,dpitch_rh,dyaw_rh,dpitch_rk = qdot
    
    ddx,ddy,ddz,ddroll,ddpitch,ddyaw, ddroll_lh,ddpitch_lh,ddyaw_lh,ddpitch_lk, ddroll_rh,ddpitch_rh,ddyaw_rh,ddpitch_rk = np.zeros(14)
    
    w, l0, l1, l2 = param_kine
    g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz = param_dyna
    
    mass_matrix_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, w, l0, l1, l2, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]
    M = get_Mass_matrix_cy(*mass_matrix_argu)
    
    N_matrix_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, roll_rh, pitch_rh, yaw_rh, pitch_rk, dx, dy, dz, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk, ddx, ddy, ddz, ddroll, ddpitch, ddyaw, ddroll_lh, ddpitch_lh, ddyaw_lh, ddpitch_lk, ddroll_rh, ddpitch_rh, ddyaw_rh, ddpitch_rk, w, l0, l1, l2, g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz]
    N = get_B_matrix_cy(*N_matrix_argu)
    
    B_ctrl_mat = np.zeros((17,8))
    B_ctrl_mat[6:14,0:8] = np.identity(8)
    
    S = None
    AA = np.zeros((17,17))
    bb = np.zeros((17,1))
    
    if which_leg == 'l':
        J_l_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, w, l1, l2]
        Jdot_l_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, w, l1, l2]
        
        J_l_ss = get_J_l_ss_cy(*J_l_argu)
        Jdot_l_ss = get_Jdot_l_ss_cy(*Jdot_l_argu)
    
        
        AA[0:14, 0:14] = M
        AA[0:14, 14:17] = -J_l_ss.transpose()
        AA[14:17, 0:14] = -J_l_ss
        
        bb[0:14] = N
        bb[14:17] = -np.array([Jdot_l_ss @ qdot]).transpose() 
        
        S_L = np.array([
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # roll
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # pitch
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # yaw
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], # roll_rh
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0], # pitch_rh
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0], # yaw_rh
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0], # pitch_lk
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]  # pitch_rk
        ])
        # when left leg on ground, we are desired to control roll, pitch, yaw of the revolution between torso and hip
        # and also the roll, pitch, yaw of r-hip, and pitch of l- & r- knee
        # the controller, yet, it based on the revolusion of hips and knees
        S = S_L
        
    elif which_leg == 'r':
        J_r_argu = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, w, l1, l2]
        Jdot_r_argu = [roll, pitch, yaw, roll_rh, pitch_rh, yaw_rh, pitch_rk, droll, dpitch, dyaw, droll_rh, dpitch_rh, dyaw_rh, dpitch_rk, w, l1, l2]    
        J_r_ss = get_J_r_ss_cy(*J_r_argu)
        Jdot_r_ss = get_Jdot_r_ss_cy(*Jdot_r_argu)
        
        AA = np.zeros((17,17))
        bb = np.zeros((17,1))
        
        AA[0:14, 0:14] = M
        AA[0:14, 14:17] = -J_r_ss.transpose()
        AA[14:17, 0:14] = -J_r_ss
        
        bb[0:14] = N
        bb[14:17] = -np.array([Jdot_r_ss @ qdot]).transpose()
        
        S_R = np.array([
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # roll
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # pitch
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # yaw
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # roll_rh
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], # pitch_rh
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0], # yaw_rh
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0], # pitch_lk
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]  # pitch_rk
        ])
        # when right leg on ground, we are desired to control roll, pitch, yaw of the revolution between torso and hip
        # and also the roll, pitch, yaw of l-hip, and pitch of l- & r- knee
        # the controller, yet, it based on the revolusion of hips and knees
        S = S_R
        
    else:   
        print("CHECK WHICH_LEG")
        exit()
        
    Kp = 100
    Kd = 2 * np.sqrt(Kp)
    AAinv = np.linalg.inv(AA)
    
    
    s = [roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, pitch_lk, pitch_rk]
    v = [droll, dpitch, dyaw, droll_lh, dpitch_lh, dyaw_lh, dpitch_lk, dpitch_rk]
    q_ddot_c = a_ref + Kd*(v_ref-v) + Kp*(s_ref-s);
    
    # SAinvB = S @ AAinv @ B_ctrl_mat
    # SAinvB_inv = np.linalg.inv(SAinvB)
    # tau = SAinvB_inv @ (q_ddot_c - S @ AAinv @ bb);

    A_lin = S @ AAinv @ B_ctrl_mat    
    q_ddot_c = np.array([q_ddot_c]).transpose()
    b_lin = (q_ddot_c - S @ AAinv @ bb)
    
    tau = np.linalg.solve(A_lin,b_lin)
    
    print('tau.shape')
    print(tau.flatten().shape)
    
    return tau.flatten()

# START HERE!
# 1. INITIALIZATION
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

# 2. FIND FIX POINT FOR POINCARE MAP

# 3. SET TRAJECTORY
ctrller_dof = 8
traj_coeffs = np.zeros((8,6))
tf_one_step = 1.1

for i in range(ctrller_dof):    
    traj_coeffs[i,:] = traj_setting(1,2,3,4,5,6,7,8)

print(traj_coeffs.shape)
get_ref(traj_coeffs,0,tf_one_step)

gen_control_partitioning(
    x=x,
    traj_coeff=traj_coeffs,
    t_now=0,
    tf_one_step=1.1,
    param_kine=param_kine,
    param_dyna=param_dyna,
    which_leg='r'
)
    
print("END")





