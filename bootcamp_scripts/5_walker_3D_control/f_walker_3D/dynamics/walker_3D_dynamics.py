import sympy as sp
import time as time
import numpy as np
from sympy.utilities.autowrap import autowrap

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), './libs')))
from cython_acc_func import cython_acc_func as cy

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))
from dynamics_bootcamp import Walker3DModelling as model

def gen_func_file(expr_name, expr, var):
    file_name = 'get_'+expr_name + '.py'
    func_name = 'get_'+expr_name
    print(func_name)
    
    model().gen_func(
        file_name=file_name,
        func_name=func_name,
        arguments=(*var,),
        expression=expr,
        return_object='get_p_Hip_R_init'
    )
    
    return

def main():
    t_now = time.time()
    
    # SYMBOLS
    x, y, z = sp.symbols('x y z', real=True)
    roll, pitch, yaw = sp.symbols('roll pitch yaw',real=True)
    roll_lh, pitch_lh, yaw_lh = sp.symbols('roll_lh pitch_lh yaw_lh',real=True)
    roll_rh, pitch_rh, yaw_rh = sp.symbols('roll_rh pitch_rh yaw_rh',real=True)
    pitch_lk, pitch_rk = sp.symbols('pitch_lk pitch_rk',real=True)

    w, l0, l1, l2 = sp.symbols('w l0 l1 l2', real=True)

    dx, dy, dz = sp.symbols('dx dy dz', real=True)
    droll, dpitch, dyaw = sp.symbols('droll dpitch dyaw',real=True)
    droll_lh, dpitch_lh, dyaw_lh = sp.symbols('droll_lh dpitch_lh dyaw_lh',real=True)
    droll_rh, dpitch_rh, dyaw_rh = sp.symbols('droll_rh dpitch_rh dyaw_rh',real=True)
    dpitch_lk, dpitch_rk = sp.symbols('dpitch_lk dpitch_rk',real=True)

    ddx, ddy, ddz = sp.symbols('ddx ddy ddz', real=True)
    ddroll, ddpitch, ddyaw = sp.symbols('ddroll ddpitch ddyaw',real=True)
    ddroll_lh, ddpitch_lh, ddyaw_lh = sp.symbols('ddroll_lh ddpitch_lh ddyaw_lh',real=True)
    ddroll_rh, ddpitch_rh, ddyaw_rh = sp.symbols('ddroll_rh ddpitch_rh ddyaw_rh',real=True)
    ddpitch_lk, ddpitch_rk = sp.symbols('ddpitch_lk ddpitch_rk',real=True)
    
    g = sp.symbols('g', real=True)
    P = sp.symbols('P', real=True)
    Ibx, Iby, Ibz = sp.symbols('Ibx, Iby, Ibz', real=True)
    Itx, Ity, Itz = sp.symbols('Itx, Ity, Itz', real=True)
    Icx, Icy, Icz = sp.symbols('Icx, Icy, Icz', real=True)
    mb, mt, mc = sp.symbols('mb, mt, mc', real=True) 

    u = sp.symbols('u0 u1 u2', real=True)
    r = sp.symbols('r0 r1 r2', real=True)

    dof = 14 # keep in mind!
    I = sp.Matrix([[1,0,0],
                [0,1,0],
                [0,0,1]])

    # TRANSFORMATION
    hip_mid = sp.Matrix([x,y,z])
    T_O_2_I = sp.Matrix([[I,hip_mid],
                        [0,0,0,1]]) # from ZRM {O} to {I}
    R_O_2_I = I # from ZRM {O} to {I}

    # torso
    # roll -> pitch -> yaw
        # roll
    r = sp.Matrix([0,0,0])
    u = sp.Matrix([1,0,0])
    T_UR_2_O = cy().T_ZRM_F(u,r,roll) # from upperbody {U} (roll) to ZRM {O}
    R_UR_2_O = T_UR_2_O[0:2][0:2] # from upperbody {U} (roll) to ZRM {O}

        # pitch
    r = sp.Matrix([0,0,0])
    u = sp.Matrix([0,1,0])
    T_UP_2_UR = cy().T_ZRM_F(u,r,pitch) 
    # from upperbody {U} (pitch) to from upperbody {U} (roll)
    R_UP_2_UR = T_UP_2_UR[0:2,0:2] 
    # from upperbody {U} (pitch) to from upperbody {U} (roll)

        # yaw
    r = sp.Matrix([0,0,0])
    u = sp.Matrix([0,0,1])
    T_UY_2_UP = cy().T_ZRM_F(u,r,yaw) 
    # from upperbody {U} (yaw) to upperbody {U} (pitch)
    R_UY_2_UP = T_UY_2_UP[0:2,0:2] 
    # from upperbody {U} (yaw) to upperbody {U} (pitch)

    # left-side
    # yaw -> roll -> pitch
        # yaw
    r = sp.Matrix([0,w,0])
    u = sp.Matrix([0,0,1])
    T_LHY_2_UY = cy().T_ZRM_F(u,r,yaw_lh) 
    # from left-hip {LH} (yaw) to upperbody {U} (yaw)
    R_LHY_2_UY = T_LHY_2_UY[0:2,0:2] 
    # from left-hip {LH} (yaw) to upperbody {U} (yaw)

        # roll
    r = sp.Matrix([0,w,0])
    u = sp.Matrix([1,0,0])
    T_LHR_2_LHY = cy().T_ZRM_F(u,r,roll_lh) 
    # from left-hip {LH} (roll) to left-hip {LH} (yaw)
    R_LHR_2_LHY = T_LHR_2_LHY[0:2,0:2]
    # from left-hip {LH} (roll) to left-hip {LH} (yaw)

        # pitch
    r = sp.Matrix([0,w,0])
    u = sp.Matrix([0,-1,0])
    T_LHP_2_LHR = cy().T_ZRM_F(u,r,pitch_lh) 
    # from left-hip {LH} (pitch) to left-hip {LH} (roll)
    R_LHP_2_LHR = T_LHP_2_LHR[0:2,0:2]
    # from left-hip {LH} (pitch) to left-hip {LH} (roll)

        # knee
    r = sp.Matrix([0,w,-l1])
    u = sp.Matrix([0,-1,0])
    T_LK_2_LHP = cy().T_ZRM_F(u,r,pitch_lk) 
    # from left-knee {LK} to left-hip {LH} (pitch)
        

    # right-side
    # yaw -> roll -> pitch
    # yaw -> roll -> pitch
        # yaw
    r = sp.Matrix([0,-w,0])
    u = sp.Matrix([0,0,-1])
    T_RHY_2_UY = cy().T_ZRM_F(u,r,yaw_rh) 
    # from right-hip {RH} (yaw) to upperbody {U} (yaw)
    R_RHY_2_UY = T_RHY_2_UY[0:2,0:2] 
    # from right-hip {RH} (yaw) to upperbody {U} (yaw)

        # roll
    r = sp.Matrix([0,-w,0])
    u = sp.Matrix([-1,0,0])
    T_RHR_2_RHY = cy().T_ZRM_F(u,r,roll_rh) 
    # from right-hip {RH} (roll) to right-hip {RH} (yaw)
    R_RHR_2_RHY = T_RHR_2_RHY[0:2,0:2]
    # from right-hip {RH} (roll) to right-hip {RH} (yaw)

        # pitch
    r = sp.Matrix([0,-w,0])
    u = sp.Matrix([0,-1,0])
    T_RHP_2_RHR = cy().T_ZRM_F(u,r,pitch_rh) 
    # from right-hip {RH} (pitch) to right-hip {RH} (roll)
    R_RHP_2_RHR = T_RHP_2_RHR[0:2,0:2]
    # from right-hip {RH} (pitch) to right-hip {RH} (roll)

        # knee
    r = sp.Matrix([0,-w,-l1])
    u = sp.Matrix([0,-1,0])
    T_RK_2_RHP = cy().T_ZRM_F(u,r,pitch_rk) 
    # from right-knee {RK} to right-hip {RH} (pitch)


    # POSITION VECTORS
    p_B = hip_mid # position of base

    T_UY_2_I = T_O_2_I @ T_UR_2_O @ T_UP_2_UR @ T_UY_2_UP
    # from upper body (yaw) {U} (yaw) to {I}
    p_H = T_UY_2_I @ sp.Matrix([0,0,l0,1]) 
    # position of head

    # left
    T_LHP_2_I = T_UY_2_I @ T_LHY_2_UY @ T_LHR_2_LHY @ T_LHP_2_LHR 
    # from left-hip (pitch) {LH} (yaw) to {I}
    p_LH = T_LHP_2_I @ sp.Matrix([0,w,0,1]) 
    # position of left hip

    T_LK_2_I = T_LHP_2_I @ T_LK_2_LHP
    # from left-knee (pitch) {LK} (yaw) to {I}
    p_LK = T_LK_2_I @ sp.Matrix([0,w,-l1,1])
    # position of left knee (topology after joint knee)
    p_LA = T_LK_2_I @ sp.Matrix([0,w,-l1,1])
    # position of left ankle

    # right
    T_RHP_2_I = T_UY_2_I @ T_RHY_2_UY @ T_RHR_2_RHY @ T_RHP_2_RHR 
    # from right-hip (pitch) {RH} (yaw) to {I}
    p_RH = T_RHP_2_I @ sp.Matrix([0,w,0,1]) 
    # position of right hip

    T_RK_2_I = T_RHP_2_I @ T_RK_2_RHP
    # from right-knee (pitch) {RK} (yaw) to {I}
    p_RK = T_RK_2_I @ sp.Matrix([0,w,-l1,1])
    # position of right knee (topology after joint knee)
    p_RA = T_RK_2_I @ sp.Matrix([0,w,-l1,1])
    # position of right ankle

    # center of masses
    p_Torso = T_UY_2_I @ sp.Matrix([0,0,l0/2,1]) 
    # COM of torso
    p_Thigh_L = T_LHP_2_I @ sp.Matrix([0,w,-l1/2,1]) 
    p_Calf_L = T_LK_2_I @ sp.Matrix([0,w,-(l1+l2/2),1]) 
    p_Thigh_R = T_RHP_2_I @ sp.Matrix([0,w,-l1/2,1])
    p_Calf_R = T_RK_2_I @ sp.Matrix([0,w,-(l1+l2/2),1]) 


    var = [x, y, z, roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, roll_rh, pitch_rh, yaw_rh, pitch_lk, pitch_rk, w, l0, l1, l2]
    # hip_mid @ init
    p_Hip_L_init = -p_LA # with x = 0, y = 0, z = 0
    p_Hip_R_init = -p_RA # with x = 0, y = 0, z = 0
    # hip_mid + p_LA = ankle position
    # when start, ankle position is on ground, and set this as the starting point and infer the rest
        
    
    
    t_end = time.time()
    print('END, TIME: ', t_end - t_now)


if __name__ =="__main__":
    main()