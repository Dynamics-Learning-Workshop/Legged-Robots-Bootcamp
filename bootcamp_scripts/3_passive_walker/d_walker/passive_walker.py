import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from dynamics import Integrator

# basic parameters for a walker
hip_m = 1.0 # kg, mass of hip
leg_m = 0.5 # kg, mass of leg
leg_I = 0.02 # kg x m^2, moment of inertia of hip
leg_l = 1.0 # kg x m^2, length of 
leg_c = 0.5 # m, CoM of the leg
g = 1.0 # gravity
slope_angle = 20/180 * np.pi
slope_angle = 0.01

# initial states in ground plane {G}
# initial state of q = [q0, q1, u0, u1, x0, x1]
# which are {q0, q1} = {theta_leg0, theta_leg1}
# which are {u0, u1} = {omega_leg0, omega_leg1}
# which are {x0, x1} = {xc_leg0, xc_leg1}
q0_initial = 0.2
q1_initial = -0.4
u0_initial = -0.25
u1_initial = 0.2
x0_initial = -4.0
x1_initial = 0.0
x_rk4 = np.array([q0_initial, q1_initial, u0_initial, u1_initial])
x_current_stance = [x0_initial, x1_initial]

t = 0

# fsm -> single_stance, foot_strike
fsm = 'single_stance'

# states after all integration
q0_all_rk4 = []
q1_all_rk4 = []
x0_all_rk4 = []
x1_all_rk4 = []
foot_on_ground_all = [] # foot 1, foot 2
dx0_all_rk4 = []
dx1_all_rk4 = []

t_all = []

# integration environs
t_step = 1/1000
ground = 0
no_of_walk = 4
walk_i = 0
event_thres = 1e-2
sample_factor = 10


def f_single_stance(x):
    I = leg_I
    M = hip_m
    c = leg_c
    l = leg_l
    m = leg_m
    
    theta0 = x[0]
    theta1 = x[1]
    omega0 = x[2]
    omega1 = x[3]
    gam = slope_angle
    
    M00 =  2.0*I + 1.0*M*l**2 + 2.0*c**2*m - 2.0*c*l*m*np.cos(theta1) - 2.0*c*l*m + 2.0*l**2*m
    M01 =  1.0*I + 1.0*c**2*m - 1.0*c*l*m*np.cos(theta1)
    M10 =  1.0*I + 1.0*c**2*m - 1.0*c*l*m*np.cos(theta1)
    M11 =  1.0*I + 1.0*c**2*m
    
    b_0 =  1.0*M*g*l*np.sin(gam - theta0) - 1.0*c*g*m*np.sin(gam - theta0) + 1.0*c*g*m*np.sin(-gam + theta0 + theta1) + 2.0*c*l*m*omega0*omega1*np.sin(theta1) + 1.0*c*l*m*omega1**2*np.sin(theta1) + 2.0*g*l*m*np.sin(gam - theta0)
    b_1 =  1.0*c*m*(g*np.sin(-gam + theta0 + theta1) - l*omega0**2*np.sin(theta1))
    
    A = np.array([[M00, M01],[M10, M11]])
    b = np.array([-b_0, -b_1])
    x_new = np.linalg.solve(A,b)
    
    return np.array([
        omega0, 
        omega1, 
        x_new[0], 
        x_new[1] 
        ])

def f_foot_strike(x):
    I = leg_I
    M = hip_m
    c = leg_c
    l = leg_l
    m = leg_m
    
    theta0_n = x[0]
    theta1_n = x[1]
    omega0_n = x[2]
    omega1_n = x[3]
    
    M00 = 1.0*M + 2.0*m
    M01 = 0
    M02 = -1.0*M*l*np.cos(theta0_n) + 1.0*m*(c*np.cos(theta0_n) + c*np.cos(theta0_n + theta1_n) - 2*l*np.cos(theta0_n))
    M03 = 1.0*c*m*np.cos(theta0_n + theta1_n)
    M10 = 0
    M11 = 1.0*M + 2.0*m
    M12 = -1.0*M*l*np.sin(theta0_n) + 1.0*m*(c*np.sin(theta0_n) + c*np.sin(theta0_n + theta1_n) - 2*l*np.sin(theta0_n))
    M13 = 1.0*c*m*np.sin(theta0_n + theta1_n)
    M20 = -1.0*M*l*np.cos(theta0_n) + 1.0*c*m*np.cos(theta0_n) + 1.0*c*m*np.cos(theta0_n + theta1_n) - 2.0*l*m*np.cos(theta0_n)
    M21 = -1.0*M*l*np.sin(theta0_n) + 1.0*c*m*np.sin(theta0_n) + 1.0*c*m*np.sin(theta0_n + theta1_n) - 2.0*l*m*np.sin(theta0_n)
    M22 = 2.0*I + 1.0*M*l**2 + 2.0*c**2*m - 2.0*c*l*m*np.cos(theta1_n) - 2.0*c*l*m + 2.0*l**2*m
    M23 = 1.0*I + 1.0*c**2*m - 1.0*c*l*m*np.cos(theta1_n)
    M30 = 1.0*c*m*np.cos(theta0_n + theta1_n)
    M31 = 1.0*c*m*np.sin(theta0_n + theta1_n)
    M32 = 1.0*I + 1.0*c**2*m - 1.0*c*l*m*np.cos(theta1_n)
    M33 = 1.0*I + 1.0*c**2*m
    
    M_fs = np.array([[M00, M01, M02, M03], [M10, M11, M12, M13], [M20, M21, M22, M23], [M30, M31, M32, M33]])
    
    J00 = 1
    J01 = 0
    J02 = l*(-np.cos(theta0_n) + np.cos(theta0_n + theta1_n))
    J03 = l*np.cos(theta0_n + theta1_n)
    J10 = 0
    J11 = 1
    J12 = l*(-np.sin(theta0_n) + np.sin(theta0_n + theta1_n))
    J13 = l*np.sin(theta0_n + theta1_n)
    
    J_C2 = np.array([[J00, J01, J02, J03], [J10, J11, J12, J13]])

    A = np.array([[M_fs, -J_C2.transpose()],[J_C2, np.zeros([2,2])]])
    
    omega0_o = x[2]
    omega1_o = x[2]
    qdot_ = np.array([0, 0, omega0_o, omega1_o])
    b = np.array([M_fs @ qdot_,0,0])
    
    x_new = np.linalg.solve(A,b)
    
    omega0_n = x_new[2]
    omega1_n = x_new[3]
    
    # change parameter expression: leg1 -> leg2, leg2 -> leg1
    theta0 = x[0] + x[1]
    theta1 = -x[1]
    omega0 = omega0_n + omega1_n
    omega1 = -omega1_n
    
    return np.array([
        theta0, 
        theta1, 
        omega0, 
        omega1 
        ])

def check_sys(x1_body,x1_leg):
    if (x1_body - ground) < -10 * event_thres or (x1_leg - ground) < -10 * event_thres:
        print(fsm)
        print('SYSTEM FAILED...')
        print()
        
        print(x1_body - ground)
        print(x1_leg - ground)
        draw_anime(False)

def draw_anime(success):
    if success:
        print('SYSTEM INTEGRATION SUCCEEDED...')
        save_name = "passive_walker_"
    else:
        print('SYSTEM INTEGRATION FAILED...')
        save_name = "passive_walker_" + "_failed"
    
    Integrator().anime(
        t=t_all[::sample_factor], 
        x_states=[
            q0_all_rk4[::sample_factor], 
            q1_all_rk4[::sample_factor], 
            x0_all_rk4[::sample_factor], 
            x1_all_rk4[::sample_factor]
        ], 
        ms=1000 * t_step * sample_factor,
        mission="Walk", 
        sim_object="walker",
        sim_info={'ground': ground,'slope_angle':slope_angle, 'leg_l':leg_l},
        save=False,
        save_name=save_name
    )
    exit()

while True:
    if fsm == 'single_stance':
        # integrate throughout single stance
        while True:
            x_rk4_new = Integrator().rk4(f_single_stance, x=x_rk4, h=t_step)
    
            q0_all_rk4.append(x_rk4_new[0])
            q1_all_rk4.append(x_rk4_new[1])
            x0_all_rk4.append(x_current_stance[0])
            x1_all_rk4.append(x_current_stance[1])
            t = t + t_step
            t_all.append(t)

            x_rk4 = x_rk4_new
            
            if t >= 4.0:
                draw_anime(False)
        
    elif fsm == 'foot_strike':
        # bounce state
        while True:
            x_new_rk4 = Integrator().rk4(f_foot_strike, x=x_rk4, h=t_step)
            q0_all_rk4.append(x_new_rk4[0])
            q1_all_rk4.append(x_new_rk4[1])
            x0_all_rk4.append(x_new_rk4[2])
            x1_all_rk4.append(x_new_rk4[3])
            dx0_all_rk4.append(x_new_rk4[4])
            dx1_all_rk4.append(x_new_rk4[5])
            
            t = t + t_step
            t_all.append(t)
            
            if t >= 20.0:
                draw_anime(False)
            
    if walk_i == no_of_walk:
        break
    
draw_anime(True)