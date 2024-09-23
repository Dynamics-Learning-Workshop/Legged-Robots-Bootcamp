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
q0_initial = -0.2
q1_initial = -0.4
u0_initial = -0.25
u1_initial = 0.2
x0_initial = -4.0
x1_initial = 0.0
x_rk4 = np.array([q0_initial, q1_initial, u0_initial, u1_initial, x0_initial, x1_initial])

t = 0

# fsm -> single_stance, foot_strike
fsm = 'single_stance'

# states after all integration
q0_all_rk4 = []
q1_all_rk4 = []
x0_all_rk4 = []
x1_all_rk4 = []
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

def test():
    q0_all_rk4.append(x_rk4[0])
    q1_all_rk4.append(x_rk4[1])
    x0_all_rk4.append(x_rk4[4])
    x1_all_rk4.append(x_rk4[5])
    
    draw_anime(False)

def f_single_stance(x):
    return np.array([
        x[2], 
        x[3], 
        0.0, 
        - g
        ])

def f_foot_strike(x):
    pass
            
    # return np.array([
    #     x[2], 
    #     x[3], 
    #     k * (l - l_now) * (x[0] - xc_stance) / l_now / m, 
    #     k * (l - l_now) * x[1] / l_now / m - g
    #     ])

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
    
test()
exit()
while True:
    if fsm == 'single_stance':
        # integrate throughout single stance
        while True:
            x_new_rk4 = Integrator().rk4(f_single_stance, x=x_rk4, h=t_step)
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