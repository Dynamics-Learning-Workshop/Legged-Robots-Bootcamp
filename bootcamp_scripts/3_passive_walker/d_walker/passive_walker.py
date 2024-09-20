import matplotlib.pyplot as plt
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
slope_angle = 0.01

# integration environs
t_step = 1/1000
ground = 0

# initial state of q = [q0, q1, u0, u1, x0, x1]
# which are {q0, q1} = {theta_leg0, theta_leg1}
# which are {u0, u1} = {omega_leg0, omega_leg1}
# which are {x0, x1} = {xc_leg0, xc_leg1}

# fsm: single_stance, foot_strike
fsm = 'apex'

q0_all_rk4 = []
q1_all_rk4 = []
x0_all_rk4 = []
x1_all_rk4 = []

t_all = []

theta = 10

jump_i = 0
x_rk4 = xstart

event_thres = 1e-2
t = 0

sample_factor = 10

def f_single_stance(x):
    return np.array([
        x[2], 
        x[3], 
        0.0, 
        - g
        ])

def f_foot_strike(x):
    l_now = np.sqrt(
                (x_rk4[0] - xc_stance) ** 2 + x_rk4[1] ** 2
            )
            
    return np.array([
        x[2], 
        x[3], 
        k * (l - l_now) * (x[0] - xc_stance) / l_now / m, 
        k * (l - l_now) * x[1] / l_now / m - g
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
        save_name = "passive_walker_" + str(no_of_jump)
    else:
        save_name = "passive_walker_" + str(no_of_jump) + "_failed"
    
    Integrator().anime(
        t=t_all[::10], 
        x_states=[
            x0_all_rk4[::sample_factor], 
            x1_all_rk4[::sample_factor], 
            lx0_all_rk4[::sample_factor], 
            lx1_all_rk4[::sample_factor]
        ], 
        ground=ground, 
        ms=1000 * t_step * sample_factor,
        mission="Hop", 
        sim_object="hopper",
        save=False,
        save_name=save_name
    )
    exit()

while True:
    if fsm == 'apex':
        print(jump_i)
        dx0_desired = 2.0 * (x0_desired - x_rk4[0])
        if np.abs(dx0_desired) > 2.0:
            dx0_desired = dx0_desired / np.abs(dx0_desired) * 1.5
        print(dx0_desired)
        theta = np.arcsin( x_rk4[2] * np.pi / 2 / l * np.sqrt(m/k)) + Kp * (x_rk4[2] - dx0_desired)
        theta = theta / np.pi * 180
        fsm = 'flight_down'
        jump_i = jump_i + 1
        
    elif fsm == 'flight_down':
        while True:
            x_new_rk4 = Integrator().rk4(f_flight, x=x_rk4, h=t_step)
            x0_all_rk4.append(x_new_rk4[0])
            x1_all_rk4.append(x_new_rk4[1])
            lx0_all_rk4.append(x_new_rk4[0] + l * np.sin(theta / 180 * np.pi))
            lx1_all_rk4.append(x_new_rk4[1] - l * np.cos(theta / 180 * np.pi))
            dx0_all_rk4.append(x_new_rk4[2])
            dx1_all_rk4.append(x_new_rk4[3])
            
            # print(theta)
            
            t = t + t_step
            t_all.append(t)
            
            check_sys(x_new_rk4[1], x_new_rk4[1] - l * np.cos(theta / 180 * np.pi))
            x_rk4 = x_new_rk4
            
            if np.abs(x_rk4[1] - l * np.cos(theta / 180 *np.pi)) < event_thres:
                #  @ touchdown
                xc_stance = x_rk4[0] + l * np.sin(theta / 180 *np.pi)
                fsm = 'stance'
                
                break 
        
    elif fsm == 'stance':
        
        while True:
            x_new_rk4 = Integrator().rk4(f_stance, x=x_rk4, h=t_step)
            x0_all_rk4.append(x_new_rk4[0])
            x1_all_rk4.append(x_new_rk4[1])
            lx0_all_rk4.append(xc_stance)
            lx1_all_rk4.append(0)
            dx0_all_rk4.append(x_new_rk4[2])
            dx1_all_rk4.append(x_new_rk4[3])
            
            t = t + t_step
            t_all.append(t)
            
            check_sys(x_new_rk4[1],0)
            x_rk4 = x_new_rk4
            
            if np.abs(
                np.sqrt(
                    (x_rk4[0] - xc_stance) ** 2 + x_rk4[1] ** 2
                    )  - l
                ) < event_thres and x_rk4[3] > 0:
                # print()
                # print("HERE!!!")
                # print(x_rk4)
                # print(theta / np.pi * 180)
                # print(np.abs(x_rk4[0] - xc_stance))
                # print((x_rk4[1] - ground))
                # print(np.tan())
                theta = np.arctan(np.abs(x_rk4[0] - xc_stance) / np.abs(x_rk4[1] - ground))
                # print(theta)
                theta = theta / np.pi * 180
                # print(theta)
                # print()
                # print(theta / np.pi * 180)
                #  @ takeoff
                fsm = 'flight_up'
                
                break         
        
    elif fsm == 'flight_up':
        
        while True:
            x_new_rk4 = Integrator().rk4(f_flight, x=x_rk4, h=t_step)
            x0_all_rk4.append(x_new_rk4[0])
            x1_all_rk4.append(x_new_rk4[1])
            if x_new_rk4[3] > 0:
                lx0_all_rk4.append(x_new_rk4[0] - l * np.sin(theta / 180 * np.pi))
            else:
                lx0_all_rk4.append(x_new_rk4[0] + l * np.sin(theta / 180 * np.pi))
            lx1_all_rk4.append(x_new_rk4[1] - l * np.cos(theta / 180 * np.pi))
            # print("LALA")
            # print(theta / np.pi * 180)
            # print(x_new_rk4[1] - l * np.cos(theta / 180 * np.pi))
            dx0_all_rk4.append(x_new_rk4[2])
            dx1_all_rk4.append(x_new_rk4[3])
            
            t = t + t_step
            t_all.append(t)
            
            check_sys(x_new_rk4[1], x_new_rk4[1] - l * np.cos(theta / 180 * np.pi))
            x_rk4 = x_new_rk4
            
            if np.abs(x_rk4[3] - 0) < event_thres:
                #  @ apex
                fsm = 'apex'
                break 
            
    # print('end once')
    if jump_i == no_of_jump or np.abs(x_rk4[0] - x0_desired) < 10 * event_thres:
        break
    
print('SYSTEM INTEGRATION SUCCEEDED...')
draw_anime(True)

exit()