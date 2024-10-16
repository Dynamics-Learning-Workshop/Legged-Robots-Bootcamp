import matplotlib.pyplot as plt
import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from dynamics_workshop import Integrator as inte, Simulation2D as sim2D, Simulation2D as sim2D

x0 = 0
x1 = 10
dx0 = 2
dx1 = 0

xstart = np.array([x0, x1, dx0, dx1])

c = 0.02
m = 1
g = 9.81
e = 0.9

t_start = 0
t_end = 10
t_step = 1/1000

ground = 0


# x = [x0, x1, dx0, dx1]
# dx = f(x) = [dx0, dx1, ddx0, ddx1]

def f(x):
    return np.array([
        x[2], 
        x[3], 
        - c / m * np.sqrt(x[2]**2 + x[3]**2) * x[2], 
        - c / m * np.sqrt(x[2]**2 + x[3]**2) * x[3] - g
        ])

t = np.arange(t_start, t_end, t_step)

print(t.size)
x0_all_euler = []
x1_all_euler = []
dx0_all_euler = []
dx1_all_euler = []

x0_all_rk4 = []
x1_all_rk4 = []
dx0_all_rk4 = []
dx1_all_rk4 = []

event_thres = 1e-3

sample_factor = 10

x_euler = xstart
x_rk4 = xstart
for t_ in t:
    x_new_euler = inte().euler_forward(func=f,x=x_euler, h=t_step)
    x_new_rk4 = inte().rk4(func=f,x=x_rk4, h=t_step)
    
    x0_all_euler.append(x_new_euler[0])
    x1_all_euler.append(x_new_euler[1])
    dx0_all_euler.append(x_new_euler[2])
    dx1_all_euler.append(x_new_euler[3])
    
    x0_all_rk4.append(x_new_rk4[0])
    x1_all_rk4.append(x_new_rk4[1])
    dx0_all_rk4.append(x_new_rk4[2])
    dx1_all_rk4.append(x_new_rk4[3])
    
    x_euler = x_new_euler
    x_rk4 = x_new_rk4
    
    if x_euler[1] - ground < -event_thres and x_euler[3] < 0:
        x_euler[3] = (-1) * e * x_euler[3]
    if x_rk4[1] - ground < -event_thres and x_rk4[3] < 0:
        x_rk4[3] = (-1) * e * x_rk4[3]


sim2D().anime(
    t=t[::sample_factor], 
    x_states=[x0_all_rk4[::sample_factor], x1_all_rk4[::sample_factor]], 
    ms=1000 * t_step * sample_factor,
    mission="Bounce", 
    sim_object="ball",
    sim_info={'ground':ground},
    save=False,
    save_name='bounce'
)

exit()