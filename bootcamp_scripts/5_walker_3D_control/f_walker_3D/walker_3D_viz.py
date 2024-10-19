import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import importlib

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from dynamics_bootcamp import Integrator as inte, Simulation3D as sim3D, RobotUtils as util

folder_path = os.path.abspath(os.path.join(os.path.dirname(__file__), './dynamics/funcs'))
sys.path.append(folder_path)

# List all Python files in the folder
module_names = [f[:-3] for f in os.listdir(folder_path) if f.endswith('.py')]

# Dynamically import all files and functions
for module_name in module_names:
    module = importlib.import_module(module_name)
    # Import all functions from each module dynamically
    for func in dir(module):
        if callable(getattr(module, func)):
            globals()[func] = getattr(module, func)

l0 = 1;
l1 = 0.5
l2 = 0.5
w = 0.1

q = np.zeros(14)
param = np.array([w, l0, l1, l2])

x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh, roll_rh,pitch_rh,yaw_rh, pitch_lk, pitch_rk = q

# print(x, y, z, roll, pitch, yaw, roll_lh, pitch_lh, yaw_lh, roll_rh, pitch_rh, yaw_rh, pitch_lk, pitch_rk)

p_H = get_p_H(*q,*param) # head
print(p_H)
p_B = get_p_B(*q,*param)
print(p_B)

p_LH = get_p_LH(*q,*param)
print(p_LH)
p_RH = get_p_RH(*q,*param)
print(p_RH)

p_LK = get_p_LK(*q,*param)
print(p_LK)
p_RK = get_p_RK(*q,*param)
print(p_RK)

p_LA = get_p_LA(*q,*param)
print(p_LA)
p_RA = get_p_RA(*q,*param)
print(p_RA)

p_Torso = get_p_Torso(*q,*param)
print(p_Torso)

p_Thigh_L = get_p_Thigh_L(*q,*param)
print(p_Thigh_L)
p_Thigh_R = get_p_Thigh_R(*q,*param)
print(p_Thigh_R)

p_Calf_L = get_p_Calf_L(*q,*param)
print(p_Calf_L)
p_Calf_R = get_p_Calf_R(*q,*param)
print(p_Calf_R)

print("==========")
p_Hip_I = get_p_Hip_L_init(0,0,0,*q[3:],*param)
print(p_Hip_I)

p_Hip_I = get_p_Hip_R_init(0,0,0,*q[3:],*param)
print(p_Hip_I)


