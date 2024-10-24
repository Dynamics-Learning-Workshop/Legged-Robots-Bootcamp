# This script shows the difference in computation time when using python and cython

import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import importlib
import time as time
import pickle
from sympy.utilities.autowrap import autowrap
import sympy as sp


folder_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../funcs'))
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
g=9.8
mb=70
mt=30
mc=20
Ibx=1 
Iby=1 
Ibz=1 
Itx=1 
Ity=1
Itz=1
Icx=1
Icy=1 
Icz=1

q = np.zeros(14)
qdot = np.zeros(14)
qddot = np.zeros(14)
param = np.array([w, l0, l1, l2])
param_more = np.array([g, mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz])

x,y,z,roll,pitch,yaw, roll_lh,pitch_lh,yaw_lh,pitch_lk, roll_rh,pitch_rh,yaw_rh,pitch_rk = q

dx,dy,dz,droll,dpitch,dyaw, droll_lh,dpitch_lh,dyaw_lh,dpitch_lk, droll_rh,dpitch_rh,dyaw_rh,dpitch_rk = qdot

ddx,ddy,ddz,ddroll,ddpitch,ddyaw, ddroll_lh,ddpitch_lh,ddyaw_lh,ddpitch_lk, ddroll_rh,ddpitch_rh,ddyaw_rh,ddpitch_rk = qddot

t_now = time.time()
get_J_l_ss(*q, *param)
get_J_r_ss(*q, *param)
get_Jdot_l_ss(*q, *qdot, *param)
get_Jdot_r_ss(*q, *qdot, *param)

t_py = time.time() - t_now

print(t_py)
