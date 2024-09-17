import matplotlib.pyplot as plt
import sys
import os
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../')))
from integrator import Integrator

c = 0.02
m = 100
g = 9.81
e = 0.9
k = 10000
l = 1

t_start = 0
t_end = 4
t_step = 1/1000

ground = 0

x0 = 0
x1 = 1.2
dx0 = 1.0
dx1 = 0
no_jump = 5 # cannot set too high, as the current fsm switching condition is ideal

xstart = np.array([x0, x1, dx0, dx1])
xc_stance = 0

# apex, flight, touchdown, stance, takeoff
fsm = 'apex'

x0_all_rk4 = []
x1_all_rk4 = []
lx0_all_rk4 = []
lx1_all_rk4 = []
dx0_all_rk4 = []
dx1_all_rk4 = []

t_all = []

theta = 10

jump_i = 0
x_rk4 = xstart

event_thres = 1e-2
t = 0