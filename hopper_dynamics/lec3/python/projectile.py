import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Add the path to the upper two-level folder to the system path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

from integrator import Integrator


x0 = 0
x1 = 0
dx0 = 100
dx1 = 0

xstart = np.array([x0, x1, dx0, dx1])

c = 0.5  
m = 1
g = 9.81

t_start = 0
t_end = 5
t_step = 0.001

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

x_euler = xstart
x_rk4 = xstart
for t_ in t:
    x_new_euler = Integrator().euler_forward(func=f,x=x_euler, h=t_step)
    x_new_rk4 = Integrator().rk4(func=f,x=x_rk4, h=t_step)
    
    x0_all_euler.append(x_new_euler[0])
    x1_all_euler.append(x_new_euler[1])
    dx0_all_euler.append(x_new_euler[2])
    dx1_all_euler.append(x_new_euler[3])
    
    x0_all_rk4.append(x_new_rk4[0])
    x1_all_rk4.append(x_new_rk4[1])
    dx0_all_rk4.append(x_new_rk4[2])
    dx1_all_rk4.append(x_new_rk4[3])
    
    print(x_new_rk4)
    
    x_euler = x_new_euler
    x_rk4 = x_new_rk4

# Create a new figure with specified size
plt.figure(figsize=(8, 10))  # Adjust size to accommodate both subplots

# Subplot 1: x0 vs x1 for both Euler and RK4 methods
plt.subplot(5, 1, 1)  # 2 rows, 1 column, 1st subplot
plt.plot(x0_all_euler, x1_all_euler, label='Euler Method')
plt.plot(x0_all_rk4, x1_all_rk4, label='RK4 Method')
plt.xlabel('x0')
plt.ylabel('x1')
plt.title('x0 vs x1')
plt.grid(True)
plt.legend()

# Subplot 2: t vs x0 for RK4 method
plt.subplot(5, 1, 2)  # 2 rows, 1 column, 2nd subplot
plt.plot(t, x0_all_rk4, label='RK4 Method')
plt.xlabel('t')
plt.ylabel('x0')
plt.title('t vs x0')
plt.grid(True)
plt.legend()

plt.subplot(5, 1, 3)  # 2 rows, 1 column, 2nd subplot
plt.plot(t, x1_all_rk4, label='RK4 Method')
plt.xlabel('t')
plt.ylabel('x1')
plt.title('t vs x1')
plt.grid(True)
plt.legend()

plt.subplot(5, 1, 4)  # 2 rows, 1 column, 2nd subplot
plt.plot(t, dx0_all_rk4, label='RK4 Method')
plt.xlabel('t')
plt.ylabel('dx0')
plt.title('t vs dx0')
plt.grid(True)
plt.legend()

plt.subplot(5, 1, 5)  # 2 rows, 1 column, 2nd subplot
plt.plot(t, dx1_all_rk4, label='RK4 Method')
plt.xlabel('t')
plt.ylabel('dx1')
plt.title('t vs dx1')
plt.grid(True)
plt.legend()

# Adjust layout to prevent overlap
plt.tight_layout()
plt.show()



