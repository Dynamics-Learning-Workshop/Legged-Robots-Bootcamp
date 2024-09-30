import sympy as sp, numpy as np
import matplotlib.pyplot as plt

A = 0.5
B = A
a = 2
b = 1
r_x0 = 1
r_y0 = 0

t = np.arange(0,2,1e-3)
tf = 2

r_x =  A*np.sin(2*np.pi*a*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3)) + r_x0
r_y =  B*np.cos(2*np.pi*b*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3)) + r_y0
r_xdot =  2*np.pi*A*a*(30*t**4/tf**5 - 60*t**3/tf**4 + 30*t**2/tf**3)*np.cos(2*np.pi*a*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3))
r_ydot =  -2*np.pi*B*b*(30*t**4/tf**5 - 60*t**3/tf**4 + 30*t**2/tf**3)*np.sin(2*np.pi*b*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3))
r_xddot =  -4*np.pi**2*A*a**2*(30*t**4/tf**5 - 60*t**3/tf**4 + 30*t**2/tf**3)**2*np.sin(2*np.pi*a*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3)) + 2*np.pi*A*a*(120*t**3/tf**5 - 180*t**2/tf**4 + 60*t/tf**3)*np.cos(2*np.pi*a*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3))
r_yddot =  -4*np.pi**2*B*b**2*(30*t**4/tf**5 - 60*t**3/tf**4 + 30*t**2/tf**3)**2*np.cos(2*np.pi*b*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3)) - 2*np.pi*B*b*(120*t**3/tf**5 - 180*t**2/tf**4 + 60*t/tf**3)*np.sin(2*np.pi*b*(6*t**5/tf**5 - 15*t**4/tf**4 + 10*t**3/tf**3))

print(r_x[0], r_y[0])
r_ref = np.vstack([r_x, r_y, r_xdot, r_ydot, r_xddot, r_yddot])


# # Create a new figure with specified size
# plt.figure(figsize=(8, 10))  # Adjust size to accommodate both subplots

# # Subplot 1: x0 vs x1 for both Euler and RK4 methods
# plt.subplot(1, 1, 1)  # 2 rows, 1 column, 1st subplot
# plt.plot(r_x, r_y, label='lemniscate')
# plt.xlabel('r_x0')
# plt.ylabel('r_y0')
# plt.title('r_x0 vs r_y0')
# plt.grid(True)
# plt.legend()

# plt.tight_layout()
# plt.show()