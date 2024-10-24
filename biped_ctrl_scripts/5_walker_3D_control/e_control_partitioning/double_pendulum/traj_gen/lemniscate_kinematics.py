import sympy as sp, numpy as np
import matplotlib.pyplot as plt

import sympy as sp

# Define symbols
r_x0, r_y0 = sp.symbols('r_x0, r_y0', real=True)
A, B = sp.symbols('A, B', real=True)
a, b = sp.symbols('a, b', real=True)
t, tf = sp.symbols('t, tf', real=True)

# Define phi
phi = 2 * sp.pi * (-15 * (t/tf)**4 + 6 * (t/tf)**5 + 10 * (t/tf)**3)
phidot = phi.diff(t)
phiddot = phidot.diff(t)

# Define r_x and r_y
r_x = r_x0 + A * sp.sin(a * phi)
r_y = r_y0 + B * sp.cos(b * phi)

# Compute first derivatives (xdot and ydot)
r_xdot = A*a*sp.cos(a*phi)*phidot  
r_ydot = -B*b*sp.sin(b*phi)*phidot


r_xddot = -A*a*a*sp.sin(a*phi)*phidot+A*a*sp.cos(a*phi)*phiddot
r_yddot = -B*b*b*sp.sin(b*phi)*phidot-B*b*sp.sin(b*phi)*phiddot;

# Compute second derivatives (xddot and yddot)
# r_xddot = r_xdot.diff(t)
# r_xddot_chainrule = (-A * (2 * sp.pi * a)**2 * phidot**2 * sp.sin(a * phi) + A * 2 * sp.pi * a * phiddot * sp.cos(a * phi))
# r_yddot = r_ydot.diff(t)
# r_yddot_chainrule = (-B * (2 * sp.pi * b)**2 * phidot**2 * sp.cos(b * phi) - B * 2 * sp.pi * b * phiddot * sp.sin(b * phi))


# Display the results
# r_xdot, r_ydot, r_xddot, r_yddot


print()
print("r_x = ", r_x)
print("r_y = ", r_y)
print()
print("r_xdot = ", r_xdot)
print("r_ydot = ", r_ydot)
print()
print("r_xddot = ", r_xddot)
print("r_yddot = ", r_yddot)
print()
print('REF(CARTESIAN) ACQUIRED')


