import sympy as sp, numpy as np
import matplotlib.pyplot as plt

r_x0, r_y0 = sp.symbols('r_x0, r_y0', real=True)
A, B = sp.symbols('A, B', real=True)
a, b = sp.symbols('a, b', real=True)

t, tf = sp.symbols('t, tf', real=True)

phi = 2 * sp.pi * (-15 * (t/tf)**4 + 6 * (t/tf)**5 + 10 * (t/tf)**3)
phidot = phi.diff(t)
phiddot = phidot.diff(t)

r_x = r_x0 + A * sp.sin(a*phi)
r_y = r_y0 + B * sp.cos(b*phi)
r_xdot = r_x.diff(t)
r_ydot = r_y.diff(t)
r_xddot = r_xdot.diff(t)
r_yddot = r_ydot.diff(t)

print("r_x = ", r_x)
print("r_y = ", r_y)
print("r_xdot = ", r_xdot)
print("r_ydot = ", r_ydot)
print("r_xddot = ", r_xddot)
print("r_yddot = ", r_yddot)

print('REF(CARTESIAN) ACQUIRED')


