import sympy as sp, numpy as np

theta1, theta2 = sp.symbols('theta1, theta2', real=True)
c1, c2, l = sp.symbols('c1, c2, l', real=True)
omega1, omega2 = sp.symbols('omega1, omega2', real=True)

x_G1 = c1*sp.sin(theta1); 
y_G1 = -c1*sp.cos(theta1);
x_G2 = l*sp.sin(theta1) + c2*sp.sin(theta1+theta2); 
y_G2 = -l*sp.cos(theta1) -c2*sp.cos(theta1+theta2);

q = sp.Matrix([theta1, theta2])
J_G1 = sp.Matrix([x_G1, y_G1]).jacobian(q)
J_G2 = sp.Matrix([x_G2, y_G2]).jacobian(q)

# print(sp.simplify(J_G1))
# print(sp.simplify(J_G2))

qdot = sp.Matrix([omega1, omega2])
v_G1 = J_G1 * qdot
v_G2 = J_G2 * qdot

print()
print('EXPRESS V (POI G1, G2) W.R.T. q')
print(v_G1)
print()
print(v_G2)