import sympy as sp, numpy as np
import matplotlib.pyplot as plt

al0_0, al0_1, al0_2, al0_3 = sp.symbols('al0_0, al0_1, al0_2, al0_3', real=True)
t = sp.symbols('t', real=True)

theta0 = -sp.pi/2 - 0.5
thetaF = -sp.pi/2 + 0.5

theta = al0_0 + al0_1 * t + al0_2 * t**2 + al0_3 * t**3
thetadot = theta.diff(t)
thetaddot = thetadot.diff(t)

t0 = 0
tf = 3

# 0 = Ax - b
eqn0 = theta.subs([(t,t0)]) - theta0
eqn1 = theta.subs([(t,tf)]) - thetaF
eqn2 = thetadot.subs([(t,t0)]) - 0
eqn3 = thetadot.subs([(t,tf)]) - 0

eqn = sp.Matrix([eqn0, eqn1, eqn2, eqn3])
x = sp.Matrix([al0_0, al0_1, al0_2, al0_3])

# Now get A
A = eqn.jacobian(x)

# Then get b
b = -eqn.subs([(al0_0, 0), (al0_1, 0), (al0_2, 0), (al0_3, 0)])

coeff = A**(-1) * b

print()
print(coeff)
for i in range(len(coeff)):
    print('al0_'+str(i)+" = ", coeff[i])
    
print()
t_local = sp.symbols('t_local', real=True)
print(theta.subs([(t, t_local)]))
print()
print(thetadot.subs([(t, t_local)]))
print()
print(thetaddot.subs([(t, t_local)]))