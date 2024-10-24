import sympy as sp, numpy as np
import matplotlib.pyplot as plt

al1_p0_0, al1_p0_1, al1_p0_2, al1_p0_3 = sp.symbols('al1_p0_0, al1_p0_1, al1_p0_2, al1_p0_3', real=True)
al1_p1_0, al1_p1_1, al1_p1_2, al1_p1_3 = sp.symbols('al1_p1_0, al1_p1_1, al1_p1_2, al1_p1_3', real=True)
t = sp.symbols('t', real=True)

theta0_p0 = 0
thetaF_p0 = -sp.pi/2 + 1.0
theta0_p1 = thetaF_p0
thetaF_p1 = 0

t0_p0 = 0
tf_p0 = 1.5
t0_p1 = 1.5
tf_p1 = 3.0

theta_p0 = al1_p0_0 + al1_p0_1 * t + al1_p0_2 * t**2 + al1_p0_3 * t**3
thetadot_p0 = theta_p0.diff(t)
thetaddot_p0 = thetadot_p0.diff(t)

theta_p1 = al1_p1_0 + al1_p1_1 * t + al1_p1_2 * t**2 + al1_p1_3 * t**3
thetadot_p1 = theta_p1.diff(t)
thetaddot_p1 = thetadot_p1.diff(t)


# 0 = Ax - b
eqn0 = theta_p0.subs([(t,t0_p0)]) - theta0_p0
eqn1 = theta_p0.subs([(t,tf_p0)]) - thetaF_p0
eqn2 = theta_p1.subs([(t,t0_p1)]) - theta0_p1
eqn3 = theta_p1.subs([(t,tf_p1)]) - thetaF_p1
eqn4 = thetadot_p0.subs([(t,t0_p0)]) - 0
eqn5 = thetadot_p1.subs([(t,tf_p1)]) - 0
eqn6 = thetadot_p0.subs([(t,tf_p0)]) - thetadot_p1.subs([(t,t0_p1)])
eqn7 = thetaddot_p0.subs([(t,tf_p0)]) - thetaddot_p1.subs([(t,t0_p1)])

eqn = sp.Matrix([eqn0, eqn1, eqn2, eqn3, eqn4, eqn5, eqn6, eqn7])
x = sp.Matrix([al1_p0_0, al1_p0_1, al1_p0_2, al1_p0_3, al1_p1_0, al1_p1_1, al1_p1_2, al1_p1_3])

# print(eqn)
# exit()

# Now get A
A = eqn.jacobian(x)

# Then get b
b = -eqn.subs([(al1_p0_0, 0), (al1_p0_1, 0), (al1_p0_2, 0), (al1_p0_3, 0), (al1_p1_0, 0), (al1_p1_1, 0), (al1_p1_2, 0), (al1_p1_3, 0)])

# print(A)
# exit()
coeff = A**(-1) * b

print()
print(coeff)
for i in range(len(coeff)):
    print('al1_p0_'+str(i)+" = ", coeff[i])


print()
t_local = sp.symbols('t_local', real=True)

print(theta_p0.subs([(t, t_local)]))
print()
print(thetadot_p0.subs([(t, t_local)]))
print()
print(thetaddot_p0.subs([(t, t_local)]))

print()
print()

print(theta_p1.subs([(t, t_local)]))
print()
print(thetadot_p1.subs([(t, t_local)]))
print()
print(thetaddot_p1.subs([(t, t_local)]))