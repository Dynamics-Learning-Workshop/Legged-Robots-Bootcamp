import sympy as sp

# Define symbols
M, m, I = sp.symbols('M m I', real=True)  # Mass Hip, leg, Inertia
c, l = sp.symbols('c l', real=True)  # Distances
gam, g = sp.symbols('gam g', real=True)  # Slope of ramp, gravity
theta1, theta2 = sp.symbols('theta1 theta2', real=True)  # Angles
omega1, omega2 = sp.symbols('omega1 omega2', real=True)  # Angular velocity
alpha1, alpha2 = sp.symbols('alpha1 alpha2', real=True)  # Angular acceleration
theta1_n, theta2_n = sp.symbols('theta1_n theta2_n', real=True)  # Angles before heelstrike
omega1_n, omega2_n = sp.symbols('omega1_n omega2_n', real=True)  # Velocities before heelstrike
x, y = sp.symbols('x y', real=True)  # Position of the stance leg
vx, vy = sp.symbols('vx vy', real=True)  # Velocity of the stance leg
ax, ay = sp.symbols('ax ay', real=True)  # Acceleration of the stance leg

# Rotation matrices
R01 = sp.Matrix([[sp.cos(sp.pi/2 + theta1), -sp.sin(sp.pi/2 + theta1)],
                 [sp.sin(sp.pi/2 + theta1), sp.cos(sp.pi/2 + theta1)]])
R12 = sp.Matrix([[sp.cos(-sp.pi + theta2), -sp.sin(-sp.pi + theta2)],
                 [sp.sin(-sp.pi + theta2), sp.cos(-sp.pi + theta2)]])
O01 = sp.Matrix([x, y])
O12 = sp.Matrix([l, 0])

H01 = sp.Matrix([[R01, O01], [0, 0, 1]])
H12 = sp.Matrix([[R12, O12], [0, 0, 1]])

# Position of hip, com, and foot
r_C1 = sp.Matrix([x, y])
R_H = H01 @ sp.Matrix([l, 0, 1])
x_H, y_H = R_H[0], R_H[1]

R_G1 = H01 @ sp.Matrix([l - c, 0, 1])
R_G1 = sp.simplify(R_G1)
x_G1, y_G1 = R_G1[0], R_G1[1]

R_G2 = H01 @ H12 @ sp.Matrix([c, 0, 1])
R_G2 = sp.simplify(R_G2)
x_G2, y_G2 = R_G2[0], R_G2[1]

R_C2 = H01 @ H12 @ sp.Matrix([l, 0, 1])
R_C2 = sp.simplify(R_C2)
x_C2, y_C2 = R_C2[0], R_C2[1]

# Velocity vectors
q = [x, y, theta1, theta2]
qdot = [vx, vy, omega1, omega2]

v_H = sp.Matrix([sp.simplify(sp.Matrix([x_H, y_H]).jacobian(q) @ sp.Matrix(qdot))])
v_G1 = sp.Matrix([sp.simplify(sp.Matrix([x_G1, y_G1]).jacobian(q) @ sp.Matrix(qdot))])
v_G2 = sp.Matrix([sp.simplify(sp.Matrix([x_G2, y_G2]).jacobian(q) @ sp.Matrix(qdot))])

print("velo end")

# Potential energy
R = sp.Matrix([[sp.cos(-gam), -sp.sin(-gam)],
               [sp.sin(-gam), sp.cos(-gam)]])
R_H = R @ sp.Matrix([x_H, y_H])
R_G1 = R @ sp.Matrix([x_G1, y_G1])
R_G2 = R @ sp.Matrix([x_G2, y_G2])

Y_H = R_H[1]
Y_G1 = R_G1[1]
Y_G2 = R_G2[1]

print("T-V HERE")
# Kinetic and potential energy
T = 0.5 * sp.simplify(m * (v_G1.dot(v_G1) + v_G2.dot(v_G2)) + M * v_H.dot(v_H) + I * (omega1**2 + (omega1 + omega2)**2))
V = sp.simplify(m * g * Y_G1 + m * g * Y_G2 + M * g * Y_H)
L = T - V


# Derive equations of motion
qddot = [ax, ay, alpha1, alpha2]
EOM = []
for i in range(4):
    print(i)
    dLdqdot = sp.diff(L, qdot[i])
    ddt_dLdqdot = sum([sp.diff(dLdqdot, q[j]) * qdot[j] + sp.diff(dLdqdot, qdot[j]) * qddot[j] for j in range(4)])
    dLdq = sp.diff(L, q[i])
    EOM.append(sp.simplify(ddt_dLdqdot - dLdq))

# Compute the system's A_ss matrix and b_ss vector
A_ss = sp.simplify(sp.Matrix([[sp.diff(EOM[i], qddot[j]) for j in range(4)] for i in range(4)]))
b_ss = -sp.Matrix([sp.simplify(EOM[i].subs(dict(zip(qddot, [0, 0, 0, 0])))) for i in range(4)])

# Floating base (only alpha1, alpha2 components)
A_ss_reduced = A_ss[2:, 2:]
b_ss_reduced = b_ss[2:]

# Display results for the system
print("A_ss = ", A_ss_reduced)
print("b_ss = ", b_ss_reduced)
print("alpha = A_ss\\b_ss")

# Heel strike computations
J_sw = sp.simplify(sp.Matrix([x_C2, y_C2]).jacobian([x, y, theta1, theta2]))
J_n_sw = sp.simplify(J_sw.subs([(theta1, theta1_n), (theta2, theta2_n)]))
A_n_hs = sp.simplify(A_ss.subs([(theta1, theta1_n), (theta2, theta2_n)]))

print("Heel strike Jacobian J = ", J_n_sw)
print("A_n_hs = ", A_n_hs)
print("DERIVATION END")
