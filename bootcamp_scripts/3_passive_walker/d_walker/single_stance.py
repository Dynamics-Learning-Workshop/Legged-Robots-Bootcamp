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
x, y = sp.symbols('x y', real=True)  # Position of the stance leg in ground frame {G}
vx, vy = sp.symbols('vx vy', real=True)  # Velocity of the stance leg in ground frame {G}
ax, ay = sp.symbols('ax ay', real=True)  # Acceleration of the stance leg in ground frame {G}

# Generalized Coordinates in ground frame {G}
q = [x, y, theta1, theta2]
qdot = [vx, vy, omega1, omega2]

# Rotation matrices
R_B1_2_G = sp.Matrix([[sp.cos(sp.pi/2 + theta1), -sp.sin(sp.pi/2 + theta1)],
                 [sp.sin(sp.pi/2 + theta1), sp.cos(sp.pi/2 + theta1)]])
R_B2_2_B1 = sp.Matrix([[sp.cos(-sp.pi + theta2), -sp.sin(-sp.pi + theta2)],
                 [sp.sin(-sp.pi + theta2), sp.cos(-sp.pi + theta2)]])
CP_G = sp.Matrix([x, y]) # contact point in ground frame {G}
HP_B1 = sp.Matrix([l, 0]) # hip point in body frame 1 {B1}

H_B1_2_G = sp.Matrix([[R_B1_2_G, CP_G], [0, 0, 1]])
H_B2_2_B1 = sp.Matrix([[R_B2_2_B1, HP_B1], [0, 0, 1]])

# Position of hip, CoM of legs, and foot in {G}
R_H = H_B1_2_G @ sp.Matrix([l, 0, 1]) # hip
x_H, y_H = R_H[0], R_H[1]

R_G1 = H_B1_2_G @ sp.Matrix([l - c, 0, 1]) # CoM of leg 1
R_G1 = sp.simplify(R_G1)
x_G1, y_G1 = R_G1[0], R_G1[1]

R_G2 = H_B1_2_G @ H_B2_2_B1 @ sp.Matrix([c, 0, 1]) # CoM of leg 2
R_G2 = sp.simplify(R_G2)
x_G2, y_G2 = R_G2[0], R_G2[1]

R_C2 = H_B1_2_G @ H_B2_2_B1 @ sp.Matrix([l, 0, 1]) # foot of air leg
R_C2 = sp.simplify(R_C2)
x_C2, y_C2 = R_C2[0], R_C2[1]

v_H = sp.Matrix([sp.simplify(sp.Matrix([x_H, y_H]).jacobian(q) @ sp.Matrix(qdot))])
v_G1 = sp.Matrix([sp.simplify(sp.Matrix([x_G1, y_G1]).jacobian(q) @ sp.Matrix(qdot))])
v_G2 = sp.Matrix([sp.simplify(sp.Matrix([x_G2, y_G2]).jacobian(q) @ sp.Matrix(qdot))])

print("POSI IN {G} ACQUIRED")
print("VELO IN {G} ACQUIRED")

# Potential energy
R_G_2_I = sp.Matrix([[sp.cos(-gam), -sp.sin(-gam)],
               [sp.sin(-gam), sp.cos(-gam)]])
R_H = R_G_2_I @ sp.Matrix([x_H, y_H])
R_G1 = R_G_2_I @ sp.Matrix([x_G1, y_G1])
R_G2 = R_G_2_I @ sp.Matrix([x_G2, y_G2])

Y_H = R_H[1]
Y_G1 = R_G1[1]
Y_G2 = R_G2[1]

print("HEIGHT IN {I} ACQUIRED")

# Kinetic and potential energy
T = 0.5 * sp.simplify(m * (v_G1.dot(v_G1) + v_G2.dot(v_G2)) + M * v_H.dot(v_H) + I * (omega1**2 + (omega1 + omega2)**2))
V = sp.simplify(m * g * Y_G1 + m * g * Y_G2 + M * g * Y_H)
L = T - V
print("LAGRANGIAN ACQUIRED")

# Derive equations of motion
qddot = [ax, ay, alpha1, alpha2]
EOM = []
for i in range(4):
    dLdqdot = sp.diff(L, qdot[i])
    ddt_dLdqdot = sum([sp.diff(dLdqdot, q[j]) * qdot[j] + sp.diff(dLdqdot, qdot[j]) * qddot[j] for j in range(4)])
    # sp.diff(dLdqdot, q[j]) * qdot[j], remark -> d Blah / dt = d Blah / dq * dq / dt = d Blah / dq * qdot
    # sp.diff(dLdqdot, qdot[j]) * qddot[j], remark -> d Blah / dt = d Blah / ddq * ddq / dt = d Blah / dq * qddot
    dLdq = sp.diff(L, q[i])
    EOM.append(sp.simplify(ddt_dLdqdot - dLdq))
print("{d/dt dL/dqdot - dL/dq} ACQUIRED")

EOM_vec = sp.simplify(sp.Matrix([EOM[i] for i in range(4)]))
# Compute the system's M_ss matrix and b_ss vector
M_ss = EOM_vec.jacobian(qddot)
# d/dt dL/dqdot - dL/dq = tau
# M(q) q'' + B(q,q') = tau
# use Jacobian to separate the M(q) from EOM

b_ss = sp.simplify(EOM_vec.subs([(ax,0), (ay,0), (alpha1,0), (alpha2,0)]))
# M(q) q'' + B(q,q') = tau
# let q'' = 0, we can then get B(q,q')


# M(q) q'' + B(q,q') = tau
#  = M(q) q'' + B(q,q') = J^T F_c (refer to notes)
# -> we only care about the kinematics right now
# -> 3rd & 4th row
M_ss_reduced = M_ss[2:, 2:]
b_ss_reduced = b_ss[2:]

# Display results for the system
print("M_ss = ", M_ss_reduced)
print("b_ss = ", b_ss_reduced)
print("alpha = M_ss\\b_ss")




# Matrix([[1.0*M + 2.0*m, 0, -1.0*M*l*cos(theta1) + 1.0*m*(c*cos(theta1) + c*cos(theta1 + theta2) - 2*l*cos(theta1)), 1.0*c*m*cos(theta1 + theta2)], [0, 1.0*M + 2.0*m, -1.0*M*l*sin(theta1) + 1.0*m*(c*sin(theta1) + c*sin(theta1 + theta2) - 2*l*sin(theta1)), 1.0*c*m*sin(theta1 + theta2)], [-1.0*M*l*cos(theta1) + 1.0*c*m*cos(theta1) + 1.0*c*m*cos(theta1 + theta2) - 2.0*l*m*cos(theta1), -1.0*M*l*sin(theta1) + 1.0*c*m*sin(theta1) + 1.0*c*m*sin(theta1 + theta2) - 2.0*l*m*sin(theta1), 2.0*I + 1.0*M*l**2 + 2.0*c**2*m - 2.0*c*l*m*cos(theta2) - 2.0*c*l*m + 2.0*l**2*m, 1.0*I + 1.0*c**2*m - 1.0*c*l*m*cos(theta2)], [1.0*c*m*cos(theta1 + theta2), 1.0*c*m*sin(theta1 + theta2), 1.0*I + 1.0*c**2*m - 1.0*c*l*m*cos(theta2), 1.0*I + 1.0*c**2*m]])