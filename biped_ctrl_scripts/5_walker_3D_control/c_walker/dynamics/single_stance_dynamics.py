import sympy as sp
import time as time 

t_now = time.time()
# Define symbols
M, m, I = sp.symbols('M m I', real=True)  # Mass Hip, leg, Inertia
c, l = sp.symbols('c l', real=True)  # Distances
gam, g = sp.symbols('gam g', real=True)  # Slope of ramp, gravity
theta0, theta1 = sp.symbols('theta0 theta1', real=True)  # Angles
omega0, omega1 = sp.symbols('omega0 omega1', real=True)  # Angular velocity
alpha0, alpha1 = sp.symbols('alpha0 alpha1', real=True)  # Angular acceleration
theta0_n, theta1_n = sp.symbols('theta0_n theta1_n', real=True)  # Angles before heelstrike
omega0_n, omega1_n = sp.symbols('omega0_n omega1_n', real=True)  # Velocities before heelstrike
x, y = sp.symbols('x y', real=True)  # Position of the stance leg in ground frame {G}
vx, vy = sp.symbols('vx vy', real=True)  # Velocity of the stance leg in ground frame {G}
ax, ay = sp.symbols('ax ay', real=True)  # Acceleration of the stance leg in ground frame {G}

# Generalized Coordinates in ground frame {G}
q = [x, y, theta0, theta1]
qdot = [vx, vy, omega0, omega1]

# Rotation matrices
R_B1_2_G = sp.Matrix([[sp.cos(sp.pi/2 + theta0), -sp.sin(sp.pi/2 + theta0)],
                 [sp.sin(sp.pi/2 + theta0), sp.cos(sp.pi/2 + theta0)]])
R_B2_2_B1 = sp.Matrix([[sp.cos(-sp.pi + theta1), -sp.sin(-sp.pi + theta1)],
                 [sp.sin(-sp.pi + theta1), sp.cos(-sp.pi + theta1)]])
CP_G = sp.Matrix([x, y]) # contact point in ground frame {G}
HP_B1 = sp.Matrix([l, 0]) # hip point in body frame 1 {B1}

T_B1_2_G = sp.Matrix([[R_B1_2_G, CP_G], [0, 0, 1]])
T_B2_2_B1 = sp.Matrix([[R_B2_2_B1, HP_B1], [0, 0, 1]])

# Position of hip, CoM of legs, and foot in {G}
R_H = T_B1_2_G @ sp.Matrix([l, 0, 1]) # hip
x_H, y_H = R_H[0], R_H[1]

R_G1 = T_B1_2_G @ sp.Matrix([l - c, 0, 1]) # CoM of leg 1
R_G1 = sp.simplify(R_G1)
x_G1, y_G1 = R_G1[0], R_G1[1]

R_G2 = T_B1_2_G @ T_B2_2_B1 @ sp.Matrix([c, 0, 1]) # CoM of leg 2
R_G2 = sp.simplify(R_G2)
x_G2, y_G2 = R_G2[0], R_G2[1]

R_C2 = T_B1_2_G @ T_B2_2_B1 @ sp.Matrix([l, 0, 1]) # foot of air leg
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
T = 0.5 * sp.simplify(m * (v_G1.dot(v_G1) + v_G2.dot(v_G2)) + M * v_H.dot(v_H) + I * (omega0**2 + (omega0 + omega1)**2))
V = sp.simplify(m * g * Y_G1 + m * g * Y_G2 + M * g * Y_H)
L = T - V
print("LAGRANGIAN ACQUIRED")

# Derive equations of motion
qddot = [ax, ay, alpha0, alpha1]
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
# print(EOM_vec)
# Compute the system's M_ss matrix and b_ss vector
M_ss = EOM_vec.jacobian(qddot)
# d/dt dL/dqdot - dL/dq = tau
# M(q) q'' + B(q,q') = tau
# use Jacobian to separate the M(q) from EOM

b_ss = sp.simplify(EOM_vec.subs([(ax,0), (ay,0), (alpha0,0), (alpha1,0)]))
# M(q) q'' + B(q,q') = tau
# let q'' = 0, we can then get B(q,q')


# M(q) q'' + B(q,q') = tau
#  = M(q) q'' + B(q,q') = J^T F_c (refer to notes)
# -> we only care about the kinematics right now
# -> 3rd & 4th row
M_ss_reduced = M_ss[2:, 2:]
b_ss_reduced = b_ss[2:]

print()
print("SINGLE STANCE")
print("===DERIVATION ENDED===")
print("M_ss = ", M_ss_reduced)
print()
print("M00 = ", M_ss_reduced[0,0])
print("M01 = ", M_ss_reduced[0,1])
print("M10 = ", M_ss_reduced[1,0])
print("M11 = ", M_ss_reduced[1,1])
print()
print("b_ss = ", b_ss_reduced)
print()
print("b_0 = ", b_ss_reduced[0])
print("b_1 = ", b_ss_reduced[1])
print("=====================")
t_end = time.time()
print('END, TIME: ', t_end - t_now)