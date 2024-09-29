
import sympy as sp

# Define symbols
m1, I1 = sp.symbols('m1 I1', real=True)  
# Mass hinge, leg, Inertia
l1 = sp.symbols('l1', real=True)  # Distances
g = sp.symbols('g', real=True)  # gravity
theta0= sp.symbols('theta0', real=True)  # Angles
omega0 = sp.symbols('omega0', real=True)  # Angular velocity
alpha0 = sp.symbols('alpha0', real=True)  # Angular acceleration

x, y = sp.symbols('x y', real=True)  # Position of the stance leg in ground frame {I}
vx, vy = sp.symbols('vx vy', real=True)  # Velocity of the stance leg in ground frame {I}
ax, ay = sp.symbols('ax ay', real=True)  # Acceleration of the stance leg in ground frame {I}

# Generalized Coordinates in ground frame {I}
q = [x, y, theta0]
qdot = [vx, vy, omega0]

# Rotation matrices
R_B1_2_I = sp.Matrix([[sp.cos(sp.pi/2 + theta0), -sp.sin(sp.pi/2 + theta0)],
                 [sp.sin(sp.pi/2 + theta0), sp.cos(sp.pi/2 + theta0)]])

CP_I = sp.Matrix([x, y]) # contact point in ground frame {I}

T_B1_2_I = sp.Matrix([[R_B1_2_I, CP_I], [0, 0, 1]])

# Position of hinge, CoM of legs, and foot in {I}
# R_H = T_B1_2_I @ sp.Matrix([l1, 0, 1]) # hinge
# x_H, y_H = R_H[0], R_H[1]

R_G1 = T_B1_2_I @ sp.Matrix([l1/2, 0, 1]) # CoM of leg 1
R_G1 = sp.simplify(R_G1)
x_G1, y_G1 = R_G1[0], R_G1[1]

v_G1 = sp.Matrix([sp.simplify(sp.Matrix([x_G1, y_G1]).jacobian(q) @ sp.Matrix(qdot))])

print("POSI IN {I} ACQUIRED")
print("VELO IN {I} ACQUIRED")

# Potential energy
# Y_H = R_H[1]
Y_G1 = R_G1[1]

print("HEIGHT IN {I} ACQUIRED")

# Kinetic and potential energy
T = 0.5 * sp.simplify(m1 * v_G1.dot(v_G1) + I1 * omega0**2)
V = sp.simplify(m1 * g * Y_G1)
L = T - V
print("LAGRANGIAN ACQUIRED")

# Derive equations of motion
qddot = [ax, ay, alpha0]
EOM = []
for i in range(3):
    dLdqdot = sp.diff(L, qdot[i])
    ddt_dLdqdot = sum([sp.diff(dLdqdot, q[j]) * qdot[j] + sp.diff(dLdqdot, qdot[j]) * qddot[j] for j in range(3)])
    # sp.diff(dLdqdot, q[j]) * qdot[j], remark -> d Blah / dt = d Blah / dq * dq / dt = d Blah / dq * qdot
    # sp.diff(dLdqdot, qdot[j]) * qddot[j], remark -> d Blah / dt = d Blah / ddq * ddq / dt = d Blah / dq * qddot
    dLdq = sp.diff(L, q[i])
    EOM.append(sp.simplify(ddt_dLdqdot - dLdq))
print("{d/dt dL/dqdot - dL/dq} ACQUIRED")

EOM_vec = sp.simplify(sp.Matrix([EOM[i] for i in range(3)]))
# print(EOM_vec)
# Compute the system's M_ss matrix and b_ss vector
M_ss = EOM_vec.jacobian(qddot)
# d/dt dL/dqdot - dL/dq = tau
# M(q) q'' + B(q,q') = tau
# use Jacobian to separate the M(q) from EOM
# The linear dependence in EoM allows the Jacobian to extract the mass matrix

b_ss = sp.simplify(EOM_vec.subs([(ax,0), (ay,0), (alpha0,0)]))
# M(q) q'' + B(q,q') = tau
# let q'' = 0, we can then get B(q,q')

print(M_ss)
print()
print(b_ss)
exit()
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
