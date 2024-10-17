import sympy as sp

# homogeneous
x, y, theta0, theta1 = sp.symbols('x y theta0 theta1', real=True)
z = sp.Symbol('z', real=True)
l, c = sp.symbols('l, c', real=True)

R_B1_2_I = sp.Matrix([[sp.cos(sp.pi/2+theta0), -sp.sin(sp.pi/2 + theta0)],
                 [sp.sin(sp.pi/2 + theta0), sp.cos(sp.pi/2 + theta0)]])
R_B2_2_B1 = sp.Matrix([[sp.cos(-sp.pi + theta1), -sp.sin(-sp.pi + theta1)],
                 [sp.sin(-sp.pi + theta1), sp.cos(-sp.pi + theta1)]])

C1_I = sp.Matrix([x, y]) # contact point in ground frame {G}
H_B1 = sp.Matrix([l, 0]) # hip point in body frame 1 {B1}

T_B1_2_I = sp.Matrix([[R_B1_2_I, C1_I], [0, 0, 1]])
T_B2_2_B1 = sp.Matrix([[R_B2_2_B1, H_B1], [0, 0, 1]])

# Position of hip, CoM of legs, and foot in {G}
H_I = T_B1_2_I @ sp.Matrix([l, 0, 1]) # hip

G1_I = T_B1_2_I @ sp.Matrix([l - c, 0, 1]) # CoM of leg 1
G1_I = sp.simplify(G1_I)

G2_I = T_B1_2_I @ T_B2_2_B1 @ sp.Matrix([c, 0, 1]) # CoM of leg 2
G2_I = sp.simplify(G2_I)

C2_I = T_B1_2_I @ T_B2_2_B1 @ sp.Matrix([l, 0, 1]) # foot of air leg


C1_I = sp.simplify(C1_I)
C2_I = sp.simplify(C2_I)
H_I = sp.simplify(H_I)
G1_I = sp.simplify(G1_I)
G2_I = sp.simplify(G2_I)

print("HOMOGENEOUS")
print()
print("C1_I : ", C1_I[:2])
print("C2_I : ", C2_I[:2])
print("H_I : ", H_I[:2])
print("G1_I : ", G1_I[:2])
print("G2_I : ", G2_I[:2])
print()

print()
print()

# Zero Reference Model
phi = sp.symbols('phi', real=True)
cphi = sp.cos(phi)
sphi = sp.sin(phi)
vphi = 1-cphi
ux, uy, uz = sp.symbols('ux uy uz', real=True)
rx, ry, rz = sp.symbols('rx ry rz', real=True)

r00 = ux**2 * vphi + cphi
r01 = ux * uy * vphi - uz * sphi
r02 = ux * uz * vphi + uy * sphi
r10 = ux * uy * vphi + uz * sphi
r11 = uy**2 * vphi + cphi
r12 = uy * uz * vphi - ux * sphi
r20 = ux * uy * vphi - uy * sphi
r21 = uy * uz * vphi + ux * sphi
r22 = uz**2 * vphi + cphi
R_ZRM = sp.Matrix([[r00,r01,r02],
                   [r10,r11,r12],
                   [r20,r21,r22]])
I = sp.Matrix([[1,0,0],
               [0,1,0],
               [0,0,1]])
r = sp.Matrix([rx, ry, rz])

t_ZRM = sp.simplify((I - R_ZRM) @ r)
# print(t_ZRM)
# exit()

# Create the transformation matrix T
T_ZRM = sp.Matrix([[R_ZRM, t_ZRM],
                   [0,0,0,1]])


C1_I = sp.Matrix([x,y,z])
C1_I = C1_I.subs([(z,0)])
T_O_2_I = sp.Matrix([[I,C1_I],
                     [0,0,0,1]])
T_B1_2_O = sp.simplify(T_ZRM.subs([(ux,0), (uy,0), (uz,1), (phi, theta0), (rx, 0), (ry, 0), (rz, 0)]))
T_B2_2_B1 = sp.simplify(T_ZRM.subs([(ux,0), (uy,0), (uz,1), (phi, theta1), (rx, 0), (ry, l), (rz, 0)]))

T_B1_2_I = T_O_2_I @ T_B1_2_O
T_B2_2_I = T_B1_2_I @ T_B2_2_B1

C1_ZRM = sp.Matrix([0,0,0,1])
C1_I_ZRM = T_B1_2_I @ C1_ZRM

G1_ZRM = sp.Matrix([0,l-c,0,1])
G1_I_ZRM = T_B1_2_I @ G1_ZRM

H_ZRM = sp.Matrix([0,l,0,1])
H_I_ZRM = T_B1_2_I @ H_ZRM

G2_ZRM = sp.Matrix([0,l-c,0,1])
G2_I_ZRM = T_B2_2_I @ G2_ZRM

C2_ZRM = sp.Matrix([0,0,0,1])
C2_I_ZRM = T_B2_2_I @ C2_ZRM

C1_I_ZRM = sp.simplify(C1_I_ZRM)
C2_I_ZRM = sp.simplify(C2_I_ZRM)
H_I_ZRM = sp.simplify(H_I_ZRM)
G1_I_ZRM = sp.simplify(G1_I_ZRM)
G2_I_ZRM = sp.simplify(G2_I_ZRM)

print("ZERO REFERENCE MODEL")
print()
print("C1_I : ", C1_I_ZRM[:2])
print("C2_I : ", C2_I_ZRM[:2])
print("H_I : ", H_I_ZRM[:2])
print("G1_I : ", G1_I_ZRM[:2])
print("G2_I : ", G2_I_ZRM[:2])
print()




