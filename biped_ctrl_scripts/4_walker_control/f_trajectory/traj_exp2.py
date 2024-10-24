import sympy as sp, numpy as np
import matplotlib.pyplot as plt

ta_0, ta_f, qa_0, qa_f = sp.symbols('ta_0 ta_f qa_0 qa_f', real=True)
tb_f, qb_f = sp.symbols('tb_f qb_f', real=True)
qadot_0, qadot_f, qbdot_f = sp.symbols('qadot_0 qadot_f qbdot_f', real=True)

# solving a 3rd-order polynomial
A = sp.Matrix([
    [1, ta_0, ta_0**2, ta_0**3, 0, 0, 0, 0],
    [1, ta_f, ta_f**2, ta_f**3, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, ta_f, ta_f**2, ta_f**3],
    [0, 0, 0, 0, 1, tb_f, tb_f**2, tb_f**3],
    [0, 1, 2*ta_0, 3*ta_0**2, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 2*tb_f, 3*tb_f**2],
    [0, 1, 2*ta_f, 3*ta_f**2, 0, -1, -2*ta_f, -3*ta_f**2],
    [0, 0, 2, 6*ta_f, 0, 0, -2, -6*ta_f]
])

b= sp.Matrix([
    qa_0, qa_f, qa_f, qb_f, qadot_0, qadot_f, qadot_f, qbdot_f
])

# print("=====A Matrix=====")
# print(A)
# print()
# for i in range(8):
#     for j in range(8):
#         print("A"+str(i)+str(j)+" = ", A[i,j])
# print("==================")
# print()
# exit()

def solve(
        ta_0_, ta_f_, tb_f_, 
        qa_0_, qa_f_, qb_f_, 
        qadot_0_, qadot_f_, qbdot_f_
    ):
    
    A00 =  1
    A01 =  ta_0_
    A02 =  ta_0_**2
    A03 =  ta_0_**3
    A04 =  0
    A05 =  0
    A06 =  0
    A07 =  0
    A10 =  1
    A11 =  ta_f_
    A12 =  ta_f_**2
    A13 =  ta_f_**3
    A14 =  0
    A15 =  0
    A16 =  0
    A17 =  0
    A20 =  0
    A21 =  0
    A22 =  0
    A23 =  0
    A24 =  1
    A25 =  ta_f_
    A26 =  ta_f_**2
    A27 =  ta_f_**3
    A30 =  0
    A31 =  0
    A32 =  0
    A33 =  0
    A34 =  1
    A35 =  tb_f_
    A36 =  tb_f_**2
    A37 =  tb_f_**3
    A40 =  0
    A41 =  1
    A42 =  2*ta_0_
    A43 =  3*ta_0_**2
    A44 =  0
    A45 =  0
    A46 =  0
    A47 =  0
    A50 =  0
    A51 =  0
    A52 =  0
    A53 =  0
    A54 =  0
    A55 =  1
    A56 =  2*tb_f_
    A57 =  3*tb_f_**2
    A60 =  0
    A61 =  1
    A62 =  2*ta_f_
    A63 =  3*ta_f_**2
    A64 =  0
    A65 =  -1
    A66 =  -2*ta_f_
    A67 =  -3*ta_f_**2
    A70 =  0
    A71 =  0
    A72 =  2
    A73 =  6*ta_f_
    A74 =  0
    A75 =  0
    A76 =  -2
    A77 =  -6*ta_f_
    
    A = np.array([
        [A00, A01, A02, A03, A04, A05, A06, A07],
        [A10, A11, A12, A13, A14, A15, A16, A17],
        [A20, A21, A22, A23, A24, A25, A26, A27],
        [A30, A31, A32, A33, A34, A35, A36, A37],
        [A40, A41, A42, A43, A44, A45, A46, A47],
        [A50, A51, A52, A53, A54, A55, A56, A57],
        [A60, A61, A62, A63, A64, A65, A66, A67],
        [A70, A71, A72, A73, A74, A75, A76, A77],
    ])
    
    print(A)
    
    b = np.array([qa_0_, qa_f_, qa_f_, qb_f_, qadot_0_, qadot_f_, 0, 0])
    
    print()
    print('hereclear')
    print(b)
    print()
    
    poly_coef = np.linalg.solve(A, b)
    
    return poly_coef

# boundary conditions here, as the variable name suggest
poly_coef = solve(ta_0_=0, ta_f_=1, tb_f_=3, qa_0_=0, qa_f_=0.5, qb_f_=1.0, qadot_0_=0, qadot_f_=0, qbdot_f_=0)

print(poly_coef)


t_step = 1e-3
ta = np.arange(0, 1, t_step)
tb = np.arange(1, 3, t_step)

aa_0 = poly_coef[0]
aa_1 = poly_coef[1]
aa_2 = poly_coef[2]
aa_3 = poly_coef[3]

ab_0 = poly_coef[4]
ab_1 = poly_coef[5]
ab_2 = poly_coef[6]
ab_3 = poly_coef[7]

q_a = aa_0 + aa_1 * ta + aa_2 * ta**2 + aa_3 * ta**3
q_b = ab_0 + ab_1 * tb + ab_2 * tb**2 + ab_3 * tb**3

q_a_dot = aa_1 + 2 * aa_2 * ta + 3 * aa_3 * ta**2
q_b_dot = ab_1 + 2 * ab_2 * tb + 3 * ab_3 * tb**2

q_a_ddot = 2 * aa_2 + 6 * aa_3 * ta
q_b_ddot = 2 * ab_2 + 6 * ab_3 * tb


# Create plots
plt.figure(figsize=(12, 8))

# Plot position
plt.subplot(3, 1, 1)
# Ensure ta, tb, q_a, q_b are 1D arrays
plt.plot(np.concatenate([ta, tb]), np.concatenate([q_a, q_b]), label='Position q(t)', color='b')

plt.title('Position, Velocity, and Acceleration vs. Time')
plt.ylabel('Position (q)')
plt.legend()
plt.grid()

# Plot velocity
plt.subplot(3, 1, 2)
plt.plot(np.concatenate([ta, tb]), np.concatenate([q_a_dot, q_b_dot]), label='Position q(t)', color='b')
plt.ylabel('Velocity (q\' )')
plt.legend()
plt.grid()

# Plot acceleration
plt.subplot(3, 1, 3)
plt.plot(np.concatenate([ta, tb]), np.concatenate([q_a_ddot, q_b_ddot]), label='Position q(t)', color='b')
plt.ylabel('Acceleration (q\'\' )')
plt.xlabel('Time (t)')
plt.legend()
plt.grid()

# Show the plots
plt.tight_layout()
plt.show()


