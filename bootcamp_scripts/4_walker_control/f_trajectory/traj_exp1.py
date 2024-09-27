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
    [0, 1, 2*ta_f, 3*ta_f**2, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 2*ta_f, 3*ta_f**2],
    [0, 0, 0, 0, 0, 1, 2*tb_f, 3*tb_f**2],
])

b= sp.Matrix([
    qa_0, qa_f, qa_f, qb_f, qadot_0, qadot_f, qadot_f, qbdot_f
])

print("=====A Matrix=====")
print(A)
print()
for i in range(8):
    for j in range(8):
        print("A"+str(i)+str(j)+" = ", A[i,j])
print("==================")
print()
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
    A51 =  1
    A52 =  2*ta_f_
    A53 =  3*ta_f_**2
    A54 =  0
    A55 =  0
    A56 =  0
    A57 =  0
    A60 =  0
    A61 =  0
    A62 =  0
    A63 =  0
    A64 =  0
    A65 =  1
    A66 =  2*ta_f_
    A67 =  3*ta_f_**2
    A70 =  0
    A71 =  0
    A72 =  0
    A73 =  0
    A74 =  0
    A75 =  1
    A76 =  2*tb_f_
    A77 =  3*tb_f_**2
    
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
    
    b = np.array([qa_0_, qa_f_, qa_f_, qb_f_, qadot_0_, qadot_f_, qadot_f_, qbdot_f_])
    
    poly_coef = np.linalg.solve(A, b)
    
    return poly_coef


poly_coef = solve(ta_0_=0, ta_f_=1, tb_f_=3, qa_0_=0, qa_f_=0.5, qb_f_=1.0, qadot_0_=0, qadot_f_=0.5, qbdot_f_=0)

print(poly_coef)


t_step = 1e-3
ta = np.arange(0, 1, t_step)
tb = np.arange(1, 3, t_step)



q1 = a10 + a11*t1 + a12*t1.^2 + a13*t1.^3;
q2 = a20 + a21*t2 + a22*t2.^2 + a23*t2.^3;



