import sympy as sp
import numpy as np

# Define symbols
x, y = sp.symbols('x y', real=True)
q = [x, y]

f = [x**2+y**2, 2*x+3*y+5]

J = sp.simplify(sp.Matrix(f).jacobian(q))

print("===JACOBIAN HERE===")
print(J)
print("===================")
print()

q0 = [1, 2]
J_lala = J.subs([(x,1), (y,2)])
J_lala = J.subs(list(zip(q, q0)))
print("===JACOBIAN HERE===")
print(J_lala)
print("===================")
print()

def fn(x):
    return [x[0]**2+x[1]**2, 2*x[0]+3*x[1]+5]

def numerical_jacob(f, x):
    delta = 1e-5
    
    f_x = f(x)
    n_row = len(f_x)
    n_col = len(x)
    
    J_return = np.zeros([n_row, n_col])
    
    for i in range(n_row):
        for j in range(n_col):
            x_delta = x.copy()
            x_delta[j] = x_delta[j] + delta
            f_x_delta = f(x_delta)
            J_return[i,j] = (f_x_delta[i] - f_x[i]) / delta
    
    return J_return

J_num = numerical_jacob(f=fn, x=q0)
print("===JACOBIAN HERE===")
print(J_num)
print("===================")
print()

exit()