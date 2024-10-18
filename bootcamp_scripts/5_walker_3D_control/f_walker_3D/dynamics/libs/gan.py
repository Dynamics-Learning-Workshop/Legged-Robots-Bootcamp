import sympy as sp
import numpy as np

# Define a sympy symbol and expression
x = sp.Symbol('x')
expr = sp.cos(x)

# # Substitute sympy.cos with np.cos
np_cos = sp.Symbol('np.cos')
print(np_cos)
expr_np = expr.subs(sp.cos, np_cos)

print(expr_np)