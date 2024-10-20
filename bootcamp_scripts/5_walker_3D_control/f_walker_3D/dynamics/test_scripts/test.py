# try:
#     from symengine import var, exp, cos, sin, Integer, Float
# except ImportError:
#     from sympy import symbols as var
#     from sympy import exp, cos, sin, Integer, Float
from symengine import symengine
# import symengine


# Define symbols
dof = 3
q = se.symbols('q:3')      # Generalized coordinates
qdot = se.symbols('qdot:3')  # Generalized velocities
qddot = se.symbols('qddot:3')  # Generalized accelerations
L = sum(q[i]**2 + qdot[i]**2 for i in range(dof))  # Example Lagrangian

# # Deriving the Equations of Motion using SymEngine
EOM = []
for i in range(dof):
    dLdqdot = se.diff(L, qdot[i])
    ddt_dLdqdot = sum(se.diff(dLdqdot, q[j]) * qdot[j] + se.diff(dLdqdot, qdot[j]) * qddot[j] for j in range(dof))
    dLdq = se.diff(L, q[i])
    EOM.append(ddt_dLdqdot - dLdq)

print("{d/dt dL/dqdot - dL/dq} ACQUIRED")
