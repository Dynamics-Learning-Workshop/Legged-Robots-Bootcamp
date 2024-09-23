# Heel strike computations
# J_sw = sp.simplify(sp.Matrix([x_C2, y_C2]).jacobian([x, y, theta1, theta2]))
# J_n_sw = sp.simplify(J_sw.subs([(theta1, theta1_n), (theta2, theta2_n)]))
# A_n_hs = sp.simplify(M_ss.subs([(theta1, theta1_n), (theta2, theta2_n)]))

# print("Heel strike Jacobian J = ", J_n_sw)
# print("A_n_hs = ", A_n_hs)
print("DERIVATION END")