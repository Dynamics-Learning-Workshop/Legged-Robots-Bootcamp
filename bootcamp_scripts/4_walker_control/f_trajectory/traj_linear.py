import sympy as sp, numpy as np
import matplotlib.pyplot as plt

t0, tf, q0, qf = sp.symbols('t0 tf q0 qf', real=True)

# solving a 3rd-order polynomial
A = sp.Matrix([
    [1, t0],
    [1, tf]
])

b= sp.Matrix([
    q0, qf    
])

x = sp.simplify(A**(-1) * b)
print(x)
# lala = x.subs([(t0, 0), (tf, 1), (q0, 0), (qf, 1)])
# print(sp.simplify(lala))

print("=====A Matrix=====")
print(A)
print()
for i in range(2):
    for j in range(2):
        print("A"+str(i)+str(j)+" = ", A[i,j])
print("==================")
print()


# example 0
def solve(t0_, tf_, q0_, qf_):
    A00 =  1
    A01 =  t0_
    A10 =  1
    A11 =  tf_
    
    A = np.array([
        [A00, A01],
        [A10, A11]
    ])
    print(A)
    b = np.array([q0_, qf_])
    
    poly_coef = np.linalg.solve(A, b)
    
    return poly_coef

t0_ = 0
tf_ = 1
q0_ = 0
qf_ = 1

poly_coef = solve(t0_=t0_, tf_=tf_, q0_=q0_, qf_=qf_)
print(poly_coef)

a0 = poly_coef[0]
a1 = poly_coef[1]

t_step = 1e-3
t = np.arange(t0_, tf_, t_step)
q = a0 + a1 * t
qdot = a1 * np.ones([len(t)])


# Create plots
plt.figure(figsize=(12, 8))

# Plot position
plt.subplot(3, 1, 1)
plt.plot(t, q, label='Position q(t)', color='b')
plt.title('Position, Velocity, and Acceleration vs. Time')
plt.ylabel('Position (q)')
plt.legend()
plt.grid()

# Plot velocity
plt.subplot(3, 1, 2)
plt.plot(t, qdot, label='Velocity q\'(t)', color='orange')
plt.ylabel('Velocity (q\' )')
plt.legend()
plt.grid()

# Show the plots
plt.tight_layout()
plt.show()