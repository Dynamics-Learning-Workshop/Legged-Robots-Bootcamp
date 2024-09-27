import sympy as sp, numpy as np
import matplotlib.pyplot as plt

t0, tf, q0, qf = sp.symbols('t0 tf q0 qf', real=True)

# solving a 3rd-order polynomial
A = sp.Matrix([
    [1, t0, t0**2, t0**3],
    [1, tf, tf**2, tf**3],
    [0, 1, 2*t0, 3*t0**2],
    [0, 1, 2*tf, 3*tf**2]
])

b= sp.Matrix([
    q0, qf, 0, 0    
])

x = sp.simplify(A**(-1) * b)
print(x)
lala = x.subs([(t0, 0), (tf, 1), (q0, 0), (qf, 1)])
print(sp.simplify(lala))

# exit()

print("=====A Matrix=====")
print(A)
print()
for i in range(4):
    for j in range(4):
        print("A"+str(i)+str(j)+" = ", A[i,j])
print("==================")
print()

# example 0
def solve(t0_, tf_, q0_, qf_):
    A00 =  1
    A01 =  t0_
    A02 =  t0_**2
    A03 =  t0_**3
    A10 =  1
    A11 =  tf_
    A12 =  tf_**2
    A13 =  tf_**3
    A20 =  0
    A21 =  1
    A22 =  2*t0_
    A23 =  3*t0_**2
    A30 =  0
    A31 =  1
    A32 =  2*tf_
    A33 =  3*tf_**2
    A = np.array([
        [A00, A01, A02, A03],
        [A10, A11, A12, A13],
        [A20, A21, A22, A23],
        [A30, A31, A32, A33],
    ])
    print(A)
    b = np.array([q0_, qf_, 0, 0])
    
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
a2 = poly_coef[2]
a3 = poly_coef[3]

t_step = 1e-3
t = np.arange(t0_, tf_, t_step)
q = a0 + a1 * t + a2 * t**2 + a3 * t**3
qdot = a1 + 2 * a2 * t + 3 * a3 * t**2
qddot = 2 * a2 + 6 * a3 * t

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

# Plot acceleration
plt.subplot(3, 1, 3)
plt.plot(t, qddot, label='Acceleration q\'\'(t)', color='r')
plt.ylabel('Acceleration (q\'\' )')
plt.xlabel('Time (t)')
plt.legend()
plt.grid()

# Show the plots
plt.tight_layout()
plt.show()