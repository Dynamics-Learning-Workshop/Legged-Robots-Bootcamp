import sympy as sp
from T_ZRM_F import T_ZRM_F
import time

# Define symbolic variables
ux, uy, uz = sp.symbols('ux uy uz')
rx, ry, rz = sp.symbols('rx ry rz')
roll = sp.symbols('roll')



def T_ZRM(
    u_, r_, phi_ 
):
    
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
    
    T_ZRM = sp.Matrix([[R_ZRM, t_ZRM],
                    [0,0,0,1]])
    
    ux_ = u_[0]
    uy_ = u_[1]
    uz_ = u_[2]
    
    rx_ = r_[0]
    ry_ = r_[1]
    rz_ = r_[2]
    
    T_ZRM = sp.simplify(T_ZRM.subs([(ux,ux_), (uy,uy_), (uz,uz_), (rx,rx_), (ry,ry_), (rz,rz_), (phi,phi_)]))
    
    return T_ZRM

r = sp.Matrix([0,0,0])
u = sp.Matrix([1,0,0])

t_now = time.time()
T_UR_2_O = T_ZRM_F(u, r, roll)
print(time.time() - t_now)

t_now = time.time()
T_UR_2_O = T_ZRM(u,r,roll) # from upperbody {U} (roll) to ZRM {O}
print(time.time() - t_now)