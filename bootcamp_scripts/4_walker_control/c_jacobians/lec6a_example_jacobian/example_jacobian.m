function example_jacobian

clc
clear all

syms x y real
q = [x y];
q0 = [1 2];

f = fn(q);
J = jacobian(f,q)

J_sym = subs(J,q,q0)

J_num = jacob(@fn,q0) %own function jacob which uses finite difference

function f = fn(z)
x = z(1); 
y = z(2);
%f = [f1, f2];
f = [x^2+y^2 2*x+3*y+5];

function J = jacob(F,z)
pert = 1e-5;

%%%%%%%
%J = [ (f1(x+dx,y) - f1(x-dx,y)) )/ 2*dx  (f1(x,y+dy) - f1(x,y+dy)/2*dy)
     % ....
for i=1:2
    ztemp1 = z;
    ztemp1(i) = ztemp1(i) - pert;
    F1 = F(ztemp1);
    
    ztemp2 = z;
    ztemp2(i) = ztemp2(i) + pert;
    F2 = F(ztemp2);
    
    J(:,i) = (F2-F1)/(2*pert);
end
    