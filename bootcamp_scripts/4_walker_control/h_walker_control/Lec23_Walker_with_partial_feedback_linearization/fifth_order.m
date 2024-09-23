clc
clear all

syms t t0 tf q0 qf q0dot qfdot real
syms a0 a1 a2 a3 a4 a5 real

q = a0+a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5;
qd = diff(q,t);
qdd = diff(qd,t);

eq(1) = subs(q,t,t0);
eq(2) = subs(q,t,tf);
eq(3) = subs(qd,t,t0);
eq(4) = subs(qd,t,tf);
eq(5) = subs(qdd,t,t0);
eq(6) = subs(qdd,t,tf);

x = [a0 a1 a2 a3 a4 a5];
for i=1:6
    A(i,:) = jacobian(eq(i),x); 
end
b = [q0 qf q0dot qfdot 0 0]';




% x = inv(A)*b
% x = simplify(inv(A))*b
% x = simplify(inv(A)*b) 
