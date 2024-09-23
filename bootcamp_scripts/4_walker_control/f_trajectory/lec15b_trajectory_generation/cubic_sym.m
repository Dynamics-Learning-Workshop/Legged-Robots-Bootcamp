clc
clear all

syms t0 tf q0 qf real

%We have Ax = b and we are solving for x
A = [1 t0 t0^2 t0^3; ...
     1 tf tf^2 tf^3; ...
     0 1  2*t0 3*t0^2; ...
     0 1  2*tf 3*tf^2];
b = [q0 qf 0 0]';


x = inv(A)*b
x = simplify(inv(A))*b
x = simplify(inv(A)*b) 
