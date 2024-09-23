clc
clear all

syms t0 tf q0 qf real

A = [1 t0; 1 tf];
b = [q0 qf]';

x = simplify(inv(A)*b)

%inv(A)

