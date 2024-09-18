clc
clear all

syms x real
f = x^2+2*x+2;
dfdx = diff(f,x)

subs(dfdx,1)