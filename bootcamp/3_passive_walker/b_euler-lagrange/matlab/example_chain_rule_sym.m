clc
clear all

%%%%% symbolic variables 
syms x xdot xddot real

%%%%% f1 = cos(x) where x = x(t) %%%
f1 = sin(x);
df1dx = diff(f1,x)*xdot

%%%% f2 = x*xdot where x = x(t) %%%
f2 = x*xdot;
df2dx = diff(f2,x)*xdot + diff(f2,xdot)*xddot