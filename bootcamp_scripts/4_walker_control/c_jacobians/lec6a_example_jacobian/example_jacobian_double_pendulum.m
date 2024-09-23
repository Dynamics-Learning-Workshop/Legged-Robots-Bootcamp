clc
clear all
close all

%%%%% symbolic quantities
syms theta1 theta2 real
syms c1 c2 l real
syms omega1 omega2 real

%%%%% position vectors %%%%
x_G1 = c1*sin(theta1); 
y_G1 = -c1*cos(theta1);
x_G2 = l*sin(theta1) + c2*sin(theta1+theta2); 
y_G2 = -l*cos(theta1) -c2*cos(theta1+theta2);

%%%%% Jacobian %%%%%%
J_G1 = jacobian([x_G1 y_G1],[theta1 theta2])
J_G2 = jacobian([x_G2 y_G2],[theta1 theta2])

%%%% finding the velocity 
qdot = [omega1; omega2];
v_G1 = J_G1*qdot
v_G2 = J_G2*qdot
