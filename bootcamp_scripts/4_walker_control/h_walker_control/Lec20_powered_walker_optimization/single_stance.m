%===================================================================
function [zdot,Th]=single_stance(t,z,walker)  
%===================================================================
theta1 = z(1);   omega1 = z(2);                         
theta2 = z(3);   omega2 = z(4);                         
                    
M = walker.M;  m = walker.m; I = walker.I;   
l = walker.l;  c = walker.c; 
g = walker.g; gam = walker.gam;

Th = 0;

%derived from poweredderive_newton
% A11 = 2*c*l*m - M*l^2 - 2*c^2*m - 2*l^2*m - 2*I + 2*c*l*m*cos(theta2);
% A12 = c*l*m*cos(theta2) - c^2*m - I;
% A21 = c*l*m*cos(theta2) - c^2*m - I;
% A22 = - I - c^2*m;
% b1 = c*g*m*sin(theta1 - gam + theta2) + M*g*l*sin(gam - theta1) - c*g*m*sin(gam - theta1) + 2*g*l*m*sin(gam - theta1) + c*l*m*omega2^2*sin(theta2) + 2*c*l*m*omega1*omega2*sin(theta2);
% b2 = c*g*m*sin(theta1 - gam + theta2) - Th - c*l*m*omega1^2*sin(theta2);
% A_ss = [A11 A12; A21 A22];
% b_ss = [b1; b2];
% alpha = A_ss\b_ss;

%derived from poweredderive_lagrange
A11 = 2*I + M*l^2 + 2*c^2*m + 2*l^2*m - 2*c*l*m - 2*c*l*m*cos(theta2);
A12 = I + c^2*m - c*l*m*cos(theta2);
A21 = I + c^2*m - c*l*m*cos(theta2);
A22 = I + c^2*m;
A_ss = [A11 A12; A21 A22];
 
b1 = c*g*m*sin(gam - theta1) - M*g*l*sin(gam - theta1) - c*g*m*sin(theta1 - gam + theta2) - 2*g*l*m*sin(gam - theta1) - c*l*m*omega2^2*sin(theta2) - 2*c*l*m*omega1*omega2*sin(theta2);
b2 = Th - c*g*m*sin(theta1 - gam + theta2) + c*l*m*omega1^2*sin(theta2);
b_ss = [b1; b2];
 
alpha = A_ss\b_ss;

zdot = [omega1 alpha(1) omega2 alpha(2)]';  
