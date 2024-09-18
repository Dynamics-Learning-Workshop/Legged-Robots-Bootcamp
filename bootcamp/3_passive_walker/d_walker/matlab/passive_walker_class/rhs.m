function zdot = rhs(t,z,walker)

theta1 = z(1);
omega1 = z(2); 
theta2 = z(3);
omega2 = z(4);

I = walker.I;
m = walker.m;
M = walker.M;
c = walker.c;
l = walker.l;
g = walker.g;
gam = walker.gam;

A11 = 2*I + M*l^2 + 2*c^2*m + 2*l^2*m - 2*c*l*m - 2*c*l*m*cos(theta2);
A12 = I + c^2*m - c*l*m*cos(theta2);
A21 = I + c^2*m - c*l*m*cos(theta2);
A22 = I + c^2*m;
A_ss = [A11 A12; A21 A22];
 
b1 = c*g*m*sin(gam - theta1) - M*g*l*sin(gam - theta1) - c*g*m*sin(theta1 - gam + theta2) - 2*g*l*m*sin(gam - theta1) - c*l*m*omega2^2*sin(theta2) - 2*c*l*m*omega1*omega2*sin(theta2);
b2 = -c*m*(g*sin(theta1 - gam + theta2) - l*omega1^2*sin(theta2));
b_ss = [b1; b2];
 

alpha = A_ss\b_ss;
%alpha = [theta1ddot, theta2ddot] %refer on notes
theta1ddot = alpha(1);
theta2ddot = alpha(2);

%z = [theta1 theta1dot theta2 theta2dot]
zdot = [omega1 theta1ddot omega2 theta2ddot]';

% ydot = y(2);
% yddot = -ball.g;
% 
% dqdt = [ydot yddot]';

