function [T1,T2] = control(t,x,parms)

theta1 = x(1);                          
theta1dot = x(2);   
theta2 = x(3);                          
theta2dot = x(4); 

m1 = parms.m1;
l1 = parms.l1;
m2 = parms.m2;
l2 = parms.l2;
I1 = parms.I1;
I2 = parms.I2;
g = parms.g;

Kp1 = parms.control.Kp1;
Kd1 = parms.control.Kd1;
Kp2 = parms.control.Kp2;
Kd2 = parms.control.Kd2;

G1 = g*m2*((l2*cos(theta1 + theta2))/2 + l1*cos(theta1)) + (g*l1*m1*cos(theta1))/2;
G2 = (g*l2*m2*cos(theta1 + theta2))/2 ;
C1 = -(l1*l2*m2*theta2dot*sin(theta2)*(2*theta1dot + theta2dot))/2;
C2 = (l1*l2*m2*theta1dot^2*sin(theta2))/2;
M11 = I1 + I2 + (l1^2*m1)/4 + l1^2*m2 + (l2^2*m2)/4 + l1*l2*m2*cos(theta2);
M12 = I2 + (l2^2*m2)/4 + (l1*l2*m2*cos(theta2))/2;
M21 = I2 + (l2^2*m2)/4 + (l1*l2*m2*cos(theta2))/2;
M22 = I2 + (l2^2*m2)/4;

M = [M11 M12; M21 M22];
C = [C1; C2];
G = [G1; G2];
%hi

theta1_ref = interp1(parms.t,parms.control.theta1_ref,t);
theta1dot_ref = interp1(parms.t,parms.control.theta1dot_ref,t);
theta1ddot_ref = interp1(parms.t,parms.control.theta1ddot_ref,t);

theta2_ref = interp1(parms.t,parms.control.theta2_ref,t);
theta2dot_ref = interp1(parms.t,parms.control.theta2dot_ref,t);
theta2ddot_ref = interp1(parms.t,parms.control.theta2ddot_ref,t);

%the controller
theta = [theta1 theta2]';
theta_ref = [theta1_ref theta2_ref]';

thetadot = [theta1dot theta2dot]';
thetadot_ref = [theta1dot_ref theta2dot_ref]';

thetaddot_ref = [theta1ddot_ref theta2ddot_ref]';

Kp = [Kp1 0; 0 Kp2];
Kd = [Kd1 0; 0 Kd2];
T = M*(thetaddot_ref-Kp*(theta-theta_ref)-Kd*(thetadot-thetadot_ref))+C+G;

T1 = T(1,1);
T2 = T(2,1);