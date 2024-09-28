function T1 = control(t,x,parms)

theta1 = x(1);                          
theta1dot = x(2);  

m1 = parms.m1;
l1 = parms.l1;
I1 = parms.I1;
g = parms.g;

Kp1 = parms.control.Kp1;
Kd1 = parms.control.Kd1;
theta1ref = parms.control.theta1ref;

G1 = 0.5*m1*g*l1*cos(theta1);
C1 = 0;
M1 = I1+(m1*l1*l1/4);

%the controller
%control partitioning
% torque = M(-kp(q-qd) - kdqdot) + C + G
T1 = M1*(-Kp1*(theta1-theta1ref)-Kd1*theta1dot)+C1+G1;