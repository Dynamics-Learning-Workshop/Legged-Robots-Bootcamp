function T1 = control(t,x,parms)

theta1 = x(1);                          
theta1dot = x(2);  

%the controller
%SIMPLE PD CONTROL
%tau = -kp*(q - qd) - kd*qdot
T1 = -parms.control.Kp1*(theta1-parms.control.theta1ref)-parms.control.Kd1*theta1dot;