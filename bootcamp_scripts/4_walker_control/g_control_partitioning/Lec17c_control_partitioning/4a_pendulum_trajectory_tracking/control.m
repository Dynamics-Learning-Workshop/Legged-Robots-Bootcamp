function T1 = control(t,x,parms)

theta1 = x(1);                          
theta1dot = x(2);  

m1 = parms.m1_hat;
l1 = parms.l1;
I1 = parms.I1_hat;
g = parms.g;


Kp1 = parms.control.Kp1;
Kd1 = parms.control.Kd1;


G1 = 0.5*m1*g*l1*cos(theta1);
C1 = 0;
M1 = I1+(m1*l1*l1/4);

%interp1 interpolates the values for an intermediate
theta1_ref = interp1(parms.t,parms.control.theta_ref,t);
theta1dot_ref = interp1(parms.t,parms.control.thetadot_ref,t);
theta1ddot_ref = interp1(parms.t,parms.control.thetaddot_ref,t);

%the controller (control partitioning)
T1 = M1*(theta1ddot_ref-Kp1*(theta1-theta1_ref)-Kd1*(theta1dot-theta1dot_ref))+C1+G1;

