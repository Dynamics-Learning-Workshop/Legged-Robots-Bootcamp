function xdot=onelink_rhs(t,x,parms)           

theta1 = x(1);                          
theta1dot = x(2);                          
m1 = parms.m1;
l1 = parms.l1;
I1 = parms.I1;
g = parms.g;

%get disturbance values
T1_disturb = interp1(parms.t,parms.disturb.T1,t);
dtheta1_noise =  interp1(parms.t,parms.disturb.theta1,t);
dtheta1dot_noise =  interp1(parms.t,parms.disturb.theta1dot,t);
theta1_sensor = theta1 + dtheta1_noise;
theta1dot_sensor = theta1dot + dtheta1dot_noise;
sensor = [theta1_sensor theta1dot_sensor]; %corrupt sensor values

%get motor torque
T1 = control(t,sensor,parms)-T1_disturb; %-parms.control.Kp1*(theta1-parms.control.theta1ref)-parms.control.Kd1*theta1dot;

%equations of motion
G1 = -T1+0.5*m1*g*l1*cos(theta1);
C1 = 0;
M1 = I1+(m1*l1*l1/4);
theta1ddot = M1\(-G1-C1);

xdot = [theta1dot theta1ddot]'  ;            