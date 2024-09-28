function xdot=twolink_rhs(t,x,parms)    

theta1 = x(1);                          
theta1dot = x(2);   
theta2 = x(3);                          
theta2dot = x(4);  

%use estimates for mass and inertia
m1 = parms.m1_hat;
I1 = parms.I1_hat;
m2 = parms.m2_hat;
I2 = parms.I2_hat;

l1 = parms.l1;
l2 = parms.l2;
g = parms.g;



%get disturbance values
T1_disturb = interp1(parms.t,parms.disturb.T1,t);
dtheta1_noise =  interp1(parms.t,parms.disturb.theta1,t);
dtheta1dot_noise =  interp1(parms.t,parms.disturb.theta1dot,t);
T2_disturb = interp1(parms.t,parms.disturb.T2,t);
dtheta2_noise =  interp1(parms.t,parms.disturb.theta2,t);
dtheta2dot_noise =  interp1(parms.t,parms.disturb.theta2dot,t);

theta1_sensor = theta1 + dtheta1_noise;
theta1dot_sensor = theta1dot + dtheta1dot_noise;
theta2_sensor = theta2 + dtheta2_noise;
theta2dot_sensor = theta2dot + dtheta2dot_noise;

sensor = [theta1_sensor theta1dot_sensor theta2_sensor theta2dot_sensor]; %corrupt sensor values

[T1,T2] = control(t,sensor,parms);

%add disturbance due to torque
T1 = T1 - T1_disturb;
T2 = T2 - T2_disturb;

%T1 = 0; %zero torques for unactuated system
%T1 = -0.1*theta1dot; %viscous friction
%T2 = 0; %zero torques for unactuated system
%T2 = -0.3*theta2dot; %viscous friction

G1 = g*m2*((l2*cos(theta1 + theta2))/2 + l1*cos(theta1)) - T1 + (g*l1*m1*cos(theta1))/2;
G2 = (g*l2*m2*cos(theta1 + theta2))/2 - T2;
C1 = -(l1*l2*m2*theta2dot*sin(theta2)*(2*theta1dot + theta2dot))/2;
C2 = (l1*l2*m2*theta1dot^2*sin(theta2))/2;
M11 = I1 + I2 + (l1^2*m1)/4 + l1^2*m2 + (l2^2*m2)/4 + l1*l2*m2*cos(theta2);
M12 = I2 + (l2^2*m2)/4 + (l1*l2*m2*cos(theta2))/2;
M21 = I2 + (l2^2*m2)/4 + (l1*l2*m2*cos(theta2))/2;
M22 = I2 + (l2^2*m2)/4;

M = [M11 M12; M21 M22]; 
C = [C1; C2];
G = [G1; G2];



%Note that M Xdot + G + C = 0
%Thus Xdot = inv(M)*(-G-C)

thetaddot = M\(-G-C); %M\ is same as inv(M)* but more efficient
theta1ddot = thetaddot(1,1);
theta2ddot = thetaddot(2,1);

xdot = [theta1dot theta1ddot theta2dot theta2ddot]'  ;            
