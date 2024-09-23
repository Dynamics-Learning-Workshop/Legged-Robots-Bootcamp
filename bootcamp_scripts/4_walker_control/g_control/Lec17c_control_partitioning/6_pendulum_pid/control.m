function T1 = control(t,x,parms)

theta1 = x(1);                          
theta1dot = x(2); 
theta1sum = x(3);

%the controller
T1 = -parms.control.Kp1*(theta1-parms.control.theta1ref)-parms.control.Kd1*theta1dot-parms.control.Ki1*theta1sum;