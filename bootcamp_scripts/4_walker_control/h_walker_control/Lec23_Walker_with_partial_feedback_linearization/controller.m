function [u,theta2_ref,theta2dot_ref,theta2ddot_ref] = controller(t,z,walker)

tt = t-walker.control.t0;

theta1 = z(1);   omega1 = z(2);                         
theta2 = z(3);   omega2 = z(4);                         
                    
M = walker.M;  m = walker.m; I = walker.I;   
l = walker.l;  c = walker.c; 
g = walker.g; gam = walker.gam;

%%%%%%%% fifth order polynomial %%%%%%
t0 = 0;
tf = walker.control.tf;  
q0 = walker.control.theta20;
qf = walker.control.theta2f;
q0dot = walker.control.theta20dot;
qfdot = 0;
AA = [ 1, t0, t0^2,   t0^3,    t0^4,    t0^5;
       1, tf, tf^2,   tf^3,    tf^4,    tf^5; 
       0,  1, 2*t0, 3*t0^2,  4*t0^3,  5*t0^4;
       0,  1, 2*tf, 3*tf^2,  4*tf^3,  5*tf^4;
       0,  0,    2,   6*t0, 12*t0^2, 20*t0^3;
       0,  0,    2,   6*tf, 12*tf^2, 20*tf^3];
bb = [q0 qf q0dot qfdot 0 0]';
xx = AA\bb;
a0 = xx(1); a1 = xx(2); a2 = xx(3); 
a3 = xx(4); a4 = xx(5); a5 = xx(6);
if (tt>tf)
    tt = tf;
end

theta2_ref = a5*tt^5 + a4*tt^4 + a3*tt^3 + a2*tt^2 + a1*tt + a0;
theta2dot_ref = 5*a5*tt^4 + 4*a4*tt^3 + 3*a3*tt^2 + 2*a2*tt + a1;
theta2ddot_ref =   20*a5*tt^3 + 12*a4*tt^2 + 6*a3*tt + 2*a2;

M11 = 2*I + M*l^2 + 2*c^2*m + 2*l^2*m - 2*c*l*m - 2*c*l*m*cos(theta2);
M12 = I + c^2*m - c*l*m*cos(theta2);
M21 = I + c^2*m - c*l*m*cos(theta2);
M22 = I + c^2*m;
Ms = [M11 M12; M21 M22];
 
N1 = c*g*m*sin(theta1 - gam + theta2) + M*g*l*sin(gam - theta1) - c*g*m*sin(gam - theta1) + 2*g*l*m*sin(gam - theta1) + c*l*m*omega2^2*sin(theta2) + 2*c*l*m*omega1*omega2*sin(theta2);
N2 = c*m*(g*sin(theta1 - gam + theta2) - l*omega1^2*sin(theta2));
Ns = [N1; N2];

Kp = walker.control.Kp;
Kd = 2*sqrt(Kp);
B = [0; 1];
Sc = [0 1];
e = theta2-theta2_ref;
edot = omega2 - theta2dot_ref;
v = theta2ddot_ref - Kp*e - Kd*edot;
Minv = Ms\eye(2);
u = (Sc*Minv*B)\(v+Sc*Minv*Ns);
