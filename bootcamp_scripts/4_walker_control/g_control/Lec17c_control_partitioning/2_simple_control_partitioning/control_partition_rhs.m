function xdot=control_partition_rhs(t,x,parms)           

q1 = x(1);                          
q1dot = x(2);                          
q2 = x(3);
q2dot = x(4);

q = [q1; q2];
qdot = [q1dot; q2dot];

M = parms.M;
C = parms.C;
K = parms.K;


M_hat = parms.M_hat;
C_hat = parms.C_hat;
K_hat = parms.K_hat;
Kp = parms.Kp;
Kd = parms.Kd;
qd = parms.qd;

%controller
tau = M_hat*(-Kp*(q-qd)-Kd*qdot)+C_hat*qdot+K_hat*q; %control partitioning

%system dynamics
qddot = M \ (-C*qdot-K*q+tau);

q1ddot = qddot(1);
q2ddot = qddot(2);

xdot = [q1dot q1ddot q2dot q2ddot]'  ;            
