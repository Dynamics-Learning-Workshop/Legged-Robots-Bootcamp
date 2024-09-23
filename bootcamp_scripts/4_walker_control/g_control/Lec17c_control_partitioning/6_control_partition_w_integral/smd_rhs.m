function xdot=smd_rhs(t,x,parms)           

q = x(1);                          
qdot = x(2);  
qsum = x(3); %added third state

m = parms.m;
c = parms.c;
k = parms.k;

kp = parms.kp;
kd = parms.kd;

qddot = -( (k+kp)/m)*q-((c+kd)/m)*qdot - parms.ki*qsum + parms.T_dist;

xdot = [qdot qddot q]'  ;            
