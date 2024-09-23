clc
clear all

q0 = 0;
qf = 1;
t0 = 0;
tf = 1;


a0 =  (qf*t0^2*(t0 - 3*tf))/(t0 - tf)^3 + (q0*tf^2*(3*t0 - tf))/(t0 - tf)^3;
a1 =                    (6*qf*t0*tf)/(t0 - tf)^3 - (6*q0*t0*tf)/(t0 - tf)^3;
a2 =            (3*q0*(t0 + tf))/(t0 - tf)^3 - (3*qf*(t0 + tf))/(t0 - tf)^3;
a3 =                               (2*qf)/(t0 - tf)^3 - (2*q0)/(t0 - tf)^3;

t = linspace(t0,tf);

q = a0+a1*t+a2*t.^2+a3*t.^3;
qdot = a1+2*a2*t+3*a3*t.^2;
qddot = 2*a2+6*a3*t;

figure(1)
subplot(2,2,1)
plot(t,q); ylabel('$q$','Interpreter','latex');  xlabel('t');

subplot(2,2,2)
plot(t,qdot); ylabel('$\dot{q}$','Interpreter','latex'); xlabel('t'); 

subplot(2,2,3)
plot(t,qddot); ylabel('$\ddot{q}$','Interpreter','latex'); xlabel('t');

suptitle('Cubic polynomial')
