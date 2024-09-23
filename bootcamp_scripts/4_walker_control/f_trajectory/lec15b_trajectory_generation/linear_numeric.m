clc
clear all

q0 = 0;
qf = 1;
t0 = 0;
tf = 1;


 
a0 = -(q0*tf - qf*t0)/(t0 - tf);
a1 =        (q0 - qf)/(t0 - tf);
        
t = linspace(t0,tf);

q = a0+a1*t;
qdot = a1*ones(1,length(t));


figure(1)
subplot(2,2,1)
plot(t,q); ylabel('$q$','Interpreter','latex');  xlabel('t');

subplot(2,2,2)
plot(t,qdot); ylabel('$\dot{q}$','Interpreter','latex'); xlabel('t'); 

suptitle('Linear polynomial')
