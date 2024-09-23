clc
clear all
close all

A = [1 zeros(1,7); ....
     ones(1,4) zeros(1,4); ...
     zeros(1,4) ones(1,4); ...
     zeros(1,4) 1 3 9 27; ...
     0 1 zeros(1,6); ...
     zeros(1,5) 1 6 27; ...
     0 1 2 3 0 -1 -2 -3; ...
     0 0 1 3 0 0 -1 -3];
 
 b = [0 0.5 0.5 1 0 0 0 0]';
 x = inv(A)*b
 
a10 = x(1,1); a11 = x(2,1); a12 = x(3,1); a13 = x(4,1);
a20 = x(5,1); a21 = x(6,1); a22 = x(7,1); a23 = x(8,1);

t1 = linspace(0,1);
t2 = linspace(1,3);

q1 = a10 + a11*t1 + a12*t1.^2 + a13*t1.^3;
q2 = a20 + a21*t2 + a22*t2.^2 + a23*t2.^3;

q1dot = a11 + 2*a12*t1 + 3*a13*t1.^2;
q2dot = a21 + 2*a22*t2 + 3*a23*t2.^2;

q1ddot = 2*a12 + 6*a13*t1;
q2ddot = 2*a22 + 6*a23*t2;

figure(1)
subplot(2,2,1)
plot(t1,q1,'b',t2,q2','r--'); 
ylabel('$q$','Interpreter','latex');  xlabel('t');

subplot(2,2,2)
plot(t1,q1dot,'b',t2,q2dot,'r--'); 
ylabel('$\dot{q}$','Interpreter','latex'); xlabel('t');

subplot(2,2,3)
plot(t1,q1ddot,'b',t2,q2ddot,'r--'); 
ylabel('$\ddot{q}$','Interpreter','latex'); xlabel('t');
suptitle('Trajectory for Example 2')