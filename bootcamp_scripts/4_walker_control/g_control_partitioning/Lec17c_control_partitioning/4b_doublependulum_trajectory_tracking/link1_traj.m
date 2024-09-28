clc
clear all
close all

syms a10 a11 a12 a13 real
syms t real

%segment 1
t1_0 = 0; %initial time
t1_N = 3; %final time
theta1_0 = -(sym('pi')/2)-0.5; %initial position
theta1_N = -(sym('pi')/2)+0.5;

theta1 = a10+a11*t+a12*t^2+a13*t^3;
theta1dot = diff(theta1,t);
theta1ddot = diff(theta1dot,t);

%various eqns
eqn1 = subs(theta1,t,t1_0)-theta1_0;
eqn2 = subs(theta1,t,t1_N)-theta1_N;
eqn3 = subs(theta1dot,t,t1_0)-0;
eqn4 = subs(theta1dot,t,t1_N)-0;

%%We want to write the above equations at A x = b
%where x = [a10 a11 a12 a13 ]; 
%A = coefficients of a10 a11 a12 a13 ; b=constants coefficients; 
A(1,:) = jacobian(eqn1,[a10 a11 a12 a13]);
A(2,:) = jacobian(eqn2,[a10 a11 a12 a13]);
A(3,:) = jacobian(eqn3,[a10 a11 a12 a13]);
A(4,:) = jacobian(eqn4,[a10 a11 a12 a13]);


%The negative sign is because we want to move b to the other side 
b(1,1) = -subs(eqn1,[a10 a11 a12 a13],zeros(1,4));
b(2,1) = -subs(eqn2,[a10 a11 a12 a13],zeros(1,4));
b(3,1) = -subs(eqn3,[a10 a11 a12 a13],zeros(1,4));
b(4,1) = -subs(eqn4,[a10 a11 a12 a13],zeros(1,4));


%Next we solve for x = inv(A)*b = A\b
% %The matrices A and b
% A 
% b

%Find constants a's
x = A\b;


disp('Copy-paste into the code - onelink_main.');
disp(['t1_0 = ',num2str(t1_0),';']);
disp(['t1_N = ',num2str(t1_N),';']);
disp(' ');

disp('Copy-paste into the code - onelink_main.');
disp(['a10 = ',char(x(1)),';']);
disp(['a11 = ',char(x(2)),';']);
disp(['a12 = ',char(x(3)),';']);
disp(['a13 = ',char(x(4)),';']);


% disp(['thetaA =', char(theta1),';']); 
% disp(['thetaAdot =', char(theta1dot),';']); 
% disp(['thetaAddot =', char(theta1ddot),';']); 
% disp(['thetaB =', char(theta2),';']); 
% disp(['thetaBdot =', char(theta2dot),';']); 
% disp(['thetaBddot =', char(theta2ddot),';']); 
