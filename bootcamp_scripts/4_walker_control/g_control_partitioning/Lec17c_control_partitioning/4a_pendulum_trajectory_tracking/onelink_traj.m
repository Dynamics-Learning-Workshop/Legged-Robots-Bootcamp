clc
clear all
close all

syms a10 a11 a12 a13 real
syms a20 a21 a22 a23 real
syms t real

%segment 1
t1_0 = 0; %initial time
t1_N = 1.5; %final time
theta1_0 = 0; %initial position
theta1_N = sym('pi')/2; %final position


%segment 2
t2_0 = 1.5;
t2_N = 3;
theta2_0 = theta1_N;
theta2_N = 0;

theta1 = a10+a11*t+a12*t^2+a13*t^3;
theta2 = a20+a21*t+a22*t^2+a23*t^3;
theta1dot = diff(theta1,t);
theta2dot = diff(theta2,t);
theta1ddot = diff(theta1dot,t);
theta2ddot = diff(theta2dot,t);

%various eqns
eqn1 = subs(theta1,t,t1_0)-theta1_0;
eqn2 = subs(theta1,t,t1_N)-theta1_N;
eqn3 = subs(theta2,t,t2_0)-theta2_0;
eqn4 = subs(theta2,t,t2_N)-theta2_N;
eqn5 = subs(theta1dot,t,t1_0)-0;
eqn6 = subs(theta2dot,t,t2_N)-0;
eqn7 = subs(theta1dot,t,t1_N)-subs(theta2dot,t,t2_0);
eqn8 = subs(theta1ddot,t,t1_N)-subs(theta2ddot,t,t2_0);

%%We want to write the above equations at A x = b
%where x = [a10 a11 a12 a13 a20 a21 a22 a23 ]; 
%A = coefficients of a10 a11 a12 a13 a20 a21 a22 a23; b=constants coefficients; 
A(1,:) = jacobian(eqn1,[a10 a11 a12 a13 a20 a21 a22 a23]);
A(2,:) = jacobian(eqn2,[a10 a11 a12 a13 a20 a21 a22 a23]);
A(3,:) = jacobian(eqn3,[a10 a11 a12 a13 a20 a21 a22 a23]);
A(4,:) = jacobian(eqn4,[a10 a11 a12 a13 a20 a21 a22 a23]);
A(5,:) = jacobian(eqn5,[a10 a11 a12 a13 a20 a21 a22 a23]);
A(6,:) = jacobian(eqn6,[a10 a11 a12 a13 a20 a21 a22 a23]);
A(7,:) = jacobian(eqn7,[a10 a11 a12 a13 a20 a21 a22 a23]);
A(8,:) = jacobian(eqn8,[a10 a11 a12 a13 a20 a21 a22 a23]);

%The negative sign is because we want to move b to the other side 
b(1,1) = -subs(eqn1,[a10 a11 a12 a13 a20 a21 a22 a23],zeros(1,8));
b(2,1) = -subs(eqn2,[a10 a11 a12 a13 a20 a21 a22 a23],zeros(1,8));
b(3,1) = -subs(eqn3,[a10 a11 a12 a13 a20 a21 a22 a23],zeros(1,8));
b(4,1) = -subs(eqn4,[a10 a11 a12 a13 a20 a21 a22 a23],zeros(1,8));
b(5,1) = -subs(eqn5,[a10 a11 a12 a13 a20 a21 a22 a23],zeros(1,8));
b(6,1) = -subs(eqn6,[a10 a11 a12 a13 a20 a21 a22 a23],zeros(1,8));
b(7,1) = -subs(eqn7,[a10 a11 a12 a13 a20 a21 a22 a23],zeros(1,8));
b(8,1) = -subs(eqn8,[a10 a11 a12 a13 a20 a21 a22 a23],zeros(1,8));

%Next we solve for x = inv(A)*b = A\b
% %The matrices A and b
% A 
% b

%Find constants a's
x = A\b;


disp('Copy-paste into the code - onelink_main.');
disp(['t1_0 = ',num2str(t1_0),';']);
disp(['t1_N = ',num2str(t1_N),';']);
disp(['t2_0 = ',num2str(t2_0),';']);
disp(['t2_N = ',num2str(t2_N),';']);
disp(' ');

disp('Copy-paste into the code - onelink_main.');
disp(['a10 = ',char(x(1)),';']);
disp(['a11 = ',char(x(2)),';']);
disp(['a12 = ',char(x(3)),';']);
disp(['a13 = ',char(x(4)),';']);
disp(['a20 = ',char(x(5)),';']);
disp(['a21 = ',char(x(6)),';']);
disp(['a22 = ',char(x(7)),';']);
disp(['a23 = ',char(x(8)),';']);

% disp(['thetaA =', char(theta1),';']); 
% disp(['thetaAdot =', char(theta1dot),';']); 
% disp(['thetaAddot =', char(theta1ddot),';']); 
% disp(['thetaB =', char(theta2),';']); 
% disp(['thetaBdot =', char(theta2dot),';']); 
% disp(['thetaBddot =', char(theta2ddot),';']); 
