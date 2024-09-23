clc;
clear all;

syms M I real %Mass Hip, leg, Inertia
syms l real % Distances as defined in figures
syms gam g beta real %Slope of ramp, gravity
syms theta1 real %Angles as defined in figures 
syms omega1 real %Angular velocity
syms alpha1 real%Angular Acceleration
syms theta1_n real %angles before heelstrike
syms omega1_n real %velocities before heelstrike
syms x y real %position of the stance leg
syms vx vy real %velocity of the stance leg
syms ax ay real %acceleration of the stance leg


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Position Vectors                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% position vectors %%%%%%%
R01 = simplify([cos(pi/2+theta1) -sin(pi/2+theta1); 
       sin(pi/2+theta1)  cos(pi/2+theta1)]);
R12 = simplify([cos(-pi+beta) -sin(-pi+beta); 
                sin(-pi+beta)  cos(-pi+beta)]);
r_C1 = [x; y];

r_H = r_C1 + R01*[l; 0];
x_H = r_H(1); y_H = r_H(2);

y_H

r_C2 = r_H + R01*R12*[l; 0]; 
r_C2 = simplify(r_C2);
x_C2 = r_C2(1); y_C2 = r_C2(2);

%%%%% velocity vectors %%%%%%
v_H_x = jacobian(x_H,[x y theta1])*[vx vy omega1]'; 
v_H_y = jacobian(y_H,[x y theta1])*[vx vy omega1]';
v_H = [v_H_x; v_H_y];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Position vectors for potential energy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Get positions of masses wrt to global frame
R = simplify([cos(-gam) -sin(-gam); 
              sin(-gam)  cos(-gam)]);
R_H = R*[x_H; y_H];
Y_H = R_H(2);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Potential, Kinetic, and Total Energy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


T = 0.5*(M*dot(v_H,v_H) + I*dot(omega1,omega1));
V = simplify(M*g*Y_H); %potential is positive because com is above reference point
L = T-V;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Derive equations of motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q = [x y theta1];
qdot = [vx vy omega1];
qddot = [ax ay alpha1];

for ii=1:3
    dLdqdot(ii) = diff(L,qdot(ii));
    ddt_dLdqdot(ii) = diff(dLdqdot(ii),q(1))*qdot(1) + diff(dLdqdot(ii),qdot(1))*qddot(1)+...
                     diff(dLdqdot(ii),q(2))*qdot(2) + diff(dLdqdot(ii),qdot(2))*qddot(2)+...
                     diff(dLdqdot(ii),q(3))*qdot(3) + diff(dLdqdot(ii),qdot(3))*qddot(3);
    dLdq(ii) = diff(L,q(ii));

    EOM(ii) = ddt_dLdqdot(ii) - dLdq(ii);
end

%%%%%%% ss stuff starts now %%%%%%%
%here A_ss is floating base 
A_ss = jacobian(EOM,[ax ay alpha1]);
b_ss(1,1) = -subs(EOM(1),[ax ay alpha1],[0 0 0]);
b_ss(2,1) = -subs(EOM(2),[ax ay alpha1],[0 0 0]);
b_ss(3,1) = -subs(EOM(3),[ax ay alpha1],[0 0 0]);


disp('copy paste in MATLAB');
disp(' ');
disp('ss equations start here');
disp(' ');

%We only use the elements from alpha1 and alpha2 (row, columns 3 and 4)
disp(['A_ss = ', char(simplify(A_ss(3,3))), ';'])
disp(' ');
disp(['b_ss = ', char(simplify(b_ss(3,1))), ';'])
disp(' ');
disp('alpha1 = A_ss\b_ss;');
disp(' ');
disp(' ');

%%%%%%% reaction forces %%%%%
Rx = subs(EOM(1),[ax ay],[0 0]);
Ry = subs(EOM(2),[ax ay],[0 0]);
disp(['Rx = ', char(simplify(Rx)), ';'])
disp(['Ry = ', char(simplify(Ry)), ';'])

%%%%%%% hs stuff starts now %%%%%%%
J_sw =  jacobian([x_C2, y_C2],[x y theta1]);
J_n_sw = subs(J_sw,theta1,theta1_n);
A_n_hs = subs(A_ss,theta1,theta1_n);

disp(' ');
disp('hs equations start here');
disp(' ');

disp(['J11 = ', char(simplify(J_n_sw(1,1))), ';'])
disp(['J12 = ', char(simplify(J_n_sw(1,2))), ';'])
disp(['J13 = ', char(simplify(J_n_sw(1,3))), ';'])
disp(['J21 = ', char(simplify(J_n_sw(2,1))), ';'])
disp(['J22 = ', char(simplify(J_n_sw(2,2))), ';'])
disp(['J23 = ', char(simplify(J_n_sw(2,3))), ';'])
disp('J = [J11 J12 J13; J21 J22 J23];');
disp(' ');

disp(['A11 = ', char(simplify(A_n_hs(1,1))), ';'])
disp(['A12 = ', char(simplify(A_n_hs(1,2))), ';'])
disp(['A13 = ', char(simplify(A_n_hs(1,3))), ';'])

disp(['A21 = ', char(simplify(A_n_hs(2,1))), ';'])
disp(['A22 = ', char(simplify(A_n_hs(2,2))), ';'])
disp(['A23 = ', char(simplify(A_n_hs(2,3))), ';'])

disp(['A31 = ', char(simplify(A_n_hs(3,1))), ';'])
disp(['A32 = ', char(simplify(A_n_hs(3,2))), ';'])
disp(['A33 = ', char(simplify(A_n_hs(3,3))), ';'])

disp('A_n_hs = [A11 A12 A13; A21 A22 A23; A31 A32 A33];');
disp(' ');

disp('X_n_hs = [0 0 omega1_n]'';'); %[vx_stance vy_stance omega1 omega2]
disp('b_hs = [A_n_hs*X_n_hs; 0; 0];'); %[momentum before footstrike = A_n_hs*X_n_hs; v_swing_foot_after_foot_strike = 0 0 
disp('A_hs = [A_n_hs -J'' ; J zeros(2,2)];');
disp('X_hs = A_hs\b_hs;');
disp('omega1 = X_hs(3);');
disp(' ');
disp(' ');

