clc;
clear all;

syms M m I real %Mass Hip, leg, Inertia
syms c l real % Distances as defined in figures
syms gam g real %Slope of ramp, gravity
syms theta1 theta2 real %Angles as defined in figures 
syms omega1 omega2 real %Angular velocity
syms alpha1 alpha2 real%Angular Acceleration
syms theta1_n theta2_n real %angles before heelstrike
syms omega1_n omega2_n real %velocities before heelstrike
syms x y real %position of the stance leg
syms vx vy real %velocity of the stance leg
syms ax ay real %acceleration of the stance leg


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Position Vectors                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% position vectors %%%%%%%
R01 = simplify([cos(pi/2+theta1) -sin(pi/2+theta1); 
       sin(pi/2+theta1)  cos(pi/2+theta1)]);
R12 = simplify([cos(-pi+theta2) -sin(-pi+theta2); 
                sin(-pi+theta2)  cos(-pi+theta2)]);
O01 = [x; y];
O12 = [l; 0];
H01 = [R01 O01; 0 0 1];
H12 = [R12 O12; 0 0 1];

r_C1 = [x; y];

R_H = H01*[l; 0; 1];
r_H = R_H(1:2);
x_H = r_H(1); y_H = r_H(2);

R_G1 = H01*[(l-c); 0; 1];
r_G1 = R_G1(1:2);
x_G1 = r_G1(1); y_G1 = r_G1(2);

R_G2 = H01*H12*[c; 0; 1];
r_G2 = simplify(R_G2(1:2));
x_G2 = r_G2(1); y_G2 = r_G2(2);

R_C2 = H01*H12*[l; 0; 1];
r_C2 = simplify(R_C2(1:2));
x_C2 = r_C2(1); y_C2 = r_C2(2);


%%%%% velocity vectors %%%%%%
v_H_x = jacobian(x_H,[x y theta1 theta2])*[vx vy omega1 omega2]'; 
v_H_y = jacobian(y_H,[x y theta1 theta2])*[vx vy omega1 omega2]';
v_G1_x = jacobian(x_G1,[x y theta1 theta2])*[vx vy omega1 omega2]'; 
v_G1_y = jacobian(y_G1,[x y theta1 theta2])*[vx vy omega1 omega2]'; 
v_G2_x = jacobian(x_G2,[x y theta1 theta2])*[vx vy omega1 omega2]';
v_G2_y = jacobian(y_G2,[x y theta1 theta2])*[vx vy omega1 omega2]';
v_H = [v_H_x; v_H_y];
v_G1 = [v_G1_x; v_G1_y];
v_G2 = [v_G2_x; v_G2_y];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Position vectors for potential energy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Get positions of masses wrt to global frame
R = simplify([cos(-gam) -sin(-gam); 
              sin(-gam)  cos(-gam)]);
R_H = R*[x_H; y_H];
R_G1 = R*[x_G1; y_G1];
R_G2 = R*[x_G2; y_G2];

Y_H = R_H(2);
Y_G1 = R_G1(2);
Y_G2 = R_G2(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Potential, Kinetic, and Total Energy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


T = 0.5*(simplify(m*dot(v_G1,v_G1) + m*dot(v_G2,v_G2) + M*dot(v_H,v_H) + ...
       I*(dot(omega1,omega1) + dot(omega1+omega2,omega1+omega2))));
V = simplify(m*g*Y_G1+m*g*Y_G2+M*g*Y_H); %potential is positive because com is above reference point
L = T-V;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Derive equations of motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q = [x y theta1 theta2];
qdot = [vx vy omega1 omega2];
qddot = [ax ay alpha1 alpha2];

for ii=1:4
    dLdqdot(ii) = diff(L,qdot(ii));
    ddt_dLdqdot(ii) = diff(dLdqdot(ii),q(1))*qdot(1) + diff(dLdqdot(ii),qdot(1))*qddot(1)+...
                     diff(dLdqdot(ii),q(2))*qdot(2) + diff(dLdqdot(ii),qdot(2))*qddot(2)+...
                     diff(dLdqdot(ii),q(3))*qdot(3) + diff(dLdqdot(ii),qdot(3))*qddot(3)+...
                     diff(dLdqdot(ii),q(4))*qdot(4) + diff(dLdqdot(ii),qdot(4))*qddot(4);
    dLdq(ii) = diff(L,q(ii));

    EOM(ii) = ddt_dLdqdot(ii) - dLdq(ii);
end

%%%%%%% ss stuff starts now %%%%%%%
%here A_ss is floating base 
A_ss = jacobian(EOM,[ax ay alpha1 alpha2]);
b_ss(1,1) = -subs(EOM(1),[ax ay alpha1 alpha2],[0 0 0 0]);
b_ss(2,1) = -subs(EOM(2),[ax ay alpha1 alpha2],[0 0 0 0]);
b_ss(3,1) = -subs(EOM(3),[ax ay alpha1 alpha2],[0 0 0 0]);
b_ss(4,1) = -subs(EOM(4),[ax ay alpha1 alpha2],[0 0 0 0]);
% A = is the mass matrix

disp('copy paste in MATLAB');
disp(' ');
disp('ss equations start here');
disp(' ');

%We only use the elements from alpha1 and alpha2 (row, columns 3 and 4)
disp(['A11 = ', char(simplify(A_ss(3,3))), ';'])
disp(['A12 = ', char(simplify(A_ss(3,4))), ';'])
disp(['A21 = ', char(simplify(A_ss(4,3))), ';'])
disp(['A22 = ', char(simplify(A_ss(4,4))), ';'])
disp('A_ss = [A11 A12; A21 A22];');
disp(' ');
disp(['b1 = ', char(simplify(b_ss(3,1))), ';'])
disp(['b2 = ', char(simplify(b_ss(4,1))), ';'])
disp('b_ss = [b1; b2];');
disp(' ');
disp('alpha = A_ss\b_ss;');
disp(' ');
disp(' ');


%%%%%%% hs stuff starts now %%%%%%%
J_sw =  jacobian([x_C2, y_C2],[x y theta1 theta2]);
J_n_sw = subs(J_sw,[theta1 theta2],[theta1_n theta2_n]);
A_n_hs = subs(A_ss,[theta1 theta2],[theta1_n theta2_n]);

disp(' ');
disp('foot strike equations start here');
disp(' ');

disp(['J11 = ', char(simplify(J_n_sw(1,1))), ';'])
disp(['J12 = ', char(simplify(J_n_sw(1,2))), ';'])
disp(['J13 = ', char(simplify(J_n_sw(1,3))), ';'])
disp(['J14 = ', char(simplify(J_n_sw(1,4))), ';'])
disp(['J21 = ', char(simplify(J_n_sw(2,1))), ';'])
disp(['J22 = ', char(simplify(J_n_sw(2,2))), ';'])
disp(['J23 = ', char(simplify(J_n_sw(2,3))), ';'])
disp(['J24 = ', char(simplify(J_n_sw(2,4))), ';'])
disp('J = [J11 J12 J13 J14; J21 J22 J23 J24];');
disp(' ');

disp(['A11 = ', char(simplify(A_n_hs(1,1))), ';'])
disp(['A12 = ', char(simplify(A_n_hs(1,2))), ';'])
disp(['A13 = ', char(simplify(A_n_hs(1,3))), ';'])
disp(['A14 = ', char(simplify(A_n_hs(1,4))), ';'])

disp(['A21 = ', char(simplify(A_n_hs(2,1))), ';'])
disp(['A22 = ', char(simplify(A_n_hs(2,2))), ';'])
disp(['A23 = ', char(simplify(A_n_hs(2,3))), ';'])
disp(['A24 = ', char(simplify(A_n_hs(2,4))), ';'])

disp(['A31 = ', char(simplify(A_n_hs(3,1))), ';'])
disp(['A32 = ', char(simplify(A_n_hs(3,2))), ';'])
disp(['A33 = ', char(simplify(A_n_hs(3,3))), ';'])
disp(['A34 = ', char(simplify(A_n_hs(3,4))), ';'])

disp(['A41 = ', char(simplify(A_n_hs(4,1))), ';'])
disp(['A42 = ', char(simplify(A_n_hs(4,2))), ';'])
disp(['A43 = ', char(simplify(A_n_hs(4,3))), ';'])
disp(['A44 = ', char(simplify(A_n_hs(4,4))), ';'])
disp('A_n_hs = [A11 A12 A13 A14; A21 A22 A23 A24; A31 A32 A33 A34; A41 A42 A43 A44];');
disp(' ');

disp('X_n_hs = [0 0 omega1_n omega2_n]'';'); %[vx_stance vy_stance omega1 omega2]
disp('b_hs = [A_n_hs*X_n_hs; 0; 0];'); %[momentum before footstrike = A_n_hs*X_n_hs; v_swing_foot_after_foot_strike = 0 0 
disp('A_hs = [A_n_hs -J'' ; J zeros(2,2)];');
disp('X_hs = A_hs\b_hs;');
disp('omega(1) = X_hs(3)+X_hs(4); omega(2) = -X_hs(4);');
disp(' ');
disp(' ');