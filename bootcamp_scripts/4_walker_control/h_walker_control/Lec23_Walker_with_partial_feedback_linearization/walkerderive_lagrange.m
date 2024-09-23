%%%% derive passive walphaker with roomegand feet
%%% Pranav A. Bhounsule, 21 April 2009. pab47@cornell.edu
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

i=[1 0 0];
j=[0 1 0];
k=[0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 Reference Frames                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X1 = sin(theta1)*i-cos(theta1)*j; %unit vector alpha along stance leg, direction is from hip downwards
%Y1 = cos(theta1)*i+sin(theta1)*j; %unit vector perpendialphaar to stance leg

X2 = sin(theta1+theta2)*i - cos(theta1+theta2)*j; %unit vector alphaong swing leg, direction is from hip downwards
%Y2 = cos(theta1+theta2)*i + sin(theta1+theta2)*j; %unit vector perpendicomegalphaar to swing leg

J  = -sin(gam)*i+cos(gam)*j; %unit vector point vertically omega in the world frame (along gravity)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Position Vectors                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Original position vectors
r_H_G1 = c*X1;
r_H_G2 = c*X2;
r_C1_H = -l*X1;
r_C2_H = -l*X2;

%Derived position vectors
r_C1_G1 = r_C1_H + r_H_G1;
r_C1_G2 = r_C1_H + r_H_G2;
r_C1_C2 = r_C1_H - r_C2_H;

r_H_C2 = -r_C2_H;

%Position vectors for heelstrike
r_C2_G1 = r_C2_H + r_H_G1;
r_C2_G2 = r_C2_H + r_H_G2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%        Angular Velocities     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
om1 = omega1*k;
om2 = (omega1+omega2)*k;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Constraints, Linear Velocities and Accelerations %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
theta = [theta1; theta2];
omega = [omega1; omega2];

%find position of hip joint (xh, yh) then differentiate to find velocity
xh = x-l*sin(theta1);
yh = y+l*cos(theta1);
xhdot  = jacobian(xh,theta1)*omega1+jacobian(xh,x)*vx+jacobian(xh,y)*vy;
yhdot  = jacobian(yh,theta1)*omega1+jacobian(yh,x)*vx+jacobian(yh,y)*vy;

v_H  = xhdot*i+yhdot*j; 
v_G1 = v_H +cross(om1,r_H_G1);
v_G2 = v_H +cross(om2,r_H_G2);
v_C2 = v_H + cross(om2,r_H_C2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Position vectors for potential energy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Get positions of masses wrt ramp frame
xH = xh;
yH = yh;
xG1 = xH+dot(r_H_G1,i);
yG1 = yH+dot(r_H_G1,j);
xG2 = xH+dot(r_H_G2,i);
yG2 = yH+dot(r_H_G2,j);

%Get positions of masses wrt to global frame
Y_H = yH*cos(gam) - xH*sin(gam);
Y_G1 = yG1*cos(gam) - xG1*sin(gam); 
Y_G2 = yG2*cos(gam) - xG2*sin(gam);
% X_H = xH*cos(gam) + yH*sin(gam);
% X_G1 = xG1*cos(gam) + yG1*sin(gam);
% X_G2 = xG2*cos(gam) + yG2*sin(gam);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Potential, Kinetic, and Total Energy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T = 0.5*(simplify(m*dot(v_G1,v_G1) + m*dot(v_G2,v_G2) + M*dot(v_H,v_H) + I*(dot(om1,om1) + dot(om2,om2))));
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
M_ss = jacobian(EOM,[ax ay alpha1 alpha2]);
N_ss(1,1) = subs(EOM(1),[ax ay alpha1 alpha2],[0 0 0 0]);
N_ss(2,1) = subs(EOM(2),[ax ay alpha1 alpha2],[0 0 0 0]);
N_ss(3,1) = subs(EOM(3),[ax ay alpha1 alpha2],[0 0 0 0]);
N_ss(4,1) = subs(EOM(4),[ax ay alpha1 alpha2],[0 0 0 0]);

disp('copy paste in MATLAB');
disp(' ');
disp('ss equations start here');
disp(' ');

%We only use the elements from alpha1 and alpha2 (row, columns 3 and 4)
disp(['M11 = ', char(simplify(M_ss(3,3))), ';'])
disp(['M12 = ', char(simplify(M_ss(3,4))), ';'])
disp(['M21 = ', char(simplify(M_ss(4,3))), ';'])
disp(['M22 = ', char(simplify(M_ss(4,4))), ';'])
disp('Ms = [M11 M12; M21 M22];');
disp(' ');
disp(['N1 = ', char(simplify(N_ss(3,1))), ';'])
disp(['N2 = ', char(simplify(N_ss(4,1))), ';'])
disp('Ns = [N1; N2];');
disp(' ');
disp('B = [0; 1];');
disp(' ');
%%%% Equation is Ms alpha + Ns = Bu
disp('alpha = Ms\(-Ns+B*u);');
disp(' ');
disp(' ');


%%%%%%% hs stuff starts now %%%%%%%
x_C2 = xh + dot(r_H_C2,i);
y_C2 = yh + dot(r_H_C2,j);
J_sw =  jacobian([x_C2, y_C2],[x y theta1 theta2]);
J_n_sw = subs(J_sw,[theta1 theta2],[theta1_n theta2_n]);
M_n_hs = subs(M_ss,[theta1 theta2],[theta1_n theta2_n]);

disp(' ');
disp('hs equations start here');
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

disp(['M11 = ', char(simplify(M_n_hs(1,1))), ';'])
disp(['M12 = ', char(simplify(M_n_hs(1,2))), ';'])
disp(['M13 = ', char(simplify(M_n_hs(1,3))), ';'])
disp(['M14 = ', char(simplify(M_n_hs(1,4))), ';'])

disp(['M21 = ', char(simplify(M_n_hs(2,1))), ';'])
disp(['M22 = ', char(simplify(M_n_hs(2,2))), ';'])
disp(['M23 = ', char(simplify(M_n_hs(2,3))), ';'])
disp(['M24 = ', char(simplify(M_n_hs(2,4))), ';'])

disp(['M31 = ', char(simplify(M_n_hs(3,1))), ';'])
disp(['M32 = ', char(simplify(M_n_hs(3,2))), ';'])
disp(['M33 = ', char(simplify(M_n_hs(3,3))), ';'])
disp(['M34 = ', char(simplify(M_n_hs(3,4))), ';'])

disp(['M41 = ', char(simplify(M_n_hs(4,1))), ';'])
disp(['M42 = ', char(simplify(M_n_hs(4,2))), ';'])
disp(['M43 = ', char(simplify(M_n_hs(4,3))), ';'])
disp(['M44 = ', char(simplify(M_n_hs(4,4))), ';'])
disp('M_n_hs = [M11 M12 M13 M14; M21 M22 M23 M24; M31 M32 M33 M34; M41 M42 M43 M44];');
disp(' ');

disp('X_n_hs = [0 0 omega1_n omega2_n]'';'); %[vx_stance vy_stance omega1 omega2]
disp('b_hs = [M_n_hs*X_n_hs; 0; 0];'); %[momentum before footstrike = A_n_hs*X_n_hs; v_swing_foot_after_foot_strike = 0 0 
disp('M_hs = [M_n_hs -J'' ; J zeros(2,2)];');
disp('X_hs = M_hs\b_hs;');
disp('omega(1) = X_hs(3)+X_hs(4); omega(2) = -X_hs(4);');
disp(' ');
disp(' ');


