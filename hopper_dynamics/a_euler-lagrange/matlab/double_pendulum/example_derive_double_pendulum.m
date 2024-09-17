clc
clear all
close all

%%%%% symbolic quantities
syms theta1 theta2 real
syms omega1 omega2 real
syms alpha1 alpha2 real
syms m1 I1 m2 I2 g real
syms c1 c2 l real

%%%%% position vectors %%%%
cos1 = cos(3*pi/2 + theta1); sin1 = sin(3*pi/2 + theta1);
cos1 = simplify(cos1); sin1 = simplify(sin1);
R01 = [cos1 -sin1; sin1 cos1]; O01 = [0; 0];
H01 = [R01, O01; 0 0 1]; 
cos2 = cos(theta2); sin2 = sin(theta2);
R12 = [cos2 -sin2; sin2 cos2]; O12 = [l; 0];
H12 = [R12, O12; 0 0 1];

G1 = H01*[c1 0 1]';
G2 = H01*H12*[c2 0 1]';
x_G1 = G1(1);
y_G1 = G1(2);
x_G2 = G2(1);
y_G2 = G2(2);

% x_G1 = c1*sin(theta1); y_G1 = -c1*cos(theta1);
% x_G2 = l*sin(theta1) + c2*sin(theta1+theta2); y_G2 = -l*cos(theta1) -c2*cos(theta1+theta2);

%%%%% velocity vectors %%%%%%
v_G1_x = jacobian(x_G1,[theta1 theta2])*[omega1 omega2]'; %jacobian will be explained in the next few lectures
v_G1_y = jacobian(y_G1,[theta1 theta2])*[omega1 omega2]'; 
v_G2_x = jacobian(x_G2,[theta1 theta2])*[omega1 omega2]';
v_G2_y = jacobian(y_G2,[theta1 theta2])*[omega1 omega2]';
v_G1 = [v_G1_x; v_G1_y];
v_G2 = [v_G2_x; v_G2_y];

 
%%%% lagrangian %%
T = 0.5*m1*(v_G1'*v_G1) + 0.5*m2*(v_G2'*v_G2) + 0.5*I1*omega1*omega1 + 0.5*I2*(omega1+omega2)*(omega1+omega2);
V = m1*g*y_G1 + m2*g*y_G2;
L = T-V;
disp('copy paste energy in the code');
disp(['KE(i) = ',char(T),';']);
disp(['PE(i) = ',char(V),';']);
disp(['TE(i) = KE(i)+PE(i);']);
disp(' ');

%Derive equations of motion using Euler-Lagrange method
q = [theta1 theta2];
qdot = [omega1 omega2];
qddot = [alpha1 alpha2];

for ii=1:2
    dLdqdot(ii) = diff(L,qdot(ii));
    ddt_dLdqdot(ii) = diff(dLdqdot(ii),q(1))*qdot(1) + diff(dLdqdot(ii),qdot(1))*qddot(1)+...
                     diff(dLdqdot(ii),q(2))*qdot(2) + diff(dLdqdot(ii),qdot(2))*qddot(2);
    dLdq(ii) = diff(L,q(ii));

    EOM(ii) = ddt_dLdqdot(ii) - dLdq(ii);
end

%%%%%%%%% collecting equations as M(th)thddot + C(th,thdot)thdot + G(th) = tau
M = jacobian(EOM,[alpha1 alpha2]);
N(1,1) = subs(EOM(1),[alpha1 alpha2],[0 0]);
N(2,1) = subs(EOM(2),[alpha1 alpha2],[0 0]);

G(1,1) = subs(N(1,1),[omega1 omega2],[0 0]);
G(2,1) = subs(N(2,1),[omega1 omega2],[0 0]);
C(1,1) = N(1,1) - G(1,1);
C(2,1) = N(2,1) - G(2,1);


disp('copy paste in MATLAB');
disp(' ');

disp(['M11 = ', char(M(1,1)), ';'])
disp(['M12 = ', char(M(1,2)), ';'])
disp(['M21 = ', char(M(2,1)), ';'])
disp(['M22 = ', char(M(2,2)), ';'])

disp(['C1 = ', char(C(1,1)), ';'])
disp(['C2 = ', char(C(2,1)), ';'])

disp(['G1 = ', char(G(1,1)), ';'])
disp(['G2 = ', char(G(2,1)), ';'])

disp('M = [M11 M12; M21 M22];');
disp('C = [C1; C2];');
disp('G = [G1; G2];');
disp('thetaddot = M\(-G-C);');
