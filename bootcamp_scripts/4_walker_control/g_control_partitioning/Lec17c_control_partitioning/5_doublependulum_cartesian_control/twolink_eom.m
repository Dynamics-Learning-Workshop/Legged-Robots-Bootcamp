clc
clear all
close all


%%%%% symbolic quantities
syms theta1 theta2 real
syms theta1dot theta2dot real
syms theta1ddot theta2ddot real %angular acceleration
syms m1 I1 m2 I2 g real
syms l1 l2 real
syms T1 T2 real %torques

c1 = l1/2;
c2 = l2/2;

%%%%% position vectors %%%%
x_G1 = c1*cos(theta1); y_G1 = c1*sin(theta1);
x_G2 = l1*cos(theta1) + c2*cos(theta1+theta2); y_G2 = l1*sin(theta1)+c2*sin(theta1+theta2);

%%%%% velocity vectors %%%%%%
%%% I used jacobian instead of diff.
v_G1_x = jacobian(x_G1,[theta1 theta2])*[theta1dot theta2dot]'; %jacobian will be explained in the next few lectures
v_G1_y = jacobian(y_G1,[theta1 theta2])*[theta1dot theta2dot]'; 
v_G2_x = jacobian(x_G2,[theta1 theta2])*[theta1dot theta2dot]';
v_G2_y = jacobian(y_G2,[theta1 theta2])*[theta1dot theta2dot]';
v_G1 = [v_G1_x; v_G1_y];
v_G2 = [v_G2_x; v_G2_y];

 
%%%% lagrangian %%
%%%% v_G1 = [xdot_G1 ydot_G1]
%%% v_G1' * v_G1 = xdot_G1^2 + ydot_G1^2 %%% (Check this)
T = 0.5*m1*(v_G1'*v_G1) + 0.5*m2*(v_G2'*v_G2) + 0.5*I1*theta1dot*theta1dot + 0.5*I2*(theta1dot+theta2dot)*(theta1dot+theta2dot);
V = m1*g*y_G1 + m2*g*y_G2;
L = T-V;
disp('copy paste energy in the code');
disp(['KE(i) = ',char(T),';']);
disp(['PE(i) = ',char(V),';']);
disp(['TE(i) = KE(i)+PE(i);']);
disp(' ');

%Derive equations of motion using Euler-Lagrange method
q = [theta1 theta2];
qdot = [theta1dot theta2dot];
qddot = [theta1ddot theta2ddot];

for ii=1:2
    dLdqdot(ii) = diff(L,qdot(ii));
    ddt_dLdqdot(ii) = diff(dLdqdot(ii),q(1))*qdot(1) + diff(dLdqdot(ii),qdot(1))*qddot(1)+...
                     diff(dLdqdot(ii),q(2))*qdot(2) + diff(dLdqdot(ii),qdot(2))*qddot(2);
    dLdq(ii) = diff(L,q(ii));

    EOM(ii) = ddt_dLdqdot(ii) - dLdq(ii);
end
EOM1 = EOM(1) - T1;
EOM2 = EOM(2) - T2;

%Finally, we will simplify the expression.

%To get G1 and G2, we put Xdot, Xddot to zero.
G1 = subs(EOM1,[theta1ddot theta2ddot theta1dot theta2dot],[0 0 0 0]); 
G1 = simplify(G1);
G2 = subs(EOM2,[theta1ddot theta2ddot theta1dot theta2dot],[0 0 0 0]); 
G2 = simplify(G2);

%Display for easy copy-paste
disp(['G1 = ', char(G1),';']);
disp(['G2 = ', char(G2),';']);

%To get C terms, we put theta1ddot theta2ddot each equal to zero and subtract from G
C1 = subs(EOM1,[theta1ddot theta2ddot],[0 0])-G1; 
C1 = simplify(C1);

C2 = subs(EOM2,[theta1ddot theta2ddot],[0 0])-G2; 
C2 = simplify(C2);

%Display for easy copy-paste
disp(['C1 = ', char(C1),';']);
disp(['C2 = ', char(C2),';']);

%Finally to get M, we subtract C and G from EOM 
M1 = EOM1 - C1 - G1; %M1; = M11(X)*theta1ddot+M12(X)*theta2ddot
M2 = EOM2 - C2 - G2; %M2; = M21(X)*theta1ddot+M22(X)*theta2ddot

%Now we will extract M11, M12, M21, M22 by putting various values for theta1ddot and theta2ddot
M11 = subs(M1,[theta1ddot theta2ddot],[1 0]); M11 = simplify(M11);
M12 = subs(M1,[theta1ddot theta2ddot],[0 1]); M12 = simplify(M12);
M21 = subs(M2,[theta1ddot theta2ddot],[1 0]); M21 = simplify(M21);
M22 = subs(M2,[theta1ddot theta2ddot],[0 1]); M22 = simplify(M22);
disp(['M11 = ', char(M11),';']);
disp(['M12 = ', char(M12),';']);
disp(['M21 = ', char(M21),';']);
disp(['M22 = ', char(M22),';']);

disp('Equations solved');
disp('Copy paste Ms, Cs, and Gs');

x_tip = l1*cos(theta1) + l2*cos(theta1+theta2); 
y_tip = l1*sin(theta1) + l2*sin(theta1+theta2);
J = jacobian([x_tip, y_tip],[theta1 theta2]);
Jdot(1,1) = simplify(diff(J(1,1),theta1)*theta1dot+diff(J(1,1),theta2)*theta2dot);
Jdot(1,2) = simplify(diff(J(1,2),theta1)*theta1dot+diff(J(1,2),theta2)*theta2dot);
Jdot(2,1) = simplify(diff(J(2,1),theta1)*theta1dot+diff(J(2,1),theta2)*theta2dot);
Jdot(2,2) = simplify(diff(J(2,2),theta1)*theta1dot+diff(J(2,2),theta2)*theta2dot);

disp(' ');
disp('Copy paste in jacobian_endeffector.m');
disp(['J(1,1) = ', char(J(1,1)),';']);
disp(['J(1,2) = ', char(J(1,2)),';']);
disp(['J(2,1) = ', char(J(2,1)),';']);
disp(['J(2,2) = ', char(J(2,2)),';']);
disp(' ');
disp('Copy paste in jacobiandot_endeffector.m');
disp(['Jdot(1,1) = ', char(Jdot(1,1)),';']);
disp(['Jdot(1,2) = ', char(Jdot(1,2)),';']);
disp(['Jdot(2,1) = ', char(Jdot(2,1)),';']);
disp(['Jdot(2,2) = ', char(Jdot(2,2)),';']);


