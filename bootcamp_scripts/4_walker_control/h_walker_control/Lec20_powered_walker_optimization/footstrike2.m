%===================================================================
function zplus=footstrike2(t,z,walker,P)      
%===================================================================

theta1_n = z(1);   omega1_n = z(2);                         
theta2_n = z(3);   omega2_n = z(4);                         
                      
theta1 = theta1_n + theta2_n;                         
theta2 = -theta2_n;                                       

M = walker.M;  m = walker.m; I = walker.I;   
l = walker.l;  c = walker.c;  

%P = walker.control.P;

%derived from poweredderive_newton
% A11 = 2*I + M*l^2 + 2*c^2*m + 2*l^2*m - 2*c*l*m - 2*c*l*m*cos(theta2);
% A12 = I + M*l^2 + c^2*m + 2*l^2*m - 2*c*l*m - c*l*m*cos(theta2);
% A21 = I + c^2*m - c*l*m*cos(theta2);
% A22 = -c*l*m*cos(theta2);
% b1 = 2*I*omega1_n + I*omega2_n - P*l*sin(theta2_n) + 2*c^2*m*omega1_n + c^2*m*omega2_n - 2*c*l*m*omega1_n - c*l*m*omega2_n + M*l^2*omega1_n*cos(theta2_n) + 2*l^2*m*omega1_n*cos(theta2_n) - 2*c*l*m*omega1_n*cos(theta2_n);
% b2 = omega1_n*(I + c^2*m - c*l*m);
% A_hs = [A11 A12; A21 A22];
% b_hs = [b1; b2];
% X_hs = A_hs\b_hs;
% omega(1) = X_hs(1)+X_hs(2);  omega(2) = -X_hs(2);
%  

%derived from poweredderive_lagrange
J11 = 1;
J12 = 0;
J13 = 0;
J14 = 0;
J21 = 0;
J22 = 1;
J23 = 0;
J24 = 0;
J_st = [J11 J12 J13 J14; J21 J22 J23 J24];
 
P_st_x = -P*sin(theta1_n);
P_st_y = P*cos(theta1_n);
P_st = [P_st_x; P_st_y];
 
J11 = 1;
J12 = 0;
J13 = l*(cos(theta1_n + theta2_n) - cos(theta1_n));
J14 = l*cos(theta1_n + theta2_n);
J21 = 0;
J22 = 1;
J23 = l*(sin(theta1_n + theta2_n) - sin(theta1_n));
J24 = l*sin(theta1_n + theta2_n);
J_sw = [J11 J12 J13 J14; J21 J22 J23 J24];
 
A11 = M + 2*m;
A12 = 0;
A13 = c*m*cos(theta1_n) - 2*l*m*cos(theta1_n) + c*m*cos(theta1_n + theta2_n) - M*l*cos(theta1_n);
A14 = c*m*cos(theta1_n + theta2_n);
A21 = 0;
A22 = M + 2*m;
A23 = c*m*sin(theta1_n) - M*l*sin(theta1_n) - 2*l*m*sin(theta1_n) + c*m*sin(theta1_n + theta2_n);
A24 = c*m*sin(theta1_n + theta2_n);
A31 = c*m*cos(theta1_n) - 2*l*m*cos(theta1_n) + c*m*cos(theta1_n + theta2_n) - M*l*cos(theta1_n);
A32 = c*m*sin(theta1_n) - M*l*sin(theta1_n) - 2*l*m*sin(theta1_n) + c*m*sin(theta1_n + theta2_n);
A33 = 2*I + M*l^2 + 2*c^2*m + 2*l^2*m - 2*c*l*m - 2*c*l*m*cos(theta2_n);
A34 = I + c^2*m - c*l*m*cos(theta2_n);
A41 = c*m*cos(theta1_n + theta2_n);
A42 = c*m*sin(theta1_n + theta2_n);
A43 = I + c^2*m - c*l*m*cos(theta2_n);
A44 = I + c^2*m;
A_n_hs = [A11 A12 A13 A14; A21 A22 A23 A24; A31 A32 A33 A34; A41 A42 A43 A44];
 
X_n_hs = [0 0 omega1_n omega2_n]';
b_hs = [A_n_hs*X_n_hs + J_st'*P_st; 0; 0];
A_hs = [A_n_hs -J_sw' ; J_sw zeros(2,2)];
X_hs = A_hs\b_hs;
omega(1) = X_hs(3)+X_hs(4); omega(2) = -X_hs(4);

zplus = [theta1 omega(1) theta2 omega(2)];                     
