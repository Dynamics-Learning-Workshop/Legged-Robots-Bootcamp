clc;
clear all;
syms x y z real %position of the stance leg
syms phi theta psi real
syms phi_lh theta_lh psi_lh theta_lk real
syms phi_rh theta_rh psi_rh theta_rk real

syms xd yd zd real
syms phid thetad psid real 
syms phi_lhd theta_lhd psi_lhd theta_lkd real
syms phi_rhd theta_rhd psi_rhd theta_rkd real

syms xdd ydd zdd real
syms phidd thetadd psidd real
syms phi_lhdd theta_lhdd psi_lhdd theta_lkdd real
syms phi_rhdd theta_rhdd psi_rhdd theta_rkdd real

syms w l0 l1 l2 real
syms g real
syms P real
syms Ibx Iby Ibz real
syms Itx Ity Itz real
syms Icx Icy Icz real
syms mb mt mc real

%%%%%% total dofs 
dof=14;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Position Vectors                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% world frame %%%%%%
T01 = [1 0 0 x; 
       0 1 0 y; 
       0 0 1 z; 
       0 0 0 1];
R01 = T01(1:3,1:3);

%roll
r = [0 0 0];
u = [1 0 0];
T12 = revolute(phi,r,u); %T_UR_2_O
R12 = T12(1:3,1:3); %R_UR_2_O
omega_12 = phid*u'; %omega_UR_2_O

%pitch
r = [0 0 0];
u = [0 1 0];
T23 = revolute(theta,r,u); %T_UP_2_UR
R23 = T23(1:3,1:3); %R_UP_2_UR
omega_23 = thetad*u'; %omega_UP_2_UR

%yaw
r = [0 0 0];
u = [0 0 1];
T34 = revolute(psi,r,u); %T_UY_2_UP
R34 = T34(1:3,1:3); %R_UY_2_UP
omega_34 = psid*u'; %omega_UY_2_UP

%%%% left side %%%
r = [0 w 0];
u = [0 0 1];
T45l = revolute(psi_lh,r,u); %T_LHY_2_UY
R45l = T45l(1:3,1:3); %R_LHY_2_UY
omega_45l = psi_lhd*u'; %omega_LHY_2_UY

%R15l = R14*R45l;
r = [0 w 0];
u = [1 0 0];
T56l = revolute(phi_lh,r,u); %T_LHR_2_LHY
R56l = T56l(1:3,1:3); %R_LHR_2_LHY
omega_56l = phi_lhd*u'; %omega_LHR_2_LHY

r = [0 w 0];
u = [0 -1 0];
T67l = revolute(theta_lh,r,u); %T_LHP_2_LHR
R67l = T67l(1:3,1:3); %R_LHP_2_LHR
omega_67l = theta_lhd*u'; %omega_LHP_2_LHR

r = [0 w -l1];
u = [0 -1 0];
T78l = revolute(theta_lk,r,u); %T_LK_2_LHP
R78l = T78l(1:3,1:3); %R_LK_2_LHP
omega_78l = theta_lkd*u'; %omega_LK_2_LHP


%%%% right side %%%
r = [0 -w 0];
u = [0 0 -1];
T45r = revolute(psi_rh,r,u); %T_RHY_2_UY
R45r = T45r(1:3,1:3); %R_RHY_2_UY
omega_45r = psi_rhd*u'; %omega_RHY_2_UY

r = [0 -w 0];
u = [-1 0 0];
T56r = revolute(phi_rh,r,u); %T_RHR_2_RHY
R56r = T56r(1:3,1:3); %R_RHR_2_RHY
omega_56r = phi_rhd*u'; %omega_RHR_2_RHY

r = [0 -w 0];
u = [0 -1 0];
T67r = revolute(theta_rh,r,u); %T_RHP_2_RHR
R67r = T67r(1:3,1:3); %R_RHP_2_RHR
omega_67r = theta_rhd*u'; %omega_RHP_2_RHR

r = [0 -w -l1];
u = [0 -1 0];
T78r = revolute(theta_rk,r,u); %T_RK_2_RHP
R78r = T78r(1:3,1:3); %R_RK_2_RHP
omega_78r = theta_rkd*u'; %omega_RK_2_RHP

%%%%%% position vectors %%%%%%
%%%%% all joints %%%%
B = T01(1:3,4); %base
T04 = T01*T12*T23*T34;
H = simplify(T04*[0; 0; l0; 1]);
T07l = T01*T12*T23*T34*T45l*T56l*T67l;
LH = simplify(T07l*[0; w; 0; 1]);
T08l = T07l*T78l;
LK = simplify(T08l*[0; w; -l1; 1]);
LA = simplify(T08l*[0; w; -(l1+l2); 1]);
 
T07r = T01*T12*T23*T34*T45r*T56r*T67r;
RH = simplify(T07r*[0; -w; 0; 1]);
T08r = T07r*T78r;
RK = simplify(T08r*[0; -w; -l1; 1]);
RA = simplify(T08r*[0; -w; -(l1+l2); 1]);

%%%%% all center of mass %%%%
b = simplify(T04*[0; 0; 0.5*l0; 1]);
lt = simplify(T07l*[0; w; -0.5*l1; 1]);
lc = simplify(T08l*[0; w; -(l1+0.5*l2); 1]);
rt = simplify(T07r*[0; -w; -0.5*l1; 1]);
rc = simplify(T08r*[0; -w; -(l1+0.5*l2); 1]);

% f_joint = matlabFunction(B, H, LH,LK, LA, RH, RK, RA,b,rt,rc,lt,lc,...
%    'File','joint_locations','Outputs',{'B', 'H', 'LH','LK', 'LA', 'RH', 'RK', 'RA','b','rt','rc','lt','lc'});
% 
% f_foot = matlabFunction(LA(1:3), RA(1:3),...
%    'File','foot_positions','Outputs',{'r_LA', 'r_RA'});

pos_hip_l_stance = subs(-LA,[x y z],[0 0 0]);
pos_hip_r_stance = subs(-RA,[x y z],[0 0 0]);
f_pos_hip = matlabFunction(pos_hip_l_stance, pos_hip_r_stance,...
   'File','hip_positions','Outputs',{'pos_hip_l_stance', 'pos_hip_r_stance'});

%%%%%% collision detection condition %%%%%
disp(' ');
disp(['gstop = ',char(simplify(LA(3)-RA(3))),';']);
disp(' ');


===========================================================================
omega_13 = omega_12 + R12*omega_23;
R13 = R12*R23;
omega_14 = omega_13 + R13*omega_34;
R14 = R13*R34;

omega_15l = omega_14 + R14*omega_45l;
omega_15r = omega_14 + R14*omega_45r;

R15l = R14*R45l;
omega_16l = omega_15l + R15l*omega_56l;
R15r = R14*R45r;
omega_16r = omega_15r + R15r*omega_56r;
R16l = R15l*R56l;
omega_17l = omega_16l + R16l*omega_67l;
R16r = R15r*R56r;
omega_17r = omega_16r + R16r*omega_67r;
R17l = R16l*R67l;
omega_18l = omega_17l + R17l*omega_78l;
R17r = R16r*R67r;
omega_18r = omega_17r + R17r*omega_78r;
R18l = R17l*R78l;
R18r = R17r*R78r;

%%% angular velocity in body frame
omegaB_2 = omega_12;
omegaB_3 = omega_23 + R23'*omegaB_2;
omegaB_4 = omega_34 + R34'*omegaB_3;
% A = jacobian(omegaB_4,[phid,thetad,psid]) %used in sdfast to convert
% euler to body frame angular velocity for torso
omegaB_5l = omega_45l + R45l'*omegaB_4;
omegaB_6l = omega_56l + R56l'*omegaB_5l;
omegaB_7l = omega_67l + R67l'*omegaB_6l;
omegaB_8l = omega_78l + R78l'*omegaB_7l;
omegaB_5r = omega_45r + R45r'*omegaB_4;
omegaB_6r = omega_56r + R56r'*omegaB_5r;
omegaB_7r = omega_67r + R67r'*omegaB_6r;
omegaB_8r = omega_78r + R78r'*omegaB_7r;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

I_LA(1) = P*(LK(1)-LA(1))/l2;
I_LA(2) = P*(LK(2)-LA(2))/l2;
I_LA(3) = P*(LK(3)-LA(3))/l2;
I_RA(1) = P*(RK(1)-RA(1))/l2;
I_RA(2) = P*(RK(2)-RA(2))/l2;
I_RA(3) = P*(RK(3)-RA(3))/l2;
% f_impulse = matlabFunction(I_LA, I_RA,...
%    'File','foot_impulse','Outputs',{'I_LA', 'I_RA'});

%%%%%% velocity and acceleration vectors %%%%%%
q = [x y z phi theta psi phi_lh theta_lh psi_lh theta_lk phi_rh theta_rh psi_rh theta_rk];
qdot = [xd yd zd phid thetad psid phi_lhd theta_lhd psi_lhd theta_lkd phi_rhd theta_rhd psi_rhd theta_rkd];
qddot = [xdd ydd zdd phidd thetadd psidd phi_lhdd theta_lhdd psi_lhdd theta_lkdd phi_rhdd theta_rhdd psi_rhdd theta_rkdd];

for i=1:3
    pos = b(i);
    v_b(i) = jacobian(pos,q)*qdot';
    %a_b(i) = jacobian(pos,q)*qdot'+jacobian(pos,qdot)*qddot';
    
    pos = rt(i);
    v_rt(i) = jacobian(pos,q)*qdot';
    %a_rt(i) = jacobian(pos,q)*qdot'+jacobian(pos,qdot)*qddot';
    
    pos = rc(i);
    v_rc(i) = jacobian(pos,q)*qdot';
    %a_rc(i) = jacobian(pos,q)*qdot'+jacobian(pos,qdot)*qddot';
    
    pos = lt(i);
    v_lt(i) = jacobian(pos,q)*qdot';
    %a_lt(i) = jacobian(pos,q)*qdot'+jacobian(pos,qdot)*qddot';
    
    pos = lc(i);
    v_lc(i) = jacobian(pos,q)*qdot';
    %a_lc(i) = jacobian(pos,q)*qdot'+jacobian(pos,qdot)*qddot';
    
    pos = RA(i);
    v_RA(i) = jacobian(pos,q)*qdot';
    %a_RA(i) = jacobian(pos,q)*qdot'+jacobian(pos,qdot)*qddot';
    
    
    pos = LA(i);
    v_LA(i) = jacobian(pos,q)*qdot';
    %a_LA(i) = jacobian(pos,q)*qdot'+jacobian(pos,qdot)*qddot';
    
    
end

vel_hip_l_stance = subs(-v_LA,[xd yd zd],[0 0 0]);
vel_hip_r_stance = subs(-v_RA,[xd yd zd],[0 0 0]);
f_vel_hip = matlabFunction(vel_hip_l_stance, vel_hip_r_stance,...
   'File','hip_velocities','Outputs',{'vel_hip_l_stance', 'vel_hip_r_stance'});

% f_vel = matlabFunction(v_LA, v_RA,...
%    'File','foot_velocities','Outputs',{'v_LA', 'v_RA'});
% 
% f_acc = matlabFunction(a_LA, a_RA,...
%    'File','foot_acclerations','Outputs',{'a_LA', 'a_RA'});
% 
% f_pos = matlabFunction(b,rt,rc,lt,lc,...
%    'File','position','Outputs',{'b','rt','rc','lt','lc'});
% f_linvel = matlabFunction(v_b',v_rc',v_rt',v_lc',v_lt',...
%    'File','linVel','Outputs',{'v_b','v_rc','v_rt','v_lc','v_lt'});
% f_angvel = matlabFunction(omega_14,omega_17l,omega_18l,omega_17r,omega_18r,omegaB_4,omegaB_7l,omegaB_8l,omegaB_7r,omegaB_8r,...
%    'File','angVel','Outputs',{'omega_14','omega_17l','omega_18l','omega_17r','omega_18r','omegaB_4','omegaB_7l','omegaB_8l','omegaB_7r','omegaB_8r'});
% f_dc = matlabFunction(R14,R17l,R18l,R17r,R18r,...
%    'File','dircos','Outputs',{'R14','R17l','R18l','R17r','R18r'});
             
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Potential, Kinetic, and Total Energy
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ib = diag([Ibx Iby Ibz]);
It = diag([Itx Ity Itz]);
Ic = diag([Icx Icy Icz]);
T = 0.5*((mb*dot(v_b,v_b)+mt*dot(v_rt,v_rt)+mc*dot(v_rc,v_rc)+mt*dot(v_lt,v_lt)+mc*dot(v_lc,v_lc)+...
                  dot(omegaB_4,Ib*omegaB_4) + dot(omegaB_7l,It*omegaB_7l) + dot(omegaB_7r,It*omegaB_7r) + ...
                   + dot(omegaB_8l,Ic*omegaB_8l) + dot(omegaB_8r,Ic*omegaB_8r) ...
                 ));
V = mb*simplify(g*b(3))+mt*simplify(g*rt(3))+mt*simplify(g*lt(3))+mc*simplify(g*lc(3))+mc*simplify(g*rc(3)); %potential is positive because com is above reference point
L = T-V;
 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Derive equations of motion
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for ii=1:dof
    dLdqdot(ii) = diff(L,qdot(ii));
    temp = 0; 
    for j = 1:dof
        temp = temp+diff(dLdqdot(ii),q(j))*qdot(j) + diff(dLdqdot(ii),qdot(j))*qddot(j);
    end
    ddt_dLdqdot(ii) = temp;
    dLdq(ii) = diff(L,q(ii));
    EOM(ii) = ddt_dLdqdot(ii) - dLdq(ii);
end
% %%%%%%% ss stuff starts now %%%%%%%
% % here A_ss is floating base 
A_ss = jacobian(EOM,qddot);
for ii=1:dof
    b_ss(ii,1) = -subs(EOM(ii),qddot,zeros(1,dof));
end


%%%% jacobians and its rate %%%%%%%
p_right_ankle = RA(1:3)'; %row matrix
p_left_ankle = LA(1:3)'; %row matrix
J_l = jacobian(p_left_ankle,q);
J_r = jacobian(p_right_ankle,q);
for i=1:3
    for j=1:dof
        Jdot_l(i,j) = jacobian(J_l(i,j),q)*qdot';
        Jdot_r(i,j) = jacobian(J_r(i,j),q)*qdot';
    end
end



% f_footjacob = matlabFunction(J_l, J_r,...
%    'File','foot_jacobians','Outputs',{'J_l', 'J_r'});
% f_footjacobdotL = matlabFunction(Jdot_l,...
%    'File','foot_Jdot_l','Outputs',{'Jdot_l'});
% f_footjacobdotR = matlabFunction(Jdot_r,...
%    'File','foot_Jdot_r','Outputs',{'Jdot_r'});















% TMR START HERE!!!!!!!
% TMR START HERE!!!!!!!
% TMR START HERE!!!!!!!
% TMR START HERE!!!!!!!
% TMR START HERE!!!!!!!

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Generating rhs file');
fid=fopen( 'single_stanceMEX.m','w');
fprintf(fid, 'function [zdot,A,b,P_LA,P_RA,tau] = single_stanceMEX(t,zz,parms)\n\n');
fprintf(fid,'B = [0 0 0 0 0 0 0 0; \n');
fprintf(fid,'     0 0 0 0 0 0 0 0; \n');
fprintf(fid,'     0 0 0 0 0 0 0 0; \n');
fprintf(fid,'     0 0 0 0 0 0 0 0; \n');
fprintf(fid,'     0 0 0 0 0 0 0 0; \n');
fprintf(fid,'     0 0 0 0 0 0 0 0; \n');
fprintf(fid,'     1 0 0 0 0 0 0 0; \n');
fprintf(fid,'     0 1 0 0 0 0 0 0; \n');
fprintf(fid,'     0 0 1 0 0 0 0 0; \n');
fprintf(fid,'     0 0 0 1 0 0 0 0; \n');
fprintf(fid,'     0 0 0 0 1 0 0 0; \n');
fprintf(fid,'     0 0 0 0 0 1 0 0; \n');
fprintf(fid,'     0 0 0 0 0 0 1 0; \n');
fprintf(fid,'     0 0 0 0 0 0 0 1; \n');
fprintf(fid,'     0 0 0 0 0 0 0 0; \n');
fprintf(fid,'     0 0 0 0 0 0 0 0; \n');
fprintf(fid,'     0 0 0 0 0 0 0 0]; \n\n');
 
fprintf(fid,'tau = controller(t,zz,parms);\n\n');
fprintf(fid,'[x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid, ... ; \n');
fprintf(fid,'phi_lh, phi_lhd, theta_lh, theta_lhd, psi_lh, psi_lhd, theta_lk, theta_lkd, ... ; \n');
fprintf(fid,'phi_rh, phi_rhd, theta_rh, theta_rhd, psi_rh, psi_rhd, theta_rk, theta_rkd]= getstate(zz); \n\n');
 
fprintf(fid, '[mb,mt,mc,Ibx,Iby,Ibz,Itx,Ity,Itz,Icx,Icy,Icz,l0,l1,l2,w,g] = getparms(parms); \n\n');
fprintf(fid, 'params = [mb mt mc Ibx Iby Ibz Itx Ity Itz Icx Icy Icz l0 l1 l2 w g]; \n\n');
fprintf(fid,'[~,nn] = size(zz);\n');
fprintf(fid,'if (nn==1) \n');
fprintf(fid,'    zz = zz'';\n');
fprintf(fid,'end\n\n');
fprintf(fid,'[A,b,J_l,J_r,Jdot_l,Jdot_r] = gateway_dynamicsMEX(t,zz,params); \n\n');
fprintf(fid,'A = A''; \n');
fprintf(fid,'J_l = J_l''; J_r = J_r''; \n');
fprintf(fid,'Jdot_l = Jdot_l''; Jdot_r = Jdot_r''; \n');
fprintf(fid,'qdot = [xd yd zd phid thetad psid phi_lhd theta_lhd psi_lhd theta_lkd phi_rhd theta_rhd psi_rhd theta_rkd]; \n\n');

fprintf(fid,'if (strcmp(parms.stance_foot,''r''))\n');
fprintf(fid,'    AA = [A, -J_r''; J_r, zeros(3,3)];\n');
%fprintf(fid,'    bb1 = [b; -Jdot_r*qdot''];\n');
%fprintf(fid, '   bb2 = [zeros(6,1); tau; zeros(3,1)]; \n');
%fprintf(fid, '   bb = bb1 + bb2 \n');
fprintf(fid, '    bb = [b; -Jdot_r*qdot''] + B*tau; \n');
fprintf(fid, '   alpha=AA\\bb; \n\n');
fprintf(fid, '   P_RA = alpha(15:17)''; \n');
fprintf(fid, '   P_LA = zeros(1,3); \n');
fprintf(fid,'elseif (strcmp(parms.stance_foot,''l''))\n');
fprintf(fid,'    AA = [A, -J_l''; J_l, zeros(3,3)];\n');
% fprintf(fid,'    bb1 = [b; -Jdot_l*qdot''];\n');
% fprintf(fid, '   bb2 = [zeros(6,1); tau; zeros(3,1)]; \n');
% fprintf(fid, '   bb = bb1 + bb2 \n');
fprintf(fid, '    bb = [b; -Jdot_l*qdot''] + B*tau; \n');
fprintf(fid, '   alpha=AA\\bb; \n\n');
fprintf(fid, '   P_LA = alpha(15:17)''; \n');
fprintf(fid, '   P_RA = zeros(1,3); \n');
fprintf(fid,'else\n');
fprintf(fid,'   error(''parms.stance_foot needs to be set to l or r'');\n');
fprintf(fid,'end\n\n');

fprintf(fid, 'xdd = alpha(1); ydd = alpha(2); zdd = alpha(3); phidd = alpha(4); thetadd = alpha(5); psidd = alpha(6); \n\n');
fprintf(fid, 'phi_lhdd = alpha(7); theta_lhdd = alpha(8); psi_lhdd = alpha(9); theta_lkdd = alpha(10);  \n\n');
fprintf(fid, 'phi_rhdd = alpha(11); theta_rhdd = alpha(12); psi_rhdd = alpha(13); theta_rkdd = alpha(14);  \n\n');
fprintf(fid, 'zdot=[xd xdd yd ydd zd zdd phid phidd thetad thetadd psid psidd phi_lhd phi_lhdd theta_lhd theta_lhdd psi_lhd ...  \n');
fprintf(fid, 'psi_lhdd theta_lkd theta_lkdd phi_rhd phi_rhdd theta_rhd theta_rhdd psi_rhd psi_rhdd theta_rkd theta_rkdd]'';  \n\n');
fclose(fid);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Generating file to generate A and b matrices');
fid=fopen( 'get_eom.c','w');
fprintf(fid,'void get_dynamics(double *ptr_A, double *b, double *ptr_Jl, double *ptr_Jr, double *ptr_Jdotl, double *ptr_Jdotr,  double *Z, double *params) \n\n');
fprintf(fid,'{\n\n');
fprintf(fid,'int i,j; \n\n');
fprintf(fid,['double A_ss[',num2str(dof),'][',num2str(dof),']={0};\n']);
fprintf(fid,['double b_ss[',num2str(dof),'][1]={0};\n\n']);
fprintf(fid,['double J_l[3][',num2str(dof),']={0};\n']);
fprintf(fid,['double J_r[3][',num2str(dof),']={0};\n']);
fprintf(fid,['double Jdot_l[3][',num2str(dof),']={0};\n']);
fprintf(fid,['double Jdot_r[3][',num2str(dof),']={0};\n\n']);

fprintf(fid,'    double mb, mt, mc, Ibx, Iby, Ibz, Itx, Ity, Itz, Icx, Icy, Icz, l0, l1, l2, w, g;\n'); 
fprintf(fid,'    double x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid;\n'); 
fprintf(fid,'    double phi_lh, phi_lhd, theta_lh, theta_lhd, psi_lh, psi_lhd, theta_lk, theta_lkd;\n'); 
fprintf(fid,'    double phi_rh, phi_rhd, theta_rh, theta_rhd, psi_rh, psi_rhd, theta_rk, theta_rkd;\n\n'); 
    
fprintf(fid,'          i = 0;\n'); 
fprintf(fid,'      mb = params[i]; i=i+1;\n'); 
fprintf(fid,'      mt = params[i]; i=i+1;\n'); 
fprintf(fid,'     mc = params[i]; i=i+1;\n'); 
fprintf(fid,'     Ibx = params[i]; i=i+1;\n'); 
fprintf(fid,'      Iby = params[i]; i=i+1;\n'); 
fprintf(fid,'      Ibz = params[i]; i=i+1;\n'); 
fprintf(fid,'      Itx = params[i]; i=i+1;\n'); 
fprintf(fid,'      Ity = params[i]; i=i+1;\n'); 
fprintf(fid,'      Itz = params[i]; i=i+1;\n'); 
fprintf(fid,'      Icx = params[i]; i=i+1;\n'); 
fprintf(fid,'      Icy = params[i]; i=i+1;\n'); 
fprintf(fid,'      Icz = params[i]; i=i+1;\n'); 
fprintf(fid,'      l0 = params[i]; i=i+1;\n'); 
fprintf(fid,'      l1 = params[i]; i=i+1;\n'); 
fprintf(fid,'      l2 = params[i]; i=i+1;\n'); 
fprintf(fid,'      w = params[i]; i=i+1;\n'); 
fprintf(fid,'      g = params[i];\n\n'); 
      
      
fprintf(fid,'    i = 0;\n'); 
fprintf(fid,'    x = Z[i]; i=i+1;\n'); 
fprintf(fid,'    xd = Z[i]; i=i+1;\n'); 
fprintf(fid,'    y = Z[i]; i=i+1;\n'); 
fprintf(fid,'    yd = Z[i]; i=i+1; \n'); 
fprintf(fid,'    z  = Z[i]; i=i+1;\n'); 
fprintf(fid,'    zd = Z[i]; i=i+1; \n'); 
fprintf(fid,'    phi = Z[i]; i=i+1;\n');  
fprintf(fid,'    phid = Z[i]; i=i+1;\n'); 
fprintf(fid,'    theta  = Z[i]; i=i+1;\n'); 
fprintf(fid,'    thetad  = Z[i]; i=i+1;\n'); 
fprintf(fid,'    psi  = Z[i]; i=i+1;\n'); 
fprintf(fid,'    psid  = Z[i]; i=i+1;\n'); 
fprintf(fid,'    phi_lh  = Z[i]; i=i+1;\n'); 
fprintf(fid,'    phi_lhd  = Z[i]; i=i+1;\n'); 
fprintf(fid,'    theta_lh = Z[i]; i=i+1;\n'); 
fprintf(fid,'    theta_lhd = Z[i]; i=i+1;\n');  
fprintf(fid,'    psi_lh = Z[i]; i=i+1; \n'); 
fprintf(fid,'    psi_lhd = Z[i]; i=i+1; \n'); 
fprintf(fid,'    theta_lk = Z[i]; i=i+1;\n'); 
fprintf(fid,'    theta_lkd = Z[i]; i=i+1;\n');  
fprintf(fid,'    phi_rh = Z[i]; i=i+1; \n'); 
fprintf(fid,'    phi_rhd = Z[i]; i=i+1; \n'); 
fprintf(fid,'    theta_rh = Z[i]; i=i+1;\n');  
fprintf(fid,'    theta_rhd = Z[i]; i=i+1;\n'); 
fprintf(fid,'    psi_rh = Z[i]; i=i+1; \n'); 
fprintf(fid,'    psi_rhd = Z[i]; i=i+1; \n'); 
fprintf(fid,'    theta_rk = Z[i]; i=i+1;\n'); 
fprintf(fid,'    theta_rkd = Z[i]; //i=i+1;\n\n'); 
fprintf(fid,ccode(A_ss));
fprintf(fid,'\n\n');
fprintf(fid,ccode(b_ss));
fprintf(fid,'\n\n');
fprintf(fid,ccode(J_l));
fprintf(fid,'\n\n');
fprintf(fid,ccode(J_r));
fprintf(fid,'\n\n');
fprintf(fid,ccode(Jdot_l));
fprintf(fid,'\n\n');
fprintf(fid,ccode(Jdot_r));
fprintf(fid,'\n\n');
fprintf(fid,['for (i=0;i<',num2str(dof),';i++)\n']);
fprintf(fid,['for (j=0;j<',num2str(dof),';j++)\n']);
fprintf(fid,['*(ptr_A + i*',num2str(dof),'+j) = A_ss[i][j];\n\n']);
fprintf(fid,'for (i=0;i<3;i++)\n');
fprintf(fid,'{\n');
fprintf(fid,['for (j=0;j<',num2str(dof),';j++)\n']);
fprintf(fid,'{\n');
fprintf(fid,['*(ptr_Jl + i*',num2str(dof),'+j) = J_l[i][j];\n\n']);
fprintf(fid,['*(ptr_Jr + i*',num2str(dof),'+j) = J_r[i][j];\n\n']);
fprintf(fid,['*(ptr_Jdotl + i*',num2str(dof),'+j) = Jdot_l[i][j];\n\n']);
fprintf(fid,['*(ptr_Jdotr + i*',num2str(dof),'+j) = Jdot_r[i][j];\n\n']);
fprintf(fid,'}\n');
fprintf(fid,'}\n\n');
fprintf(fid,['for (i=0;i<',num2str(dof),';i++)\n\n']);
fprintf(fid,'b[i] = b_ss[i][0];\n\n');

fprintf(fid,'}\n\n');
fclose(fid);
disp('mexing gateway_dynamicsMEX.c');
mex gateway_dynamicsMEX.c
