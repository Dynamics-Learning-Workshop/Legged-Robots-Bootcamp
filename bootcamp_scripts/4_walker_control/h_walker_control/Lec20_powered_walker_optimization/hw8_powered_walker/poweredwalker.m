function poweredwalker  

clc
close all
format long
 
%%%% Dimensions %%
%%% c = COM on the leg from hip M = hip mass, m = leg mass, I = leg inertia, l = leg length
walker.M = 1.0; walker.m = 0.5; walker.I = 0.02; walker.l = 1.0; 
walker.c = 0.5; walker.g = 1.0; walker.gam = 0*0.01; 

walker.control.P = 0.1;

%%%% Initial State %%%%%
q1 = 0.2; u1 = -0.3;
q2 = -0.4; u2 = 0.3;

z0 = [q1 u1 q2 u2];
% % %%% Root finding will give this stable root 
%zstar = [0.183500818246392  -0.273335992124155  -0.367001636492785   0.031383021226154]
% onestep(zstar,walker)
% %%%%%%%%%%%%%%%%%%%%%%%%%
steps = 5; %number of steps to animate
fps = 10; %Use low frames per second for low gravity

%%%% Root finding, Period one gait %%%%
options = optimset('TolFun',1e-12,'TolX',1e-12,'Display','off');
[zstar,fval,exitflag] = fsolve(@fixedpt,z0,options,walker);
if exitflag == 1
    disp(['Root finding success (1 is successful) = ',num2str(exitflag)]);
    disp('Fixed point:');
    disp(zstar);
else
    error('Root finder not converged, change guess or change system parameters')
end

%%% Stability, using eigenvalues of Poincare map %%%
J=partialder(@onestep,zstar,walker);
disp('EigenValues for linearized map are');
eigJ = eig(J);
for i=1:4
    disp(norm(eigJ(i)));
end

% %%% forward simulation  %%%
%zstar = z0;
[z,t] = onestep(zstar,walker,steps);

%%% Animate result %%%
disp('Animating...');
figure(1)
animate(t,z,walker,steps,fps);

% %%% Plot data %%%
disp('Some plots...')
figure(2)
subplot(2,1,1);
title('passive walker position and velocity as a function of time');
plot(t,z(:,1),'r--','Linewidth',3); hold on
plot(t,z(:,3),'b','Linewidth',2);
ylabel('position','Fontsize',12);
legend('\theta_1','\theta_2','Location','best','Fontsize',12);
subplot(2,1,2)
plot(t,z(:,2),'r--','Linewidth',3); hold on
plot(t,z(:,4),'b','Linewidth',2);
ylabel('velocity','Fontsize',12);
xlabel('time','Fontsize',12);
legend('\theta_1','\theta_2','Location','best','Fontsize',12);


% %%% forward simulation  %%%
% z_pert = zstar + [0 0.01 0 0.05];
% [z,t] = onestep(z_pert,walker,steps);


% figure(3)
% subplot(2,1,1);
% title('passive walker absolute leg angle rate versus absolute leg angle');
% plot(z(:,1),z(:,2),'r','Linewidth',3); hold on;
% plot(z(1,1),z(1,2),'ko','Markersize',10,'MarkerFaceColor','k');
% text(z(1,1)+0.01,z(1,2)+0.01,'start','Fontsize',12)
% ylabel('$\dot{\theta}_1$','Fontsize',12,'Interpreter','Latex');
% xlabel('\theta_1','Fontsize',12);
% subplot(2,1,2);
% plot(z(:,1)+z(:,3),z(:,2)+z(:,4),'b','Linewidth',2); hold on;
% plot(z(1,1)+z(1,3),z(1,2)+z(1,4),'ko','Markersize',10,'MarkerFaceColor','k');
% text(z(1,1)+z(1,3)+0.01,z(1,2)+z(1,4)+0.01,'start','Fontsize',12)
% ylabel('$\dot{\theta}_1+\dot{\theta}_2$','Fontsize',12,'Interpreter','Latex');
% xlabel('\theta_1+\theta_2','Fontsize',12);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% FUNCTIONS START HERE %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%===================================================================
function zdiff=fixedpt(z0,walker)
%===================================================================
zdiff=onestep(z0,walker)-z0; 

%===================================================================
function J=partialder(FUN,z,walker)
%===================================================================
pert=1e-5;
n = length(z);
J = zeros(n,n);

%%%% Using forward difference, accuracy linear %%%
% y0=feval(FUN,z,walker); 
% for i=1:n
%     ztemp=z;
%     ztemp(i)=ztemp(i)+pert; 
%     J(:,i)=(feval(FUN,ztemp,walker)-y0) ;
% end
% J=(J/pert);

%%% Using central difference, accuracy quadratic %%%
for i=1:n
    ztemp1=z; ztemp2=z;
    ztemp1(i)=ztemp1(i)+pert; 
    ztemp2(i)=ztemp2(i)-pert; 
    J(:,i)=(feval(FUN,ztemp1,walker)-feval(FUN,ztemp2,walker)) ;
end
J=J/(2*pert);

%===================================================================
function [z,t]=onestep(z0,walker,steps)
%===================================================================

l = walker.l;

flag = 1;
if nargin<2
    error('need more inputs to onestep');
elseif nargin<3
    flag = 0; %send only last state, for root finder and jacobian
    steps = 1;
end

theta1 = z0(1);
xh = 0;
yh = l*cos(theta1);
xh_start = xh;

t0 = 0; 
dt = 4; %might need to be changed based on estimate of time taken for one step
time_stamps = 100;
t_ode = t0;
z_ode = [z0 xh yh];

for i=1:steps
    options=odeset('abstol',1e-13,'reltol',1e-13,'events',@collision);
    tspan = linspace(t0,t0+dt,time_stamps);
    [t_temp, z_temp] = ode113(@single_stance,tspan,z0,options,walker);

    zplus=footstrike(t_temp(end),z_temp(end,:),walker);    
    
    z0 = zplus;
    t0 = t_temp(end);
    
    xh_temp = xh_start + l*sin(z_temp(1,1))-l*sin(z_temp(:,1)); 
    yh_temp =  l*cos(z_temp(:,1));
    
    t_ode = [t_ode; t_temp(2:end)];
    z_ode = [z_ode; [z_temp(2:(end-1),:); zplus] xh_temp(2:end) yh_temp(2:end)];
    xh_start = xh_temp(end);
end

z = zplus(1:4);

if flag==1
   z=z_ode;
   t=t_ode;
end

%===================================================================
function [zdot,Th]=single_stance(t,z,walker)  
%===================================================================
theta1 = z(1);   omega1 = z(2);                         
theta2 = z(3);   omega2 = z(4);                         
                    
M = walker.M;  m = walker.m; I = walker.I;   
l = walker.l;  c = walker.c; 
g = walker.g; gam = walker.gam;

Th = 0;

%derived from poweredderive_newton
% A11 = 2*c*l*m - M*l^2 - 2*c^2*m - 2*l^2*m - 2*I + 2*c*l*m*cos(theta2);
% A12 = c*l*m*cos(theta2) - c^2*m - I;
% A21 = c*l*m*cos(theta2) - c^2*m - I;
% A22 = - I - c^2*m;
% b1 = c*g*m*sin(theta1 - gam + theta2) + M*g*l*sin(gam - theta1) - c*g*m*sin(gam - theta1) + 2*g*l*m*sin(gam - theta1) + c*l*m*omega2^2*sin(theta2) + 2*c*l*m*omega1*omega2*sin(theta2);
% b2 = c*g*m*sin(theta1 - gam + theta2) - Th - c*l*m*omega1^2*sin(theta2);
% A_ss = [A11 A12; A21 A22];
% b_ss = [b1; b2];
% alpha = A_ss\b_ss;

%derived from poweredderive_lagrange
A11 = 2*I + M*l^2 + 2*c^2*m + 2*l^2*m - 2*c*l*m - 2*c*l*m*cos(theta2);
A12 = I + c^2*m - c*l*m*cos(theta2);
A21 = I + c^2*m - c*l*m*cos(theta2);
A22 = I + c^2*m;
A_ss = [A11 A12; A21 A22];
 
b1 = c*g*m*sin(gam - theta1) - M*g*l*sin(gam - theta1) - c*g*m*sin(theta1 - gam + theta2) - 2*g*l*m*sin(gam - theta1) - c*l*m*omega2^2*sin(theta2) - 2*c*l*m*omega1*omega2*sin(theta2);
b2 = Th - c*g*m*sin(theta1 - gam + theta2) + c*l*m*omega1^2*sin(theta2);
b_ss = [b1; b2];
 
alpha = A_ss\b_ss;

zdot = [omega1 alpha(1) omega2 alpha(2)]';  

%===================================================================
function [gstop, isterminal,direction]=collision(t,z,walker)
%===================================================================

theta1 = z(1); theta2 = z(3); 

gstop = theta2 + 2*theta1;
if (theta1>-0.05) %allow legs to pass through for small hip angles (taken care in real walker using stepping stones)
    isterminal = 0;
else
    isterminal=1; %ode should terminate is conveyed by 1, if you put 0 it goes till the final time u specify
end
direction=[]; % The t_final can be approached by any direction is indicated by the direction

%===================================================================
function zplus=footstrike(t,z,walker)      
%===================================================================

theta1_n = z(1);   omega1_n = z(2);                         
theta2_n = z(3);   omega2_n = z(4);                         
                      
theta1 = theta1_n + theta2_n;                         
theta2 = -theta2_n;                                       

M = walker.M;  m = walker.m; I = walker.I;   
l = walker.l;  c = walker.c;  

P = walker.control.P;

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


%===================================================================
function animate(t_all,z_all,walker,steps,fps)
%===================================================================

%%%% Interpolate linearly using fps %%%%%
z_all_plot = [z_all(:,1) z_all(:,3) z_all(:,5) z_all(:,6)];
nn = size(z_all_plot,2);
total_frames = round(t_all(end)*fps);
t = linspace(0,t_all(end),total_frames);
z = zeros(total_frames,nn);
for i=1:nn
    z(:,i) = interp1(t_all,z_all_plot(:,i),t);
end

%%%%% Now animate the results %%%%%%%  
l = walker.l;    
c = walker.c;

mm = size(z,1);

min_xh = min(z(:,3)); max_xh = max(z(:,3)); 
dist_travelled = max_xh - min_xh;
camera_rate = dist_travelled/mm;

window_xmin = -1*l; window_xmax = 1*l;
window_ymin = -0.1; window_ymax = 1.1*l;

axis('equal')
axis([window_xmin window_xmax window_ymin window_ymax])
axis off
set(gcf,'Color',[1,1,1])

%%%% create ramp %%%%
rampref=[min_xh-1 max_xh+l ; 0 0];

%%%% Draw ramp %%%%%%%%%%
line('xdata',rampref(1,:),'ydata',rampref(2,:),'linewidth', 1,'color','black'); hold on;
 
for i=1:mm
   %moving window %%%%%%%
   window_xmin = window_xmin + camera_rate;
   window_xmax = window_xmax + camera_rate;
   axis('equal')
   axis([window_xmin window_xmax window_ymin window_ymax])

   %%% get angles and hip position 
   theta1 = z(i,1); theta2 = z(i,2); 
   xh = z(i,3); yh = z(i,4);
   
   %%% coordinates of points of interest %
   hinge=[xh; yh];  
   stance_foot = [xh+l*sin(theta1);         yh-l*cos(theta1)];
   stance_leg_com = [xh+c*sin(theta1);         yh-c*cos(theta1)];
   swing_foot =  [xh+l*sin(theta1 + theta2),yh-l*cos(theta1 + theta2)];
   swing_leg_com =  [xh+c*sin(theta1 + theta2),yh-c*cos(theta1 + theta2)];
   
   %animate 
   h0 = plot(hinge(1),hinge(2),'ko','MarkerFaceColor','k','Markersize',15);
   h1 = plot(stance_leg_com(1),stance_leg_com(2),'ko','MarkerFaceColor','k','Markersize',10);
   h2 = plot(swing_leg_com(1),swing_leg_com(2),'ko','MarkerFaceColor','k','Markersize',10);
   h3 = line([hinge(1) stance_foot(1)],[hinge(2) stance_foot(2)],'Color','red','Linewidth',2);
   h4 = line([hinge(1) swing_foot(1)], [hinge(2) swing_foot(2)],'Color','red','Linewidth',2);
   
   %delay to create animation
   pause(0.01);
   
   if (i~=mm)     %delete all but not the last entry
       delete(h0);
       delete(h1);
       delete(h2);
       delete(h3);
       delete(h4);
   end
   
end

