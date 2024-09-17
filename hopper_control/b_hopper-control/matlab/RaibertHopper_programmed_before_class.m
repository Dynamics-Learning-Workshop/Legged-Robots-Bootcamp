% RAIBERTRUNNER simulates Spring Loaded Inverted Pendulum (SLIP) runner
% 
% Needs ODE113, FSOLVE, INTERP1. 
% If you find bugs in this code please mail, 
% Pranav A. Bhounsule, pranav.bhounsule@utsa.edu
% Last updated: 18 May 2016

% Fixed gstop in the function release, 23 June 2017

function RaibertHopper  

clc
clear all
close all
format long

% %%% initial parameters (set 1)
% robot.g = 10;
% robot.ground = 0; %ground is at y co-ordinate equal to robot.ground
% robot.l = 1;
% robot.m = 100;
% robot.c = 2; %damping
% robot.control.k = 10000;
% robot.control.theta  = 5*(pi/180); %pi/6; %angle between leg and vertical
% x0dot = 0.50435	;% 0.5;  
% y0 = 1.1873	;
% Root finding will give this root
%zstar = [0.513806075894484   1.238312718076982];

%%%% initial parameters (set 2)
robot.g = 10;
robot.ground = 0; %ground is at y co-ordinate equal to robot.ground
robot.l = 1;
robot.m = 1;
robot.control.k = 100;

%steps = 5; %number of steps to animate
fps = 10; %Use low frames per second for low gravity
robot.control.vdes = [0 0.1 0.1 0.5 0.5 0.5 0.75 0.75 1 1.25 2 ];
steps = length(robot.control.vdes);
robot.control.Kp = 0.05;
x0dot =  robot.control.vdes(1); 
y0 = 1.2;

robot.control.tp = pi*sqrt(robot.m/robot.control.k);
robot.control.theta = asin(x0dot*robot.control.tp/(2*robot.l));
robot.control.theta

% Root finding will give this unstable root 
%zstar = [1.017450873821607   1.182441943200824];

% initial guess
z0 = [x0dot y0];


% %%% Root finding, Period one gait %%%%
% options = optimset('TolFun',1e-10,'TolX',1e-10,'Display','iter');
% [zstar,fval,exitflag] = fsolve(@fixedpt,z0,options,robot);
% if exitflag == 1
%     disp('Fixed point:');
%     disp(zstar);
% else
%     error('Root finder not converged, change guess or change system parameters')
% end
 
% %%% Stability, using eigenvalues of Poincare map %%%
% J=partialder(@onestep,zstar,robot);
% disp('EigenValues for linearized map are');
% eig(J)
 
%%%% Get data for all the steps %%%
[z,t,v_act] = onestep(z0,robot,steps,fps);

% %%% Animate result %%%
disp('Animating...');
animate(t,z,robot,steps,fps);

%%% Plot data %%%
disp('Some plots...')
figure(2)
subplot(2,1,1)
plot(t,z(:,1),'r',t,z(:,3),'b')
xlabel('time'); ylabel('Angle (m)');
legend('x','y');
subplot(2,1,2)
plot(t,z(:,2),'r',t,z(:,4),'b')
xlabel('time'); ylabel('Velocity (m/s)');
legend('vx','vy');

figure(3)
stairs(robot.control.vdes,'k-','Linewidth',2); hold on
stairs(v_act,'r','LineWidth',2);
legend('desired','actual');
ylabel('apex horizontal velocity');
xlabel('step number');

% figure(3)
% plot(z(:,1),z(:,3),'r'); %hold on
% xlabel('x'); ylabel('y');
% title('Trajectory y vs x');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% FUNCTIONS START HERE %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %===================================================================
% function zdiff=fixedpt(z0,robot)
% %===================================================================
% zdiff=onestep(z0,robot)-z0; 
% 
% %===================================================================
% function J=partialder(FUN,z,robot)
% %===================================================================
% pert=1e-5;
% n = length(z);
% J = zeros(n,n);
% 
% %%%% Using forward difference, accuracy linear %%%
% % y0=feval(FUN,z,walker); 
% % for i=1:n
% %     ztemp=z;
% %     ztemp(i)=ztemp(i)+pert; 
% %     J(:,i)=(feval(FUN,ztemp,walker)-y0) ;
% % end
% % J=(J/pert);
% 
% %%% Using central difference, accuracy quadratic %%%
% for i=1:n
%     ztemp1=z; ztemp2=z;
%     ztemp1(i)=ztemp1(i)+pert; 
%     ztemp2(i)=ztemp2(i)-pert; 
%     J(:,i)=(feval(FUN,ztemp1,robot)-feval(FUN,ztemp2,robot)) ;
% end
% J=J/(2*pert);
% 
%===================================================================
function [z,t,v_act]=onestep(z0,robot,steps,fps)  %DONE
%===================================================================

flag = 1;
if nargin<2
    error('need more inputs to onestep');
elseif nargin<3
    flag = 0; %send only last state, for root finder and jacobian
    steps = 1;
    fps = 50;
end

x0 = 0; x0dot = z0(1);  
y0 = z0(2); y0dot = 0;


z0 = [x0 x0dot y0 y0dot];

t0 = 0; 
dt = 1; %might need to be changed based on time taken for one step
t_ode = t0;
z_ode = [z0 ...
         x0+robot.l*sin(robot.control.theta) ...
         y0-robot.l*cos(robot.control.theta)];

v_act = [];
for i=1:steps
    
    vx_apex = z0(2);
    v_act = [v_act vx_apex];
    theta_c = vx_apex*robot.control.tp/(2*robot.l);
    speed_correction = robot.control.Kp*(vx_apex-robot.control.vdes(i));
    robot.control.theta = asin(theta_c)+speed_correction;
    
    robot.control.theta
    %%% apex to ground %%%
    options1 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@contact);
    tspan = linspace(t0,t0+dt,dt*1000);
    [t_temp1,z_temp1]=ode113(@flight,tspan,z0,options1,robot);
    [t_temp1,z_temp1] = loco_interpolate(t_temp1,z_temp1,10*fps);

   
    z_temp1 = [z_temp1 ...
               z_temp1(1:end,1)+robot.l*sin(robot.control.theta) ...
               z_temp1(1:end,3)-robot.l*cos(robot.control.theta) ...
              ];
    t0 = t_temp1(end);
    z0(1:4) = z_temp1(end,1:4);
    x_com = z0(1); %save the x position for future
    z0(1) = -robot.l*sin(robot.control.theta); %relative distance wrt contact point because of non-holonomic nature of the system
    x_foot = x_com + robot.l*sin(robot.control.theta); 
    y_foot = robot.ground;
   
    %%% stance phase %%%
    tspan = linspace(t0,t0+1,dt*2000);
    options2 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@release);
    [t_temp2,z_temp2]=ode113(@stance,tspan,z0,options2,robot);
    [t_temp2,z_temp2] = loco_interpolate(t_temp2,z_temp2,10*fps);

    
    z_temp2(:,1) = z_temp2(:,1) + x_com + robot.l*sin(robot.control.theta); %absolute x co-ordinate
    z_temp2 = [z_temp2, ...
          x_foot*ones(length(z_temp2),1) y_foot*ones(length(z_temp2),1)]; %the distal end of leg is 0 when touching the ground.
    t0 = t_temp2(end);
    z0(1:4) = z_temp2(end,1:4);
    
    %%% ground to apex
    tspan = linspace(t0,t0+dt,dt*1000);
    options3 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@apex);
    [t_temp3,z_temp3]=ode113(@flight,tspan,z0,options3,robot);
    [t_temp3,z_temp3] = loco_interpolate(t_temp3,z_temp3,10*fps);

    
     z_temp3 = [z_temp3 ...
               z_temp3(1:end,1)+robot.l*sin(robot.control.theta) ...
               z_temp3(1:end,3)-robot.l*cos(robot.control.theta) ...
              ];
    t0 = t_temp3(end);
    z0(1:4) = z_temp3(end,1:4);
    
    
    %%%%% Ignore time stamps for heelstrike and first integration point
    t_ode = [t_ode; t_temp1(2:end); t_temp2(2:end);  t_temp3(2:end)];
    z_ode = [z_ode; z_temp1(2:end,:); z_temp2(2:end,:); z_temp3(2:end,:)];
    
end

z = [z0(2) z0(3)];

if flag==1
   z=z_ode;
   t=t_ode;
end

%===================================================================
function zdot=flight(t,z,robot)  
%===================================================================
zdot = [z(2) 0 z(4) -robot.g]';

%===================================================================
function [gstop, isterminal,direction]=contact(t,z,robot)
%===================================================================
gstop = z(3) - robot.l*cos(robot.control.theta); %position is 0;
direction = -1; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration

%===================================================================
function zdot=stance(t,z,robot)  
%===================================================================
x = z(1); y = z(3); %x & y position of com wrt ground
l = sqrt(x^2+y^2);
F_spring = robot.control.k*(robot.l-l);
Fx_spring =  F_spring*(x/l);
Fy_spring = F_spring*(y/l);
Fy_gravity = robot.m*robot.g;
xddot = (1/robot.m)*(Fx_spring);
yddot = (1/robot.m)*(-Fy_gravity+Fy_spring);
zdot = [z(2) xddot z(4) yddot]';

%===================================================================
function [gstop, isterminal,direction]=release(t,z,robot)
%===================================================================
%gstop = z(3) - robot.l*cos(robot.control.theta); 
l = sqrt(z(1)^2+z(3)^2);
gstop = l-robot.l;
direction = 1; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration

%===================================================================
function [gstop, isterminal,direction]=apex(t,z,robot)
%===================================================================
gstop = z(4) - 0; %ydot is 0;
direction = 0; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration

%===================================================================
function animate(t_all,z_all,robot,steps,fps)
%===================================================================

%%% interpolate for animation %%
[t_interp,z_interp] = loco_interpolate(t_all,z_all,fps);

%%%%% prepare for animation %%%%%%%
[mm,nn] = size(z_interp);
min_xh = min(z_interp(:,1)); max_xh = max(z_interp(:,1)); 
dist_travelled = max_xh - min_xh;
camera_rate = dist_travelled/mm;

window_xmin = -3.0*robot.l; window_xmax = 3*robot.l;
window_ymin = -0.1; window_ymax = 1.9*robot.l;

axis('equal')
axis([window_xmin window_xmax window_ymin window_ymax])
axis off
set(gcf,'Color',[1,1,1])

%%%%% now animate %%%%%%%
figure(1);
for i=1:length(t_interp)
   
    plot(z_interp(i,1),z_interp(i,3),'ro','MarkerEdgeColor','r', 'MarkerFaceColor','r','MarkerSize',20); %com
    line([-3 max(z_interp(:,1))+3],[0 0],'Linewidth',2,'Color','black'); %ground
    line([z_interp(i,1) z_interp(i,5)],[z_interp(i,3) z_interp(i,6)],'Linewidth',4,'Color',[0 0.8 0]); %leg
     
    window_xmin = window_xmin + camera_rate;
    window_xmax = window_xmax + camera_rate;
    axis('equal')
    axis off
    axis([window_xmin window_xmax window_ymin window_ymax])

    pause(0.05);
end

%===================================================================
function [t_interp,z_interp] = loco_interpolate(t_all,z_all,fps)
%===================================================================
[m,n] = size(z_all);
t_interp = linspace(t_all(1),t_all(end),fps*(t_all(end)-t_all(1)));

for i=1:n
    z_interp(:,i) = interp1(t_all,z_all(:,i),t_interp);
end
t_interp = t_interp';