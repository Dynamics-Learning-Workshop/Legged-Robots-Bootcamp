function [thetadot_apex,flag,z,t] = walker_sim(thetadot,phi,walker)

%%%%% initial state %%%%
theta = 0;
z0 = [theta thetadot];

%%% forward simulation  %%%
[z,t,thetadot_apex,flag] = onestep(z0,walker,phi);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% FUNCTIONS START HERE %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%===================================================================
function [z,t,thetadot_apex,flag]=onestep(z0,walker,phi)
%===================================================================

l = walker.l;

if nargin<2
    error('need more inputs to onestep');
end

if isempty(phi)
    steps = walker.steps;
else
    steps = length(phi);
end

theta1 = z0(1);
xh = 0;
yh = l*cos(theta1);
xh_start = xh;

t0 = 0; 
dt = 4; %might need to be changed based on estimate of time taken for one step
time_stamps = 100;
t_ode = t0;
z_ode = [z0 xh yh ...
        xh+walker.l*sin(theta1) ...
        yh-walker.l*cos(theta1) ...
        xh+walker.l*sin(pi/6+theta1) ...
        yh-walker.l*cos(pi/6+theta1)];

flag = [0 0];
thetadot_apex = [];

for i=1:steps
    
    if isempty(phi)
        walker.phi = controller(z0(2),walker.thetadot_des(i));
    else
        walker.phi = phi(i);
    end
    %options=odeset('abstol',1e-13,'reltol',1e-13,'events',@collision_and_midstance);
    options=odeset('abstol',1e-13,'reltol',1e-13,'events',@collision);
    tspan = linspace(t0,t0+dt,time_stamps);
    [t_temp1, z_temp1,te,ye,ie] = ode113(@single_stance,tspan,z0,options,walker);
    
    flag1 = check_error(t_temp1,z_temp1,walker,'mid2collision');
       
    xh_temp1 = xh_start + l*sin(z_temp1(1,1))-l*sin(z_temp1(:,1)); 
    yh_temp1 =  l*cos(z_temp1(:,1));
    
    if (rem(i,2)==1)
        xa_foot1 = xh_temp1 + walker.l*sin(z_temp1(:,1));
        ya_foot1 = yh_temp1 - walker.l*cos(z_temp1(:,1));
        xb_foot1 = xh_temp1 + walker.l*sin(walker.phi+z_temp1(:,1));
        yb_foot1 = yh_temp1 - walker.l*cos(walker.phi+z_temp1(:,1));
    else
        xb_foot1 = xh_temp1 + walker.l*sin(z_temp1(:,1));
        yb_foot1 = yh_temp1 - walker.l*cos(z_temp1(:,1));
        xa_foot1 = xh_temp1 + walker.l*sin(walker.phi+z_temp1(:,1));
        ya_foot1 = yh_temp1 - walker.l*cos(walker.phi+z_temp1(:,1));
    end
    
    zplus=footstrike(t_temp1(end),z_temp1(end,:),walker);    
    
    z0 = zplus;
    t0 = t_temp1(end);
    
    options=odeset('abstol',1e-13,'reltol',1e-13,'events',@midstance);
    tspan = linspace(t0,t0+dt,time_stamps);
    [t_temp2, z_temp2,te,ye,ie] = ode113(@single_stance,tspan,z0,options,walker);
    flag2 = check_error(t_temp2,z_temp2,walker,'collision2mid');
     
    flag = [flag; [flag1 flag2]];
    
    xh_start = xh_temp1(end,:);
    xh_temp2 = xh_start + l*sin(z_temp2(1,1))-l*sin(z_temp2(:,1)); 
    yh_temp2 =  l*cos(z_temp2(:,1));
    
    %%%%% animate the swing leg to give the feel that leg moves gently %%
    [mm,nn] = size(t_temp2);
    phi_traj = linspace(2*z_temp1(end,1),walker.phi,mm)';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if (rem(i,2)==1)
        xb_foot2 = xh_temp2 + walker.l*sin(z_temp2(:,1));
        yb_foot2 = yh_temp2 - walker.l*cos(z_temp2(:,1));
        xa_foot2 = xh_temp2 + walker.l*sin(phi_traj+z_temp2(:,1));
        ya_foot2 = yh_temp2 - walker.l*cos(phi_traj+z_temp2(:,1));
    else
        xa_foot2 = xh_temp2 + walker.l*sin(z_temp2(:,1));
        ya_foot2 = yh_temp2 - walker.l*cos(z_temp2(:,1));
        xb_foot2 = xh_temp2 + walker.l*sin(phi_traj+z_temp2(:,1));
        yb_foot2 = yh_temp2 - walker.l*cos(phi_traj+z_temp2(:,1));
    end
    
    thetadot_apex = [thetadot_apex; z_temp2(end,1:2)]; %save midstance speed

   
    t_ode = [t_ode; t_temp1(2:end); t_temp2(2:end)];
    z_temp = [z_temp1(2:end,:); z_temp2(2:end,:)];
    xh_temp = [xh_temp1(2:end); xh_temp2(2:end)];
    yh_temp = [yh_temp1(2:end); yh_temp2(2:end)];
    xa_foot = [xa_foot1(2:end); xa_foot2(2:end)];
    ya_foot = [ya_foot1(2:end); ya_foot2(2:end)];
    xb_foot = [xb_foot1(2:end); xb_foot2(2:end)];
    yb_foot = [yb_foot1(2:end); yb_foot2(2:end)];

    z_ode = [z_ode; z_temp, xh_temp, yh_temp, xa_foot, ya_foot, xb_foot, yb_foot];
    
    t0 = t_temp2(end);
    z0 = z_temp2(end,1:2);
    xh_start = xh_temp2(end,:);
end


z=z_ode;
t=t_ode;


%===================================================================
function zdot=single_stance(t,z,walker)  
%===================================================================
theta1 = z(1);   omega1 = z(2);                                                 
                    
M = walker.M;  I = walker.I;   
l = walker.l;
g = walker.g; gam = walker.gam;

%%%%%%%%% copy pasted from rimless_derive.m %%%%%%
A_ss = I + M*l^2;
 
b_ss = -M*g*l*sin(gam - theta1);
 
alpha1 = A_ss\b_ss;

%%%%%%%%%%% ends %%%%%%%%%%%%%
 
zdot = [omega1 alpha1]';  


%===================================================================
function [gstop, isterminal,direction]=midstance(t,z,walker)
%===================================================================

theta1 = z(1); 

%n = walker.n;


% h = q1 - pi/n ;
% isterminal=1; %Ode should terminate is conveyed by 1, if you put 0 it goes till the final time u specify
% direction= 1; % The t_final can be approached by any direction is indicated by this
% 

gstop = theta1;
isterminal=1; %ode should terminate is conveyed by 1, if you put 0 it goes till the final time u specify
direction=[]; % The t_final can be approached by any direction is indicated by the direction


%===================================================================
function [gstop, isterminal,direction]=collision(t,z,walker)
%===================================================================

theta1 = z(1); 

%n = walker.n;


% h = q1 - pi/n ;
% isterminal=1; %Ode should terminate is conveyed by 1, if you put 0 it goes till the final time u specify
% direction= 1; % The t_final can be approached by any direction is indicated by this
% 

gstop = walker.phi+2*theta1;
isterminal=1; %ode should terminate is conveyed by 1, if you put 0 it goes till the final time u specify
direction=[]; % The t_final can be approached by any direction is indicated by the direction

%===================================================================
function zplus=footstrike(t,z,walker)      
%===================================================================

theta1_n = z(1);   omega1_n = z(2);                         
 
M = walker.M;  I = walker.I;   
l = walker.l;  %n = walker.n; 
phi = walker.phi;

theta1 = theta1_n + phi; %2*pi/n;                         


%%%%%%%%% copy pasted from rimless_derive.m %%%%%%
J11 = 1;
J12 = 0;
J13 = l*(cos(phi + theta1_n) - cos(theta1_n));
J21 = 0;
J22 = 1;
J23 = l*(sin(phi + theta1_n) - sin(theta1_n));
J = [J11 J12 J13; J21 J22 J23];
 
A11 = M;
A12 = 0;
A13 = -M*l*cos(theta1_n);
A21 = 0;
A22 = M;
A23 = -M*l*sin(theta1_n);
A31 = -M*l*cos(theta1_n);
A32 = -M*l*sin(theta1_n);
A33 = I + M*l^2;
A_n_hs = [A11 A12 A13; A21 A22 A23; A31 A32 A33];
 
X_n_hs = [0 0 omega1_n]';
b_hs = [A_n_hs*X_n_hs; 0; 0];
A_hs = [A_n_hs -J' ; J zeros(2,2)];
X_hs = A_hs\b_hs;
omega1 = X_hs(3);
 

%%%%%%%%%%%%% ends %%%%%%%%%%%%%

zplus = [theta1 omega1];                     

%===================================================================
function flag = check_error(t_temp,z_temp,walker,phase)
%===================================================================

flag = 0;
M = walker.M; I = walker.I; l = walker.l; g=walker.g; gam = walker.gam;
for i=1:length(t_temp)
     theta1 = z_temp(i,1);
     omega1 = z_temp(i,2);
     A_ss = I + M*l^2;
     b_ss = -M*g*l*sin(gam - theta1);
     alpha1 = A_ss\b_ss;
    %zdot=single_stance(t_temp(i),z_temp(i,:),walker);
    %alpha1 = zdot(2);
    %Rx = -M*(g*sin(gam) + alpha1*l*cos(theta1) - l*omega1^2*sin(theta1));
    Ry(i,1) = -M*(alpha1*l*sin(theta1) - g*cos(gam) + l*omega1^2*cos(theta1));
    omega_all(i,1) = omega1;
    y_h(i,1) = l*cos(theta1);
end

index1 = find(Ry<=0);
if (~isempty(index1))
    flag = 1;
    disp(['flight phase in ',phase]);
end

index2 = find(omega_all>0);
if (~isempty(index2))
    %omega_all(index2)
    flag = 2;
    disp(['falling backward in phase ',phase]);
end

index3 = find(y_h<=0);
if (~isempty(index3))
    flag = 3;
    disp(['walker penetrated the ground in phase ',phase]);
end




