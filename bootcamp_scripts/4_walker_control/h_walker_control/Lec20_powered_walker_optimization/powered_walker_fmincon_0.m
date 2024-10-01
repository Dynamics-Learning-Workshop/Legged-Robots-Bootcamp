clc
clear all
close all
format long

walker.M = 1.0; walker.m = 0.5; walker.I = 0.02; walker.l = 1.0; 
walker.c = 0.5; walker.g = 1.0; walker.gam = 0*0.01; 
walker.control.P = 0.1;


z_ss0 = [0.183500818246401  -0.273335992124201  -0.367001636492817   0.031383021226136];
z_bfs = [-0.183500818246437  -0.273335992124290   0.367001636492874   0.031383021225914];
%Torque = []
time = 2.449570273199906;

%%% check %%%%
% x = [T z_ss0 z_bfs];
% [tt,zz,uu] = simulator(x,walker);


X_solution = [time z_ss0 z_bfs]; %from fsolve

time = 3;
z_ss0 = [0.15 -0.2 -0.3 0];
z_bfs = [-0.15 -0.2 0.3 0];
X0 = [time z_ss0 z_bfs];

%[c,ceq] = constraints(X0,walker);



time_min = 1; time_max = 3;
%z_ss0 = [0.183500818246401  -0.273335992124201  -0.367001636492817   0.031383021226136];
z_ss_lb = [0.1 -0.5 -2*0.3 -0.5];
z_ss_ub = [0.3 -0.1 -2*0.1 0.5];

z_bfs_lb = [-0.3 -0.5 2*0.1 -0.5];
z_bfs_ub = [-0.1 -0.1 2*0.3 0.5];
% u_min = -5; u_max = 5;
% T_opt = 1;
% t_opt = linspace(0,T_opt,N+1);
% u_opt = u_min + (u_max-u_min)*rand(1,N+1); %piecewise linear control u_opt
% 
% 
% %FUN = @cost;
% X0 = [T_opt u_opt];
A = []; B = []; Aeq = []; Beq = [];
LB = [time_min z_ss_lb z_bfs_lb];
UB = [time_max z_ss_ub z_bfs_ub];
% %NONLCON = @(x)constraints(x,z0,N,D);
OPTIONS = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[X,FVAL,EXITFLAG] = fmincon(@cost,X0,A,B,Aeq,Beq,LB,UB,@constraints,OPTIONS,walker);
FVAL
EXITFLAG 

%[c,ceq] = constraints(X,walker)

disp([X' X_solution'])
disp('Copy paste in poweredwalker.m')
disp(X(2:5)')

%X
%%%%%%%%%
%[z_ssT,z_afs] = simulator(x,walker)
[z_ssT,z_afs,z,t] = simulator(X,walker);

%%% Animate result %%%
disp('Animating...');
fps = 30;
figure(1)
animate(t,z,walker,fps);


%X = fmincon(FUN,X0,A,B,Aeq,Beq,LB,UB,NONLCON)
% 
% X
% FVAL
% 
% [tt,zz,uu] = simulator(X,z0,N);
% subplot(3,1,1);
% plot(tt,zz(:,1)); ylabel('position');
% subplot(3,1,2);
% plot(tt,zz(:,2)); ylabel('velocity');
% subplot(3,1,3);
% plot(tt,uu); ylabel('control');
% 
function F = cost(x,walker)
    F = 0;  %add torque^2 dt
end

function [c,ceq] = constraints(x,walker)
    time = x(1);
    z_ss0 = x(2:5);
    z_bfs = x(6:9);
    
    theta2_bfs = z_bfs(3); 
    theta1_bfs = z_bfs(1);
    
    collision_condition = theta2_bfs + 2*theta1_bfs;
    [z_ssT,z_afs] = simulator(x,walker);
    c =[];
    ceq = [z_ss0 - z_afs,z_ssT - z_bfs, collision_condition];
    

end

function [z_ssT,z_afs,z,t] = simulator(x,walker)
%x = [T z_ss0 z_bfs0];
time = x(1);
z_ss0 = x(2:5);
z_bfs = x(6:9);
N = 100;
options=odeset('abstol',1e-13,'reltol',1e-13);
tspan = linspace(0,time,N);
%%% do this tspan = linspace(0,time,N+1);
%%% needs a for loop for torque
[t_temp, z_temp] = ode113(@single_stance,tspan,z_ss0,options,walker);
z_ssT = z_temp(end,1:4);

z_afs =footstrike(0,z_bfs,walker);

%%%% pass all the states and time
l = walker.l;
xh =  l*sin(z_temp(1,1))-l*sin(z_temp(:,1)); 
yh =  l*cos(z_temp(:,1));

z = [z_temp, xh yh];
t = t_temp;
%%%%%%%%%%%%%%%%%%%
    
%tt = []; zz = []; uu=[];
% T = x(1);
% u_opt = x(2:end);
% t_opt = linspace(0,T,N+1);
% zz = z0;
% tt = 0;
% for i=1:N
%     options=odeset('abstol',1e-13,'reltol',1e-13);
%     [tt_temp, zz_temp] = ode113(@car,[t_opt(i) t_opt(i+1)],z0,options,t_opt,u_opt);
%     z0 = zz_temp(end,:);
%     zz = [zz; zz_temp(2:end,:)];
%     tt = [tt; tt_temp(2:end)];
% end
% uu = interp1(t_opt,u_opt,tt);
% 
end
 

