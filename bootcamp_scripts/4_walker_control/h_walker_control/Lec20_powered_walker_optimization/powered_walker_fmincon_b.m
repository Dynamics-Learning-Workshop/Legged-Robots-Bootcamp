clc
clear all
close all
format long

walker.M = 1.0; walker.m = 0.5; walker.I = 0.02; walker.l = 1.0; 
walker.c = 0.5; walker.g = 1.0; walker.gam = 0*0.01; 
%walker.control.P = 0.1; 
walker.N = 4;


z_ss0 = [0.183500818246401  -0.273335992124201  -0.367001636492817   0.031383021226136];
z_bfs = [-0.183500818246437  -0.273335992124290   0.367001636492874   0.031383021225914];
%Torque = []
time = 2.449570273199906;

%%% check %%%%
% x = [T z_ss0 z_bfs];
% [tt,zz,uu] = simulator(x,walker);


X_solution = [time z_ss0 z_bfs]; %from fsolve

N = walker.N;

P_min = 0.1; P_max = 0.4;
u_min = -2; u_max = 2;
t_opt = linspace(0,time,N+1);
u_opt = u_min + (u_max-u_min)*rand(1,N+1); %piecewise linear control u_opt



time_min = 0.3; time_max = 0.6;
time = 0.5*(time_min+time_max);

%z_ss0 = [0.15 -0.2 -0.3 0];
%z_bfs = [-0.15 -0.2 0.3 0];
P = 0.1;
X0 = [time z_ss0 z_bfs P u_opt];
      %[1 4 4 N+1];

%[c,ceq] = constraints(X0,walker);



%z_ss0 = [0.183500818246401  -0.273335992124201  -0.367001636492817   0.031383021226136];
z_ss_lb = [0.1 -2 -2*0.4 -4];
z_ss_ub = [0.4 -0.1 -2*0.1 4];

z_bfs_lb = [-0.4 -2 2*0.1 -4];
z_bfs_ub = [-0.1 -0.1 2*0.4 4];

u_lb = u_min*ones(1,N+1);
u_ub = u_max*ones(1,N+1);

% u_min = -5; u_max = 5;
% T_opt = 1;
% t_opt = linspace(0,T_opt,N+1);
% u_opt = u_min + (u_max-u_min)*rand(1,N+1); %piecewise linear control u_opt
% 
% 
% %FUN = @cost;
% X0 = [T_opt u_opt];
A = []; B = []; Aeq = []; Beq = [];
LB = [time_min z_ss_lb z_bfs_lb P_min u_lb];
UB = [time_max z_ss_ub z_bfs_ub P_max u_ub];
% %NONLCON = @(x)constraints(x,z0,N,D);
%'sqp'
%'OptimalityTolerance',1e-8
%  X0 = [    0.625000000000051
%    0.252680255142068
%   -0.940550165914161
%   -0.505360510284137
%    0.095383786672185
%   -0.252680255142068
%   -0.792344988606115
%    0.505360510284137
%    2.634104004771229
%    0.266435193124297
%    0.540753859973618
%    0.898233349852360
%    0.636195740069337
%    0.379574560976129
%    0.105046242094005]';
%size(X0)
OPTIONS = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[X,FVAL,EXITFLAG] = fmincon(@cost,X0,A,B,Aeq,Beq,LB,UB,@constraints,OPTIONS,walker);
%FVAL
EXITFLAG 

%[c,ceq] = constraints(X,walker)

%disp([X(1:9)' X_solution'])

disp('solution')
disp(X')

disp('cost')
disp(cost(X,walker))

[c,ceq] = constraints(X,walker);
disp('equality constraints');
disp(ceq')
 
%disp('Copy paste in poweredwalker.m')
%disp(X(2:5)')


%X
%%%%%%%%%
%[z_ssT,z_afs] = simulator(x,walker)
[z_ssT,z_afs,z,t] = simulator(X,walker);

%%% Animate result %%%
disp('Animating...');
fps = 30;
figure(1)
animate(t,z,walker,fps);

time = X(1);
u = X(11:end);
t = linspace(0,time,N+1);
figure(2);
plot(t,u);
xlabel('time');
ylabel('torque');

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
    %F = 0;
    time = x(1);
    N = walker.N;
    dt = time/N;
    %add torque^2 dt
    P = x(10);
    u_opt = x(11:11+N);
    %F = sum(abs(u_opt))*dt + P;
     F = sum(u_opt.*u_opt)*dt + P^2;
end

function [c,ceq] = constraints(x,walker)
    time = x(1);
    z_ss0 = x(2:5);
    z_bfs = x(6:9);
    
    theta2_bfs = z_bfs(3); 
    theta1_bfs = z_bfs(1);
    
    collision_condition = theta2_bfs + 2*theta1_bfs;
    [z_ssT,z_afs,~,~,d_step,v_step] = simulator(x,walker);
    %[z_ssT,z_afs,z,t,d_step,v_step] = simulator(x,walker)
    c =[];
    ceq = [z_ss0 - z_afs,z_ssT - z_bfs, collision_condition, (d_step-0.5), (v_step-1)];
    

end

function [z_ssT,z_afs,z,t,d_step,v_step] = simulator(x,walker)
%x = [T z_ss0 z_bfs0];
N = walker.N;

time = x(1);
z_ss0 = x(2:5);
z_bfs = x(6:9);
P = x(10);

%N = 100;
options=odeset('abstol',1e-13,'reltol',1e-13);
t_opt = linspace(0,time,N+1);
u_opt = x(11:11+N);
%%% do this tspan = linspace(0,time,N+1);
%%% needs a for loop for torque
z0 = z_ss0;
z_temp = z0;
t_temp = 0;
% x(10:end)
% u_opt
% size(t_opt)
% size(u_opt)
for i=1:N
      [tt_temp, zz_temp] = ode113(@single_stance2,[t_opt(i) t_opt(i+1)],z0,options,walker,t_opt,u_opt);
      %[t_temp, z_temp] = ode113(@single_stance,tspan,z_ss0,options,walker);
      %[tt_temp, zz_temp] = ode113(@car,[t_opt(i) t_opt(i+1)],z0,options,t_opt,u_opt);
      z0 = zz_temp(end,:);
      z_temp = [z_temp; zz_temp(2:end,:)];
      t_temp = [t_temp; tt_temp(2:end)];
end

z_ssT = z_temp(end,1:4);

z_afs =footstrike2(0,z_bfs,walker,P);

l = walker.l;
d_step = 2*l*sin(0.5*z_ssT(3));
t_step = time;
v_step = d_step/t_step;

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
 

