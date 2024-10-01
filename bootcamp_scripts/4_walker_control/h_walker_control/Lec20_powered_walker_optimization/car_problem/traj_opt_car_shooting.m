clc
clear all
close all

rng(1); %random number generator to get repeatable u_opt
D = 5; %total distance
z0 = [0 0]; %initial state
N = 10;

T_min = 1; T_max = 3;
u_min = -5; u_max = 5;
T_opt = 1;
t_opt = linspace(0,T_opt,N+1);
u_opt = u_min + (u_max-u_min)*rand(1,N+1); %piecewise linear control u_opt


FUN = @cost;
X0 = [T_opt u_opt];
A = []; B = []; Aeq = []; Beq = [];
LB = [T_min u_min*ones(1,N+1)];
UB = [T_max u_max*ones(1,N+1)];
NONLCON = @(x)constraints(x,z0,N,D);
OPTIONS = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[X,FVAL,EXITFLAG] = fmincon(FUN,X0,A,B,Aeq,Beq,LB,UB,NONLCON,OPTIONS);

X
FVAL

[tt,zz,uu] = simulator(X,z0,N);
subplot(3,1,1);
plot(tt,zz(:,1)); ylabel('position');
subplot(3,1,2);
plot(tt,zz(:,2)); ylabel('velocity');
subplot(3,1,3);
plot(tt,uu); ylabel('control');

function F = cost(x)
    F = x(1); %Is time
end

function [c,ceq] = constraints(x,z0,N,D)
    [tt,zz] = simulator(x,z0,N);
    c = [];
    x_end = zz(end,1)-D;
    v_end = zz(end,2);
    ceq = [x_end v_end];
end

function [tt,zz,uu] = simulator(x,z0,N)
T = x(1);
zz = z0;
tt = 0;
u_opt = x(2:end);
t_opt = linspace(0,T,N+1);
for i=1:N
    options=odeset('abstol',1e-13,'reltol',1e-13);
    [tt_temp, zz_temp] = ode113(@car,[t_opt(i) t_opt(i+1)],z0,options,t_opt,u_opt);
    z0 = zz_temp(end,:);
    zz = [zz; zz_temp(2:end,:)];
    tt = [tt; tt_temp(2:end)];
end
uu = interp1(t_opt,u_opt,tt);

end
 
function zdot = car(t,z,t_opt,u_opt)
    u = interp1(t_opt,u_opt,t);
    zdot = [z(2), u]';
end
