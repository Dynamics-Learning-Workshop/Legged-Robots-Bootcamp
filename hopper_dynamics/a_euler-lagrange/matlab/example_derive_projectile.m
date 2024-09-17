clc
clear all
close all

%%%%% symbolic quantities
syms x y real
syms xdot ydot real
syms xddot yddot real %acceleration
syms m c g real

%%%%%% 1) position and velocity  %%%%%
% position are x and y 
% velocities are xdot and ydot 

%%%%%% 2) Kinetic and potential energy %%%%%
T = 0.5*m*(xdot^2+ydot^2);
V = m*g*y;
L = T-V;

%%%% 3) Euler-Lagrange equations %%%%%%
Fx = -c*xdot*sqrt(xdot^2+ydot^2);
Fy = -c*ydot*sqrt(xdot^2+ydot^2);

%%%%% method 1: writing manually 
%%% Since L(x,y,xdot,ydot) we have %%
%%% ddt(dLdqdot) - dLdq = Q
% q= x and q=y
dLdxdot = diff(L,xdot); %dLdxdot

%%% dLdqdot --> x,xdot,y,ydot --> t
ddt_dLdxdot = diff(dLdxdot,x)*xdot + diff(dLdxdot,xdot)*xddot + ...
              diff(dLdxdot,y)*ydot + diff(dLdxdot,ydot)*yddot; %chain rule
dLdx = diff(L,x); %dLdx
EOM1 = ddt_dLdxdot - dLdx -Fx; %EOM1 = 0

dLdydot = diff(L,ydot);
ddt_dLdydot = diff(dLdydot,x)*xdot + diff(dLdydot,xdot)*xddot + ...
              diff(dLdydot,y)*ydot + diff(dLdydot,ydot)*yddot; %chain rule
dLdy = diff(L,y);
EOM2 = ddt_dLdydot - dLdy -Fy;


solve(EOM1,xddot)
solve(EOM2,yddot)

%%%%% method 2: automation 
q = [x y]; %position 
qdot = [xdot ydot]; %velocity 
qddot = [xddot yddot]; %acceleration
F = [Fx Fy];

for ii=1:2
    dLdqdot(ii) = diff(L,qdot(ii));
    ddt_dLdqdot(ii) = diff(dLdqdot(ii),q(1))*qdot(1)+ ...
                      diff(dLdqdot(ii),qdot(1))*qddot(1)+...
                     diff(dLdqdot(ii),q(2))*qdot(2)+ ...
                     diff(dLdqdot(ii),qdot(2))*qddot(2);
    dLdq(ii) = diff(L,q(ii));

    EOM(ii) = ddt_dLdqdot(ii) - dLdq(ii) - F(ii);
end
solve(EOM(1),xddot)
solve(EOM(2),yddot)
% 
