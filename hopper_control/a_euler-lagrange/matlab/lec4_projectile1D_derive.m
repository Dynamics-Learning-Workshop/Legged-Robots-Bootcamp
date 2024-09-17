clc
clear all

syms m g y ydot yddot real

T = 0.5*m*ydot*ydot;
V = m*g*y;
L = T-V;

%%%%%%%% ddt(dL/dydot) - dLdy = 0 %%
dLdy = diff(L,y);
dLdydot = diff(L,ydot);

%%%%%%%%% maybe not explained well %%%%%
%%% ddt_F = (df/dy)*(dy/dt) %chain rule
%ddt_dLdydot = diff(dLdydot,dy)*dy/dt + diff(dLdydot,dydot)*dydot/dt
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% Better explanation (Chain rule) %%%%%%%%
%dLdydot = F
%dF/dt where F(y,ydot)
%dF/dt = (dF/dy)*(dy/dt) + (dF/dydot)*(dydot/dt)
%dF/dt = (dF/dy)*ydot + (dF/dydot)*yddot
ddt_dLdydot = diff(dLdydot,y)*ydot + diff(dLdydot,ydot)*yddot;

ddt_dLdydot - dLdy
%%% yddot = -g