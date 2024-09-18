clc
close all
format long

walker.M = 1.0; walker.m = 0.5; walker.I = 0.02; walker.l = 1.0; 
walker.c = 0.5; walker.g = 1.0; walker.gam = 0.01; 
walker.movieFps = 50;

theta1 = 0.18; theta1dot = -0.25;
theta2 = -2*theta1; theta2dot = 0.1;
z0 = [theta1 theta1dot theta2 theta2dot]; %initial conditions
%one root is 0.162597833780052  -0.231869638058967  -0.325195667560120   0.037978468073739

%%% root finding %%%%%%%
options = optimoptions('fsolve','Display','iter','TolFun',1e-12);
[zstar,fval,exitflag] = fsolve(@(x) fixedpt(x,walker),z0,options);
exitflag
zstar
fval

%%%% check the stability of the fixed point %%%%%%
J=partialder(@onestep,zstar,walker);
%J
%eigJ = eig(J)
disp('norm of the eigenvalues of J')
for i=1:4 %always one eigenvalue is 0
    disp(norm(eigJ(i,:))) %maximum of the norm of the eigenvalue of J
end
%since 0.4<1 the fixed point is stable


%%%%%%% a simulation %%% 
t0 = 0;
[z,t] = onestep(t0,z0,walker);

t_all = t;
xh = -walker.l*sin(z(:,1));
yh = walker.l*cos(z(:,1));
z_all = [z, xh, yh];


%%%% animation %%%%%
figure(1)
animation(t_all,z_all,walker)

%%%%% plots %%%%%
figure(2)
plot(t,z(:,1),'r',t,z(:,3),'b--');
legend('theta1','theta2');



