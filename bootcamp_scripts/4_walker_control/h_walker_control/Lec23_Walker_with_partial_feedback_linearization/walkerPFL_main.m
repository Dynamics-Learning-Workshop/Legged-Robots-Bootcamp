clc
clear all
close all
format long
 
%%%% Dimensions %%
%%% c = COM on the leg from hip M = hip mass, m = leg mass, I = leg inertia, l = leg length
walker.M = 0.0; walker.m = 1; walker.I = 0.05; walker.l = 1.0; 
walker.c = 0.5; walker.g = 1; walker.gam = 0.02; 
fps = 20; %Use low frames per second for low gravity

walker.control.on = 0; %passive walking

%%%%% Stage 1: Get passive dynamic gait %%%%%
%%%% Initial State %%%%%
q1 = 0.2; u1 = -0.4;
q2 = -2*q1; u2 = 0.1;

z0 = [q1 u1 q2 u2];

% %%%% testing forward simulation  %%%
% steps = 1;
% figure(1)
% [z,t] = onestep(z0,walker,steps);
% animate(t,z,walker,steps,20);

%%%% Root finding, Period one gait %%%%
options = optimset('TolFun',1e-12,'TolX',1e-12,'Display','off');
[zstar,fval,exitflag] = fsolve(@fixedpt,z0,options,walker);
if exitflag == 1
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
disp('Note that three eigenvalues are nonzero');


%%% forward simulation to get step time and step angle  %%%
steps = 1;
[z,t] = onestep(zstar,walker,steps);

%zstar = [ 0.142821844397568  -0.320342245235108  -0.285643688795094   0.073672117908649];
%%%%%%% Stage 2: Fixed point and eigenvalues of controlled system %%%%
walker.control.on = 1;
walker.control.tf = 1.9; 
walker.control.Kp = 100;
walker.control.theta2f = 0.28564;

%%%% Root finding, Period one gait %%%%
options = optimset('TolFun',1e-12,'TolX',1e-12,'Display','off');
[zstar2,fval,exitflag] = fsolve(@fixedpt,zstar,options,walker);
if exitflag == 1
    disp('Fixed point:');
    disp(zstar2);
else
    error('Root finder not converged, change guess or change system parameters')
end

J=partialder(@onestep,zstar2,walker);
disp('EigenValues for linearized map are');
eigJ = eig(J);
for i=1:4
    disp(norm(eigJ(i)));
end
disp('Note that only one eigenvalue is nonzero, we have achieved dimensionality reduction');
disp('Also note that the largest eigenvalue is small than the one found previously');

%%% forward simulation to get step time and step angle  %%%
walker.control.on = 1;
walker.control.tf = 1.9; 
walker.control.Kp = 100;
walker.control.theta2f = 0.28564;

%zstar2 = [ 0.142819999999995  -0.326813112785275  -0.285640000000000   0.068243824367047];
%%% forward simulation to get step time and step angle  %%%
steps = 1;
[z,t] = onestep(zstar2,walker,steps);

%%% Stage 4: Put a perturbation and check response %%%%%%%
%walker.control.theta2f = theta2f_0;
z_pert = zstar2 + [0 0.05 -0.1 0.2];
steps = 5;
[z,t,z_ref] = onestep(z_pert,walker,steps);
disp('Animating...');
figure(1)
animate(t,z,walker,steps,fps);

%%% Plot data %%%
disp('Some plots...')
figure(2)
subplot(2,1,1);
title('pfl walker position and velocity as a function of time');
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


figure(3)
subplot(2,1,1);
title('pfl walker absolute leg angle rate versus absolute leg angle');
plot(z(:,1),z(:,2),'r','Linewidth',3); hold on;
plot(z(1,1),z(1,2),'ko','Markersize',10,'MarkerFaceColor','k');
text(z(1,1)+0.01,z(1,2)+0.01,'start','Fontsize',12)
ylabel('$\dot{\theta}_1$','Fontsize',12,'Interpreter','Latex');
xlabel('\theta_1','Fontsize',12);
subplot(2,1,2);
plot(z(:,1)+z(:,3),z(:,2)+z(:,4),'b','Linewidth',2); hold on;
plot(z(1,1)+z(1,3),z(1,2)+z(1,4),'ko','Markersize',10,'MarkerFaceColor','k');
text(z(1,1)+z(1,3)+0.01,z(1,2)+z(1,4)+0.01,'start','Fontsize',12)
ylabel('$\dot{\theta}_1+\dot{\theta}_2$','Fontsize',12,'Interpreter','Latex');
xlabel('\theta_1+\theta_2','Fontsize',12);

figure(4)
subplot(2,1,1)
plot(t,z_ref(:,1),'k-.','Linewidth',2); hold on
plot(t,z(:,3),'r');
ylabel('theta_2');
legend('reference','actual');
subplot(2,1,2)
plot(t,z_ref(:,2),'k-.','Linewidth',2); hold on
plot(t,z(:,4),'r');
ylabel('thetadot_2');


% figure(5)
% plot(t,z_ref(:,3))
