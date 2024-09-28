
function [t,theta,thetadot,thetaddot] = twolink_traj(parms,h,t0,tN)


%step size for integration. Accuracy increases as h decreases
h_temp = 0.05;

%set the time
%t0 = 0; %initial time
%tN = 2; %end time
N_temp = (tN-t0)/h_temp;
N = (tN-t0)/h;
t_temp = linspace(t0,tN,N_temp);
t = linspace(t0,tN,N);
x0 = 1; y0 = 0;
[xd_temp,yd_temp,xd_dot_temp,yd_dot_temp,xd_ddot_temp,yd_ddot_temp] = figure8(x0,y0,t_temp);


%intial guess
theta1_guess = 0.1; theta2_guess = 0;
theta_temp = zeros(length(t_temp),2);
for i=1:length(t_temp)
    Xdes(1) = xd_temp(i);
    Xdes(2) = yd_temp(i);
    options=optimset('Display','off');
    [X,FVAL,EXITFLAG] = fsolve('twolink_end_effector_position',[theta1_guess,theta2_guess],options,parms,Xdes);
    theta_temp(i,1) = X(1); theta_temp(i,2) = X(2);
    theta1_guess = X(1); theta2_guess = X(2);
    %convergence_flag(i) = EXITFLAG;
end
%convergence_flag

theta = zeros(length(t),2);
Xd = zeros(length(t),2);
Xd_dot = zeros(length(t),2);
Xd_ddot = zeros(length(t),2);
for i=1:length(t)
    theta(i,1) = interp1(t_temp,theta_temp(:,1),t(i)); 
    theta(i,2) = interp1(t_temp,theta_temp(:,2),t(i));
    Xd(i,1) = interp1(t_temp,xd_temp,t(i));
    Xd(i,2) = interp1(t_temp,yd_temp,t(i));
    Xd_dot(i,1) = interp1(t_temp,xd_dot_temp,t(i));
    Xd_dot(i,2) = interp1(t_temp,yd_dot_temp,t(i));
    Xd_ddot(i,1) = interp1(t_temp,xd_ddot_temp,t(i));
    Xd_ddot(i,2) = interp1(t_temp,yd_ddot_temp,t(i));
end

%now convert x,y,xdot,ydot,xddot,yddot to theta, thetadot, thetaddot
thetadot = zeros(length(t),2);
thetaddot = zeros(length(t),2);
for i=1:length(t)    
    J = jacobian_endeffector(theta(i,1),theta(i,2),parms);
    thetadot_temp = J\(Xd_dot(i,:)');
    thetadot(i,1) = thetadot_temp(1); thetadot(i,2) = thetadot_temp(2);
    
    Jdot = jacobiandot_endeffector(theta(i,1),theta(i,2),thetadot(i,1),thetadot(i,2),parms);
    thetaddot_temp = J\(Xd_ddot(i,:)' - Jdot*Xd_dot(i,:)');
    thetaddot(i,1) = thetaddot_temp(1); thetaddot(i,2) = thetaddot_temp(2);
end

%For debugging code
% figure(1)  %animation
% twolink_animation(t,[theta(:,1), theta(:,2)],parms);
% 
% figure(2)
% plot(t,theta(:,1),'r',t,theta(:,2),'b--');
% ylabel('position');
% 
% 
% figure(3)
% plot(t,thetadot(:,1),'r',t,thetadot(:,2),'b--');
% ylabel('velocity');
% 
% 
% figure(4)
% plot(t,thetaddot(:,1),'r',t,thetaddot(:,2),'b--');
% ylabel('acceleration');