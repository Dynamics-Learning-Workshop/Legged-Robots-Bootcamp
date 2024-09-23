clc
clear all
close all

walker = walker_parms; 
walker.thetadot_des = [-0.5 -1 -1.2 -0.9 -0.7 -0.7 -1 -1.5];
walker.steps = length(walker.thetadot_des);
phi = []; %dont change this                     
fps = 30; %Use low frames per second for low gravity

%%%%%%%% initialize %%%%%
thetadot0 = walker.thetadot_des(1); %initial speed at step 1
[thetadot,flag,z,t] = walker_sim(thetadot0,phi,walker);
v_apex = thetadot(:,2);
%%% flag = [failure in mid 2 collision, failure in collision 2 mid] %%
%%%% is a step_no x 2 array as given above 
if (find(flag>0))
    disp('simulation failed');
end

%%%% Animate result %%%
disp('Animating...');
figure(1)
walker_animate(t,z,walker,fps);

%%% Plot data %%%
disp('Some plots...')
figure(2)
subplot(2,1,1);
title('passive walker position and velocity as a function of time');
plot(t,z(:,1),'k','Linewidth',3); hold on
ylabel('position','Fontsize',12);
subplot(2,1,2)
plot(t,z(:,2),'k','Linewidth',3); hold on
ylabel('velocity','Fontsize',12);
xlabel('time','Fontsize',12);

figure(3)
stairs(walker.thetadot_des,'k--','Linewidth',2); hold on
stairs(v_apex,'r','Linewidth',2); 
legend('desired','actual');
ylabel('mid stance velocity');
xlabel('step number');
