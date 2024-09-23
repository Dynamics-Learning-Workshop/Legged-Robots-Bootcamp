%%%%% File for generating data %%%%%%%

clc
clear all
close all

walker = walker_parms; 
fps = 30; %Use low frames per second for low gravity

%%%%%%%% initialize %%%%%
thetadot0 = -1; %initial speed at step 1 thetadot_i
phi = pi/6; % phi_i
%phi = [pi/6 pi/10 pi/6]; %phi array for multiple steps, gives corresponding thetadots                     
%phi = pi/6*ones(1,5); %phi array for multiple steps, gives corresponding thetadots                     
[thetadot,flag,z,t] = walker_sim(thetadot0,phi,walker);
%%% flag = [failure in mid 2 collision, failure in collision 2 mid] %%
%%%% is a step_no x 2 array as given above 
if (find(flag>0))
    disp('simulation failed');
end

disp('input')
disp(phi)
disp(thetadot0)

disp('output')
disp(thetadot(:,2))

%%%% Animate result %%%
disp('Animating...');
figure(2)
walker_animate(t,z,walker,fps);

%%% Plot data %%%
disp('Some plots...')
figure(1)
subplot(2,1,1);
title('passive walker position and velocity as a function of time');
plot(t,z(:,1),'k','Linewidth',3); hold on
ylabel('position','Fontsize',12);
subplot(2,1,2)
plot(t,z(:,2),'k','Linewidth',3); hold on
ylabel('velocity','Fontsize',12);
xlabel('time','Fontsize',12);
