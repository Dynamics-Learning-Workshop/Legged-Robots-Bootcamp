clc
close all

projectile.g = 9.81;
projectile.ground = 0; %ground is at y co-ordinate equal to projectile.ground
projectile.c = 0.5; %0.7;
projectile.m = 1;

projectile.movieWrite = 1; %set to 1 wto output a movie file
projectile.moviePause = 0.05; %delay for the movie
projectile.movieFps = 30; %frames per second. Ensures that same number of frames are there every second.
projectile.movieName = 'projectile.avi';

x0 = 0; x0dot = 100; %50
y0 = 0; y0dot = x0dot*tan(pi/3);
qstart = [x0 x0dot y0 y0dot];

t0 = 0; tend = 5;
tstart = t0;

t_all = tstart; q_all = qstart;
options = odeset('Abstol',1e-12,'Reltol',1e-12);
[t,q]=ode45(@projectile_rhs,[t0 tend],qstart,options,projectile);

t_all = [t_all; t(2:end)];
q_all = [q_all; q(2:end,:)];
    
figure(1)
subplot(2,2,1);
plot(t_all,q_all(:,1),'r');
ylabel('Position x');
subplot(2,2,2);
plot(t_all,q_all(:,3),'r');
ylabel('Position y');
subplot(2,2,3);
plot(t_all,q_all(:,2),'b');
ylabel('Velocity x');
subplot(2,2,4);
plot(t_all,q_all(:,4),'b');
ylabel('Velocity y');
xlabel('Time');

figure(2)
projectile_animation(t_all,q_all,projectile)


