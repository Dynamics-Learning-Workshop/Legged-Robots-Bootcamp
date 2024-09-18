clc
close all

ball.g = 9.81;
ball.e = 0.9;
ball.ground = 0; %ground is at y co-ordinate equal to ball.ground
ball.ground_animate = -0.175; %this is only used for animation;
                            %distance equal the radius of the ball

ball.movieWrite = 0; %set to 1 to output a movie file
ball.moviePause = 0.05; %delay for the movie
ball.movieFps = 15; %frames per second. Ensures that same number of frames are there every second.
ball.movieName = 'bouncing_ball.avi';

y0 = 0; y0dot = 10;
ystart = [y0 y0dot];

t0 = 0; tend = 10;
tstart = t0;

t_all = tstart; y_all = ystart;
while(t0<tend)
    [t,y] = one_bounce(t0,ystart,ball);
    t_all = [t_all; t(2:end)];
    y_all = [y_all; y(2:end,:)]; 
    
    t0 = t(end);  
    ystart = y_all(end,:);
end


figure(1)
subplot(2,1,1);
plot(t_all,y_all(:,1),'r','Linewidth',2);
ylabel('Position','Fontsize',12);
subplot(2,1,2);
plot(t_all,y_all(:,2),'r','Linewidth',2);
ylabel('Velocity','Fontsize',12);
xlabel('Time');

figure(2)
animation(t_all,y_all,ball)

