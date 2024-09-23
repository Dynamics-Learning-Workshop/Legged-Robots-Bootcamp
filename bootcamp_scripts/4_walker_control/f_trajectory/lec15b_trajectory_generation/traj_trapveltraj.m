function traj_trapveltraj

t0 = 0;
tf = 8;
q_pts = [1 5 6 3 2 7];

% Compute trajectory using default boundary conditions.
[q, qdot, qddot, t_pts, pp] = trapveltraj(q_pts, 10*length(q_pts));

%%%%%%% we want the output time t to be between t0 and tf
%%% trapveltraj gives t_pts which is [0, 1, ... ,length(q_pts)]
%%%%%% size of t_pts is length(q_pts)+1
%%%% (1) lets scale t_pts by length(q_pts) so t_pts goes from 0 to 1
t_pts = t_pts/(length(q_pts)-1);
t_pts(1)
t_pts(end)
%%%% (2) Now lets scale by (tf-t0) %%%%%%
t_pts = (tf-t0)*t_pts;
%%%% (3) finally, lets translate all by t0
t_pts = t0+t_pts;
%%%%% (4) t output is same as t_pts 
t = t_pts; 


figure(1)
subplot(2,2,1)
plot(t,q); ylabel('$q$','Interpreter','latex');  xlabel('t');

subplot(2,2,2)
plot(t,qdot); ylabel('$\dot{q}$','Interpreter','latex'); xlabel('t'); 

subplot(2,2,3)
plot(t,qddot); ylabel('$\ddot{q}$','Interpreter','latex'); xlabel('t');

suptitle('trapveltraj')