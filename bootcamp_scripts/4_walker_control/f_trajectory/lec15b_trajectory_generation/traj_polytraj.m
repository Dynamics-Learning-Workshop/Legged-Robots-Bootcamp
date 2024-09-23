function traj_polytraj
close all

flag = 1; %input 1 for cubicpolytraj or other numeric for quinticpolytraj

if (flag==1)
    string = 'Using cubicpolytraj';
else
    string = 'Using quinticpolytraj';
end

t0 = 0;
tf = 8;
q_pts = [1 5 6 3 2 7];
t_pts = linspace(t0,tf,length(q_pts));

t = linspace(t0,tf,10*length(q_pts));

if (flag==1)
    [q, qdot, qddot, pp] = cubicpolytraj(q_pts, t_pts,t);
else
    [q, qdot, qddot, pp] = quinticpolytraj(q_pts, t_pts,t);
end

figure(1)
subplot(2,2,1)
plot(t,q); ylabel('$q$','Interpreter','latex');  xlabel('t');

subplot(2,2,2)
plot(t,qdot); ylabel('$\dot{q}$','Interpreter','latex'); xlabel('t'); 

subplot(2,2,3)
plot(t,qddot); ylabel('$\ddot{q}$','Interpreter','latex'); xlabel('t');

suptitle(string)
