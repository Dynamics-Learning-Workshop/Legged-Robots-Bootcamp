function dqdt = projectile_rhs(t,q,projectile)

g = projectile.g;
m = projectile.m;
c = projectile.c; 


%x = q(1);
vx = q(2);
%y = q(3);
vy = q(4);
v = sqrt(vx^2+vy^2);

%%%% net acceleration %%%
ax = 0-(c*v*vx/m);
ay = -g-(c*v*vy/m);

%xdot = vx
%xddot = ax = eqn1 in the notes
%ydot = vy
%yddot = ay = eqn2 in the notes
dqdt = [vx ax vy ay]';

