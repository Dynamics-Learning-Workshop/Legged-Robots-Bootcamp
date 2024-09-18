function dqdt = rhs(t,y,ball)

ydot = y(2);
yddot = -ball.g;

dqdt = [ydot yddot]';

