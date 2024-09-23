function Jdot = jacobiandot_endeffector(theta1,theta2,theta1dot,theta2dot,parms)

l1 = parms.l1;
l2 = parms.l2;

Jdot(1,1) = - theta1dot*(l2*cos(theta1 + theta2) + l1*cos(theta1)) - l2*theta2dot*cos(theta1 + theta2);
Jdot(1,2) = -l2*cos(theta1 + theta2)*(theta1dot + theta2dot);
Jdot(2,1) = - theta1dot*(l2*sin(theta1 + theta2) + l1*sin(theta1)) - l2*theta2dot*sin(theta1 + theta2);
Jdot(2,2) = -l2*sin(theta1 + theta2)*(theta1dot + theta2dot);