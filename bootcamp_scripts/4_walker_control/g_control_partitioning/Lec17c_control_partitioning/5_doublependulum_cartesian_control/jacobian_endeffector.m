function J = jacobian_endeffector(theta1,theta2,parms)

l1 = parms.l1;
l2 = parms.l2;

J(1,1) = - l2*sin(theta1 + theta2) - l1*sin(theta1);
J(1,2) = -l2*sin(theta1 + theta2);
J(2,1) = l2*cos(theta1 + theta2) + l1*cos(theta1);
J(2,2) = l2*cos(theta1 + theta2);
 
