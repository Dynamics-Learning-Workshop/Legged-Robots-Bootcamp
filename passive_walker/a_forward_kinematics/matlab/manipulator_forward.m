clc
clear all
close all

%%%% define parameters for the two-link %%%%%%
l1 = 1; l2 = 1;
theta1 = 0.5; theta2 = pi/2;


%%%%%%%% prepping to get homogenous transformations %%%%
c1 = cos(theta1); s1 = sin(theta1);
c2 = cos(theta2); s2 = sin(theta2);
O01 = [0; 0]; O12 = [l1; 0];
R01 = [c1 -s1; s1 c1];
R12 = [c2 -s2; s2 c2];
H01 = [R01, O01; 0, 0, 1];
H12 = [R12, O12; 0, 0, 1];

%%%%%%%% origin  in world frame  %%%%%%
o = [0 0];

%%%%% end of link1 in world frame %%%%
P1 = [l1; 0; 1];
P0 = H01*P1;
p = P0(1:2); %same as p0 

%%%% end of link 2 in world frame  %%%%%%%
Q2 = [l2; 0; 1];
Q0 = H01*H12*Q2;
q = Q0(1:2); %same as q0

%Draw line from origin to end of link 1
line([o(1) p(1)],[o(2) p(2)],'LineWidth',5,'Color','red');

%Draw line from end of link 1 to end of link 2
line([p(1) q(1)],[p(2) q(2)],'LineWidth',5,'Color','blue');

xlabel('x'); ylabel('y');
grid on; %if you want the grid to show up.
axis('equal'); %make the axis equal, to avoid scaling effect
 
% These set the x and y limits for the axis (will need adjustment)
xlim([-2 2]);  
ylim([-2 2]);