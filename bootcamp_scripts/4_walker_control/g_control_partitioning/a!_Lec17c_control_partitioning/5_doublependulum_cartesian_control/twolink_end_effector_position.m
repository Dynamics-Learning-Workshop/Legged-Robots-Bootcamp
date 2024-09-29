function X = twolink_end_effector_position(theta,parms,Xdes)

%global a1 alpha1 d1 a2 alpha2 d2 %these are defined in inverse_kinematics
%global x_des y_des z_des
x_des = Xdes(1);
y_des = Xdes(2);

l1 = parms.l1;
l2 = parms.l2;

% Get individual theta's from input
theta1 = theta(1);
theta2 = theta(2);

x1 = l1*cos(theta1); 
y1 = l1*sin(theta1);
x2 = x1 + l2*cos(theta1+theta2);
y2 = y1 + l2*sin(theta1+theta2);
    
    
    %%% Get locations of joints
    %Location of joint 1
   % endOfLink1 = [x1 y1];
    
    %Location of joint 2
    endOfLink2 = [x2 y2];

%x,y,z of end of link2, X is the output
%X = [endOfLink2(1)-x_des; endOfLink2(2)-y_des; endOfLink2(3)-z_des]; 
X = [endOfLink2(1)-x_des; endOfLink2(2)-y_des]; 