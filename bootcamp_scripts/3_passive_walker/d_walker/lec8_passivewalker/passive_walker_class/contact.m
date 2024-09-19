function [gstop, isterminal,direction]=contact(t,z,walker)

theta1 = z(1);
theta2 = z(3);

%%%%% notes had an error, theta2 = 2*theta1
%%%%%%% the correct answer is theta2 = -2*theta1
%theta1 = -(theta1-theta2)

gstop = theta2 + 2*theta1; %find the event that ball y-position - ball.ground = 0
%[gstop theta1]
if (theta1>-0.1)
    isterminal = 0; %continue
else
    isterminal = 1; %stop integration
end
direction = []; %because ball goes from + heigth to 0 to negative height 
                %(other option =1, positive direction and 0, either direction
%isterminal = 1;  %1 is stop the integration, (other options is 0 not to step integration)
