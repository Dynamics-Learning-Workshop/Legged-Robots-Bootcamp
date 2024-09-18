function [gstop, isterminal,direction]=contact(t,y,ball)

gstop = y(1) - ball.ground; %find the event that ball y-position - ball.ground = 0
direction = -1; %because ball goes from + heigth to 0 to negative height 
                %(other option =1, positive direction and 0, either direction
isterminal = 1;  %1 is stop the integration, (other options is 0 not to step integration)
