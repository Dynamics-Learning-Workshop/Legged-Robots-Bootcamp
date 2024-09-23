%===================================================================
function [gstop, isterminal,direction]=collision(t,z,walker)
%===================================================================

theta1 = z(1); theta2 = z(3); 

gstop = theta2 + 2*theta1;
if (theta1>-0.05) %allow legs to pass through for small hip angles (taken care in real walker using stepping stones)
    isterminal = 0;
else
    isterminal=1; %ode should terminate is conveyed by 1, if you put 0 it goes till the final time u specify
end
direction=[]; % The t_final can be approached by any direction is indicated by the direction

