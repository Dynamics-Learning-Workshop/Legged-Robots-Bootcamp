function [z, t] = onestep(t0,z0,walker)

%z0
dt = 8;
options = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@contact);
                  %stops integration when conditions in contact are met 

%%%%%%%%%% integrate in flight phase %%%                                                       
[t,z]=ode45(@rhs,[t0 t0+dt],z0,options,walker);

zminus = z(end,:);
zplus = foot_strike(zminus,walker);

if nargout==1
    z = zplus;
end

%zplus %should be same as z0

% %%%%%%%%%%% apply restitution condition %%%%%
% ystart(1,1) = y(end,1);
% ystart(1,2) = -ball.e*y(end,2);

% tt =t(2:end);
%yy = [y(2:(end-1),:); ystart];