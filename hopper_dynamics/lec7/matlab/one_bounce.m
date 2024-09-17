function [tt, yy] = one_bounce(t0,ystart,ball)

dt = 5;
options = odeset('Abstol',1e-6,'Reltol',1e-6,'Events',@contact);
                  %stops integration when conditions in contact are met 

%%%%%%%%%% integrate in flight phase %%%                                                       
[t,y]=ode45(@rhs,[t0 t0+dt],ystart,options,ball);

%%%%%%%%%%% apply restitution condition %%%%%
ystart(1,1) = y(end,1);
ystart(1,2) = -ball.e*y(end,2);

tt =t(2:end);
yy = [y(2:(end-1),:); ystart];