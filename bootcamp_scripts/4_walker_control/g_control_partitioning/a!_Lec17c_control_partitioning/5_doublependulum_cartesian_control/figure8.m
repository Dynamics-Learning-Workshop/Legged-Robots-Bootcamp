function [x,y,xdot,ydot,xddot,yddot] = figure8(x0,y0,t)

if (t(1)~=0)
    error('t(1) should be zero');
end

T = t(end);
A = 0.5; B = A;
a = 2; b = 1;
tau = 2*pi*(-15*(t/T).^4+6*(t/T).^5+10*(t/T).^3);
taudot = 2*pi*(-15*4*(1/T)*(t/T).^3+6*5*(1/T)*(t/T).^4+10*3*(1/T)*(t/T).^2);
tauddot = 2*pi*(-15*4*3*(1/T)^2*(t/T).^2 + 6*5*4*(1/T)^2*(t/T).^3+10*3*2*(1/T)^2*(t/T));

x = x0+A*sin(a*tau); 
y = y0+B*cos(b*tau);
xdot =  A*a*cos(a*tau).*taudot;  
ydot = -B*b*sin(b*tau).*taudot;
xddot = -A*a*a*sin(a*tau).*taudot+A*a*cos(a*tau).*tauddot;
yddot = -B*b*b*sin(b*tau).*taudot-B*b*sin(b*tau).*tauddot;


% figure(1)
% plot(x,y,'LineWidth',2)
% axis('equal');
% xlabel('x');
% ylabel('y');
% 
% figure(2)
% subplot(2,1,1);
% plot(t,x);
% ylabel('x');
% subplot(2,1,2);
% plot(t,y);
% ylabel('y');
% xlabel('t');
% 
% 
% figure(3)
% subplot(2,1,1);
% plot(t,xdot);
% ylabel('xdot');
% subplot(2,1,2);
% plot(t,ydot);
% ylabel('ydot');
% xlabel('t');
% 
% figure(4)
% subplot(2,1,1);
% plot(t,xddot);
% ylabel('xddot');
% subplot(2,1,2);
% plot(t,yddot);
% ylabel('yddot');
% xlabel('t');

