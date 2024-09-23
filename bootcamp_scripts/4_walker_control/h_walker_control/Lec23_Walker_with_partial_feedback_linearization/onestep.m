%===================================================================
function [z,t,z_ref]=onestep(z0,walker,steps)
%===================================================================

l = walker.l;

flag = 1;
if nargin<2
    error('need more inputs to onestep');
elseif nargin<3
    flag = 0; %send only last state, for root finder and jacobian
    steps = 1;
end

theta1 = z0(1);
xh = 0;
yh = l*cos(theta1);
xh_start = xh;

t0 = 0; 
dt = 4; %might need to be changed based on estimate of time taken for one step
time_stamps = 100;
t_ode = t0;
z_ode = [z0 xh yh];
z_ref = [z0(3) z0(4) 0]; %theta2 theta2dot theta2ddot for reference
for i=1:steps
    
    if (walker.control.on==1)
        walker.control.t0 = t0;
        walker.control.theta20 = z0(3);
        walker.control.theta20dot = z0(4);
    end
    
    options=odeset('abstol',1e-13,'reltol',1e-13,'events',@collision);
    tspan = linspace(t0,t0+dt,time_stamps);
    [t_temp, z_temp] = ode113(@single_stance,tspan,z0,options,walker);
    
    if (walker.control.on==1)
        for j=1:length(t_temp)
            [u,theta2_ref(j),theta2dot_ref(j),theta2ddot_ref(j)] = controller(t_temp(j),z_temp(j,1:4),walker);
        end
    end
    
    if (flag==1) %means forward simulation
       disp(' **** ');
       disp(['step no: ', num2str(i)]);
       disp(['step time: ', num2str(t_temp(end)-t_temp(1))]);
       disp(['stance speed single stance take-off: ',num2str(z0(2))]);
       disp(['hip angle at touchdown: ',num2str(z_temp(end,3))]);
       disp(['hip speed at touchdown: ',num2str(z_temp(end,4))]);
       disp(' ');
    end
    zplus=footstrike(t_temp(end),z_temp(end,:),walker);    
    
    z0 = zplus;
    t0 = t_temp(end);
    
    xh_temp = xh_start + l*sin(z_temp(1,1))-l*sin(z_temp(:,1)); 
    yh_temp =  l*cos(z_temp(:,1));
    
    t_ode = [t_ode; t_temp(2:end)];
    z_ode = [z_ode; [z_temp(2:(end-1),:); zplus] xh_temp(2:end) yh_temp(2:end)];
    
     if (walker.control.on==1)
        z_ref = [z_ref; theta2_ref(2:end)',theta2dot_ref(2:end)',theta2ddot_ref(2:end)'];
        theta2_ref = []; theta2dot_ref = []; theta2ddot_ref = [];
     end
    
    xh_start = xh_temp(end);
end

z = zplus(1:4);

if flag==1
   z=z_ode;
   t=t_ode;
end
