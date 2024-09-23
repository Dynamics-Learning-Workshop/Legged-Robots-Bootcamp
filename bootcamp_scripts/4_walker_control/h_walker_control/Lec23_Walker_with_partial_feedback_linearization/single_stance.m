%===================================================================
function zdot=single_stance(t,z,walker)  
%===================================================================
theta1 = z(1);   omega1 = z(2);                         
theta2 = z(3);   omega2 = z(4);                         
                    
M = walker.M;  m = walker.m; I = walker.I;   
l = walker.l;  c = walker.c; 
g = walker.g; gam = walker.gam;

if (walker.control.on==1)
    u = controller(t,z,walker);   
else
    u = 0;
end

M11 = 2*I + M*l^2 + 2*c^2*m + 2*l^2*m - 2*c*l*m - 2*c*l*m*cos(theta2);
M12 = I + c^2*m - c*l*m*cos(theta2);
M21 = I + c^2*m - c*l*m*cos(theta2);
M22 = I + c^2*m;
Ms = [M11 M12; M21 M22];
 
N1 = c*g*m*sin(theta1 - gam + theta2) + M*g*l*sin(gam - theta1) - c*g*m*sin(gam - theta1) + 2*g*l*m*sin(gam - theta1) + c*l*m*omega2^2*sin(theta2) + 2*c*l*m*omega1*omega2*sin(theta2);
N2 = c*m*(g*sin(theta1 - gam + theta2) - l*omega1^2*sin(theta2));
Ns = [N1; N2];

B = [0; 1];
alpha = Ms\(-Ns+B*u);

zdot = [omega1 alpha(1) omega2 alpha(2)]';  

