function A = DH(a,alpha,d,theta)

cth = cos(theta);
sth = sin(theta);
cal = cos(alpha);
sal = sin(alpha); 

Rot_z_theta = [cth -sth 0 0; 
               sth  cth 0 0;
                0     0 1 0;
                0     0 0 1];
            
Trans_z_d = [1 0 0 0;
             0 1 0 0;
             0 0 1 d;
             0 0 0 1];
       
Trans_x_a = [1 0 0 a;
                 0 1 0 0;
                 0 0 1 0;
                 0 0 0 1];
          
Rot_x_alpha = [1   0   0   0;
               0  cal -sal 0;
               0  sal  cal 0;
               0   0    0  1];
           
A = Rot_z_theta*Trans_z_d*Trans_x_a*Rot_x_alpha;


