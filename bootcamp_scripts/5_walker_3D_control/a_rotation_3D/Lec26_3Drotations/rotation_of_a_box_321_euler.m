function rotation_of_a_box_321_euler
%%% how to use patch with face and vertex %%
clear all
close all
clc

%%%%%%%%% a single angle %%%%%%
% phi = deg2rad(90); 
% theta = deg2rad(90);
% psi = deg2rad(90);
% angles = [phi, theta, psi];
% figure(1)
% animate(angles);

%%%%%%%% rotate by 90 degrees individually %%%
figure(1)
phi = deg2rad(0); 
theta = deg2rad(0);
psi = deg2rad(0);
angles = [phi, theta, psi];
subplot(2,2,1); animate(angles);

phi = deg2rad(0); 
theta = deg2rad(0);
psi = deg2rad(90);
angles = [phi, theta, psi];
subplot(2,2,2); animate(angles);

phi = deg2rad(0); 
theta = deg2rad(90);
psi = deg2rad(90);
angles = [phi, theta, psi];
subplot(2,2,3); animate(angles);

phi = deg2rad(90); 
theta = deg2rad(90);
psi = deg2rad(90);
angles = [phi, theta, psi];
subplot(2,2,4); animate(angles);

% %%%%%% animation %%%%%%
% phi = linspace(0,pi/2,100);
% theta = linspace(0,pi,100);
% psi = linspace(0,pi/4,100);
% angles = [phi', theta', psi'];
% figure(2)
% animate(angles);


function animate(angles)

lx = 0.5;  %along x
ly = 0.25; %along y 
lz = 0.1; %along z
ll = 1; %for lines along x-, y- and z-axis
dmax = max([lx ly lz ll]);


line_x = [0 0 0; 
         ll 0 0];
line_y = [0 0 0; 
          0 ll 0];
line_z = [0 0 0; 
          0 0 ll];

vertex(1,:) = [ lx  ly -lz];
vertex(2,:) = [ lx -ly -lz];
vertex(3,:) = [ lx -ly  lz];
vertex(4,:) = [ lx  ly  lz];
vertex(5,:) = [-lx  ly -lz];
vertex(6,:) = [-lx -ly -lz];
vertex(7,:) = [-lx -ly  lz];
vertex(8,:) = [-lx  ly  lz];

face = [ 1 2 3 4 ;
         5 6 7 8 ;
         2 6 7 3 ;
         1 5 8 4 ;
         3 7 8 4 ;
         2 6 5 1 ];


[m,n] = size(vertex);
[mm,nn] = size(angles);
[p,q] = size(line_x);

for ii=1:mm
    phi = angles(ii,1); 
    theta = angles(ii,2);
    psi = angles(ii,3);
    R = get_rotation(phi,theta,psi);
    
    for i=1:m
        r_body = vertex(i,:)';
        r_world = R*r_body;
        new_vertex(i,:) = r_world';
    end
      

    for i=1:p
        line_x_body = line_x(i,:)';
        line_x_world = R*line_x_body;
        new_line_x(i,:) = line_x_world';
        
        line_y_body = line_y(i,:)';
        line_y_world = R*line_y_body;
        new_line_y(i,:) = line_y_world';
        
        line_z_body = line_z(i,:)';
        line_z_world = R*line_z_body;
        new_line_z(i,:) = line_z_world';
    end
        
   
    h1 = patch('Vertices',new_vertex,'Faces',face,'FaceColor','green'); hold on
    
    %custom written script to animate lines with arrows
    draw_line3(new_line_x(1,:),new_line_x(2,:),'LineWidth',0.02,'ArrowLength',0.1,'ArrowDirection',1,'ArrowIntend',0.01,'ArrowColor','r','ArrowAngle',20);
    draw_line3(new_line_y(1,:),new_line_y(2,:),'LineWidth',0.02,'ArrowLength',0.1,'ArrowDirection',1,'ArrowIntend',0.01,'ArrowColor','r','ArrowAngle',20);
    draw_line3(new_line_z(1,:),new_line_z(2,:),'LineWidth',0.02,'ArrowLength',0.1,'ArrowDirection',1,'ArrowIntend',0.01,'ArrowColor','r','ArrowAngle',20);
    title(['phi (x) = ', num2str(rad2deg(phi)),'; theta (y) = ', num2str(rad2deg(theta)),'; psi (z) = ', num2str(rad2deg(psi))],'Fontsize',14);

    %matlab default lines without arrows
    %h2 = line(new_line_x(:,1)', new_line_x(:,2)', new_line_x(:,3)','Linewidth',4); 
    %h3 = line(new_line_y(:,1)', new_line_y(:,2)', new_line_y(:,3)','Linewidth',4); 
    %h4 = line(new_line_z(:,1)', new_line_z(:,2)', new_line_z(:,3)','Linewidth',4); 
    axis([-dmax dmax -dmax dmax -dmax dmax]);
    xlabel('x'); ylabel('y'); zlabel('z');
    view(3)
    pause(0.01)
    if (ii~=mm)
         clf %clear everthing on the figure window
    end
end

function R = get_rotation(phi,theta,psi)

%%%%%%% uses 3-2-1 euler angles
R_x = [1    0       0; ...
       0  cos(phi) -sin(phi); ...
       0  sin(phi) cos(phi)];
   
R_y = [cos(theta)  0   sin(theta); ...
       0           1         0; ...
      -sin(theta) 0   cos(theta)]; 
   
R_z = [cos(psi) -sin(psi)  0; ...
       sin(psi)  cos(psi)  0; ...
       0           0       1];  
R = R_z*R_y*R_x;