%===================================================================
function animate(t_all,z_all,walker,steps,fps)
%===================================================================

%%%% Interpolate linearly using fps %%%%%
z_all_plot = [z_all(:,1) z_all(:,3) z_all(:,5) z_all(:,6)];
nn = size(z_all_plot,2);
total_frames = round(t_all(end)*fps);
t = linspace(0,t_all(end),total_frames);
z = zeros(total_frames,nn);
for i=1:nn
    z(:,i) = interp1(t_all,z_all_plot(:,i),t);
end

%%%%% Now animate the results %%%%%%%  
l = walker.l;    
c = walker.c;

mm = size(z,1);

min_xh = min(z(:,3)); max_xh = max(z(:,3)); 
dist_travelled = max_xh - min_xh;
camera_rate = dist_travelled/mm;

window_xmin = -1*l; window_xmax = 1*l;
window_ymin = -0.1; window_ymax = 1.1*l;

axis('equal')
axis([window_xmin window_xmax window_ymin window_ymax])
axis off
set(gcf,'Color',[1,1,1])

%%%% create ramp %%%%
rampref=[min_xh-1 max_xh+l ; 0 0];

%%%% Draw ramp %%%%%%%%%%
line('xdata',rampref(1,:),'ydata',rampref(2,:),'linewidth', 1,'color','black'); hold on;
 
for i=1:mm
   %moving window %%%%%%%
   window_xmin = window_xmin + camera_rate;
   window_xmax = window_xmax + camera_rate;
   axis('equal')
   axis([window_xmin window_xmax window_ymin window_ymax])

   %%% get angles and hip position 
   theta1 = z(i,1); theta2 = z(i,2); 
   xh = z(i,3); yh = z(i,4);
   
   %%% coordinates of points of interest %
   hinge=[xh; yh];  
   stance_foot = [xh+l*sin(theta1);         yh-l*cos(theta1)];
   stance_leg_com = [xh+c*sin(theta1);         yh-c*cos(theta1)];
   swing_foot =  [xh+l*sin(theta1 + theta2),yh-l*cos(theta1 + theta2)];
   swing_leg_com =  [xh+c*sin(theta1 + theta2),yh-c*cos(theta1 + theta2)];
   
   %animate 
   h0 = plot(hinge(1),hinge(2),'ko','MarkerFaceColor','k','Markersize',10);
   h1 = plot(stance_leg_com(1),stance_leg_com(2),'ko','MarkerFaceColor','k','Markersize',15);
   h2 = plot(swing_leg_com(1),swing_leg_com(2),'ko','MarkerFaceColor','k','Markersize',15);
   h3 = line([hinge(1) stance_foot(1)],[hinge(2) stance_foot(2)],'Color','red','Linewidth',2);
   h4 = line([hinge(1) swing_foot(1)], [hinge(2) swing_foot(2)],'Color','red','Linewidth',2);
   
   %delay to create animation
   pause(0.01);
   
   if (i~=mm)     %delete all but not the last entry
       delete(h0);
       delete(h1);
       delete(h2);
       delete(h3);
       delete(h4);
   end
   
end

