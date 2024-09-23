%===================================================================
function walker_animate(t_all,z_all,walker,fps)
%===================================================================

%%%% Interpolate linearly using fps %%%%%
z_all_plot = [z_all(:,1) z_all(:,3) z_all(:,4) z_all(:,5) z_all(:,6) z_all(:,7) z_all(:,8)];
nn = size(z_all_plot,2);
total_frames = round(t_all(end)*fps);
t = linspace(0,t_all(end),total_frames);
z = zeros(total_frames,nn);
for i=1:nn
    z(:,i) = interp1(t_all,z_all_plot(:,i),t);
end

%%%%% Now animate the results %%%%%%%  
l = walker.l;   
%n = walker.n;

mm = size(z,1);

min_xh = min(z(:,2)); max_xh = max(z(:,2)); 
dist_travelled = max_xh - min_xh;
camera_rate = dist_travelled/mm;

window_xmin = -2*l; window_xmax = 2*l;
window_ymin = -0.1; window_ymax = 2*l;

axis('equal')
axis([window_xmin window_xmax window_ymin window_ymax])
axis off
set(gcf,'Color',[1,1,1])

%%%% create ramp %%%%
rampref=[min_xh-1 max_xh+2*l ; 0 0];

%%%% Draw ramp %%%%%%%%%%
line('xdata',rampref(1,:),'ydata',rampref(2,:),'linewidth', 1,'color','black'); hold on;
 
for i=1:mm
   %moving window %%%%%%%
   window_xmin = window_xmin + camera_rate;
   window_xmax = window_xmax + camera_rate;
   axis('equal')
   axis([window_xmin window_xmax window_ymin window_ymax])

   %%% get angles and hip position 
   theta1 = z(i,1); %phi = walker.phi; 
   xh = z(i,2); yh = z(i,3);
   xfa = z(i,4); yfa = z(i,5);
   xfb = z(i,6); yfb = z(i,7);
   
   %%% coordinates of points of interest %
   hinge=[xh; yh];  
   stance_spoke = [xfa; yfa];
   swing_spoke = [xfb; yfb];
   
   %animate 
   h0 = plot(hinge(1),hinge(2),'ko','MarkerFaceColor','k','Markersize',15);
   h1 = line([hinge(1) stance_spoke(1)],[hinge(2) stance_spoke(2)],'Color','red','Linewidth',2);
   h2 = line([hinge(1) swing_spoke(1)],[hinge(2)  swing_spoke(2)],'Color','blue','Linewidth',2);

   pause(0.01);
   
   if (i~=mm)     %delete all but not the last entry
       delete(h0);
       delete(h1);
       delete(h2);
   end
   
end

