function twolink_animation(t,x,parms)

%downsample data for animation
tspan = linspace(t(1),t(end),parms.framespersec*(t(end)-t(1)));
x = interp1(t,x,tspan); %down-sample x to enable animation
l1 = parms.l1;
l2 = parms.l2;

for i=1:length(x) %plot in a loop.
    
    %clf %clear figure axis. This ensures the old plot is erased completely.
   
    theta1 = x(i,1);
    theta2 = x(i,2);
    x1 = l1*cos(theta1); 
    y1 = l1*sin(theta1);
    x2 = x1 + l2*cos(theta1+theta2);
    y2 = y1 + l2*sin(theta1+theta2);
    
    
    %%% Get locations of joints
    %Location of joint 1
    endOfLink1 = [x1 y1];
    
    %Location of joint 2
    endOfLink2 = [x2 y2];

    %Plot the hinge point at 0,0,0.
    h1 = plot(0,0,'o','MarkerSize',10,'MarkerFaceColor','black'); hold on;
    h2 = plot(endOfLink2(1),endOfLink2(2),'o','MarkerSize',3,'MarkerFaceColor','black');
    
    %%% Draw lines from one joint to another 
    %Draw line from origin to end of link 1
    h3 = line([0 endOfLink1(1)],[0 endOfLink1(2)], 'LineWidth',5,'Color','red');
      
    %Draw line from end of link 1 to end of link 2
    h4 =  line([endOfLink1(1) endOfLink2(1)],...
     [endOfLink1(2) endOfLink2(2)],...
     'LineWidth',5,'Color',[0 0.6 0.3]); %[0 0.6 0.3] are rgb values as fractions --> [red green blue]
      
 
    
    axis('equal');
    
    % These set the x,y,z limits for the axis (will need adjustment)
    xlim([-2 2]); 
    ylim([-2 2]);
    zlim([-2 2]);
    view(2) %for a 2-d animation
    %view(3) %for a 3-d animaton
    %view([-12,20]); %gives you more control over a 3d animation (type help view for more info)
    
    pause(parms.time_delay); %pause for sometime (will need adjustment depending on the problem)
    delete(h1);
    delete(h3);
    delete(h4);
end