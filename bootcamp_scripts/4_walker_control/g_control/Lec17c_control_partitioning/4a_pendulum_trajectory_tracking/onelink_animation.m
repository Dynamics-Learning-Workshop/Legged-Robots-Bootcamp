function onelink_animation(t,theta,parms)

tspan = linspace(t(1),t(end),parms.framespersec*(t(end)-t(1)));
theta = interp1(t,theta,tspan); %down-sample theta to enable animation
l1 = parms.l1;

for i=1:length(theta) %plot in a loop.
    
    clf %clear figure axis. This ensures the old plot is erased completely.
    
    %%% Specify parameters and get transformation matrices
    x = l1*cos(theta(i));
    y = l1*sin(theta(i));
    
    %%% Get locations of joints
    %Location of joint 1
    endOfLink1 = [x y];

    %Plot the hinge point at 0,0,0.
    plot(0,0,'o','MarkerSize',10,'MarkerFaceColor','black');

    %%% Draw lines from one joint to another 
    %Draw line from origin to end of link 1
    line([0 endOfLink1(1)],[0 endOfLink1(2)],'LineWidth',5,'Color','red');
      
    axis('equal');
    
    % These set the x,y,z limits for the axis (will need adjustment)
    xlim([-2 2]); 
    ylim([-2 2]);
    zlim([-2 2]);
    view(2) %for a 2-d animation
    %view(3) %for a 3-d animaton
    %view([-12,20]); %gives you more control over a 3d animation (type help view for more info)
    
    pause(parms.time_delay); %pause for sometime (will need adjustment depending on the problem)
end