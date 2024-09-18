function animation(t_all,z_all,walker)


l = walker.l;

%%% interpolate for animation %%
tstart = t_all(1); 
tend = t_all(end);
tinterp = linspace(tstart,tend,walker.movieFps*(tend-tstart));
[m,n]=size(z_all);
for i=1:n
    zinterp(:,i) = interp1(t_all,z_all(:,i),tinterp);
end

% if (ball.movieWrite)
%     mov = VideoWriter(ball.movieName); 
%     open(mov);
% end

%%%% plotting %%%%%%%

theta1 = zinterp(:,1); %theta1dot  2nd
theta2 = zinterp(:,3); %theta2dot  4th
xh = zinterp(:,5);
yh = zinterp(:,6);

%%%% (xh,yh) H 
%%%%  |\ 
%%%%% | \
%%%%% |  \
%%%%% |   \
%%%%% |    \
%%%%% C1    C2
line([-2 2],[0 0],'linewidth',1,'color','k'); hold on
for i=1:length(tinterp)
    
    x_C1 = xh(i,1) + l*sin(theta1(i,1));
    y_C1 = yh(i,1) - l*cos(theta1(i,1));
    x_C2 = xh(i,1) + l*sin(theta1(i,1)+theta2(i,1));
    y_C2 = yh(i,1) - l*cos(theta1(i,1)+theta2(i,1));
    h0 = plot(xh(i,1),yh(i,1),'ko','MarkerFaceColor','k','Markersize',10);
    h1 = line([xh(i,1) x_C1],[yh(i,1) y_C1],'color','r','Linewidth',2);
    h2 = line([xh(i,1) x_C2],[yh(i,1) y_C2],'color','b','Linewidth',2);
    
    axis([-2 2 -2 2]);
    
    pause(0.01);
    if (i<length(tinterp))
        delete(h0)
        delete(h1)
        delete(h2)
    end
    %plot(0,zinterp(i,1),'ro','MarkerEdgeColor','r', 'MarkerFaceColor','r','MarkerSize',20); hold on; %the ball
    %line([-0.1 0.1],[ball.ground_animate ball.ground_animate],'Linewidth',2); %the ground
    %axis([-0.1 0.1 min(zinterp(:,1))-1 max(zinterp(:,1))+1]); %the axis 
    
%     if (ball.movieWrite)
%         axis off %does not show axis
%         set(gcf,'Color',[1,1,1]) %set background to white
%         writeVideo(mov,getframe);
%     end
%     
%     pause(ball.moviePause);
%     hold off
end
% 
% if (ball.movieWrite)
%     close(mov);
% end
