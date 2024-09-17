function animation(t_all,y_all,ball)

%%% interpolate for animation %%
tstart = t_all(1); 
tend = t_all(end);
tinterp = linspace(tstart,tend,ball.movieFps*(tend-tstart));
[m,n]=size(y_all);
for i=1:n
    yinterp(:,i) = interp1(t_all,y_all(:,i),tinterp);
end

if (ball.movieWrite)
    mov = VideoWriter(ball.movieName); 
    open(mov);
end

for i=1:length(tinterp)
    plot(0,yinterp(i,1),'ro','MarkerEdgeColor','r', 'MarkerFaceColor','r','MarkerSize',20); hold on; %the ball
    line([-0.1 0.1],[ball.ground_animate ball.ground_animate],'Linewidth',2); %the ground
    axis([-0.1 0.1 min(yinterp(:,1))-1 max(yinterp(:,1))+1]); %the axis 
    
    if (ball.movieWrite)
        axis off %does not show axis
        set(gcf,'Color',[1,1,1]) %set background to white
        writeVideo(mov,getframe);
    end
    
    pause(ball.moviePause);
    hold off
end

if (ball.movieWrite)
    close(mov);
end
