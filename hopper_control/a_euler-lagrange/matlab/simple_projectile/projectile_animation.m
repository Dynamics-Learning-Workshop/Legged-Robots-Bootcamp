function projectile_animation(t_all,q_all,projectile)

%%% interpolate for animation %%
tstart = t_all(1); 
tend = t_all(end);
tinterp = linspace(tstart,tend,projectile.movieFps*(tend-tstart));
[m,n]=size(q_all);
for i=1:n
    qinterp(:,i) = interp1(t_all,q_all(:,i),tinterp);
end

if (projectile.movieWrite)
    mov = VideoWriter(projectile.movieName); 
    open(mov);
end

for i=1:length(tinterp)
    plot(qinterp(i,1),qinterp(i,3),'ro','MarkerEdgeColor','r', 'MarkerFaceColor','r','MarkerSize',10); hold on
    axis([-1 max(qinterp(:,1))+1 min(qinterp(:,3))-1 max(qinterp(:,3))+1]); %axis([xmin xmax ymin ymax]); 
    
    plot(qinterp(1:i,1),qinterp(1:i,3),'k');
    grid on;
    
    if (projectile.movieWrite)
        axis off %does not show axis
        set(gcf,'Color',[1,1,1]) %set background to white
        writeVideo(mov,getframe);
    end
    
    pause(projectile.moviePause);
    hold off;
end

if (projectile.movieWrite)
    close(mov);
end
