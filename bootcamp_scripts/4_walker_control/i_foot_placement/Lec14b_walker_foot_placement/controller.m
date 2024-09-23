function phi = controller(thetadot,thetadot_des)


%%% put your controller here %%
%%% phi = f(thetadot,thetadot_des) 
%%% thetadot is measured (current) mid-stance velocity
%%% thetadot_des is desired mid-stance velocity at the next step

phi = 0.5+0.05*(thetadot-thetadot_des); %example of a linear controller
%phi = pi/6;  %example of an open-loop controller