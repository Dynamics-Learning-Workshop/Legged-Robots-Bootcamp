%===================================================================
function J=partialder(FUN,z,walker)
%===================================================================
pert=1e-5;
n = length(z);
J = zeros(n,n);

%%%% Using forward difference, accuracy linear %%%
% y0=feval(FUN,z,walker); 
% for i=1:n
%     ztemp=z;
%     ztemp(i)=ztemp(i)+pert; 
%     J(:,i)=(feval(FUN,ztemp,walker)-y0) ;
% end
% J=(J/pert);

%%% Using central difference, accuracy quadratic %%%
for i=1:n
    ztemp1=z; ztemp2=z;
    ztemp1(i)=ztemp1(i)+pert; 
    ztemp2(i)=ztemp2(i)-pert; 
    J(:,i)=(feval(FUN,ztemp1,walker)-feval(FUN,ztemp2,walker)) ;
end
J=J/(2*pert);
