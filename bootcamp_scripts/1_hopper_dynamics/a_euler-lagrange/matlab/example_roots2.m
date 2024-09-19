function example_roots2
clc %clear screen
close all %closes figures 

%f = x^2 - x - 2; 
%ax^2+bx+c=0
a = 1; b = -2; c = -2; d=0.5;
parms.a=a;parms.b=b; parms.c=c;parms.d=d;
x = linspace(-6,6);
for i=1:length(x)
    f(i) = fn(x(i),parms);
end
plot(x,f,'Linewidth',2); %plot the function to get idea of the roots
title('f(x) = a*x^2 +b*x +c');
xlabel('x','fontsize',12);
ylabel('f(x)','fontsize',12);
grid on;

%%% solving the equation %%
x0 = -2; %Try different initial guesses, e.g., 3, -3, 0
options = optimoptions('fsolve','Display','iter','MaxIter',100,'MaxFunEval',300);
[x,fval,exitflag] = fsolve(@fn,x0,options,parms);

%options = optimset('Display','iter');
%X = fzero(@fn,x0,options) 
fval
exitflag

function f = fn(x,parms)
a = parms.a; b = parms.b; c = parms.c; d=parms.d; 
%f = x^2 - x - 2;
f = a*x^2 + b*x + c + d*sin(x);

