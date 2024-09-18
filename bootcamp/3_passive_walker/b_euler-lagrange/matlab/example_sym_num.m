function example_sym_num

x = 1;
dx = 1e-3;
dfdx_fwd = (fn(x+dx)-fn(x))/dx
dfdx_cen = (fn(x+dx)-fn(x-dx))/(2*dx)




function f = fn(x)
f=x^2+2*x+2;