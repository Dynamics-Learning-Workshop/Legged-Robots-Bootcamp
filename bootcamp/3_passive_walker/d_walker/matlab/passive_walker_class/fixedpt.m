function F = fixedpt(z,walker)

t0 = 0;
zplus = onestep(t0,z,walker);
F = z-zplus; %trying to set F = 0 
