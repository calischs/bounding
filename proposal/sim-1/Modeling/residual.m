function r = residual(x)
%x is [end time of sim, mass of leg segment]
%r is [difference between com height at end of sim and target height, vertical com speed at end of sim]
target  = 1.;

p = parameters();                       % get parameters from file
p(4) = x(2);                            %set leg mass from function arguments
p(5) = x(2);                            %set leg mass from function arguments
z0 = [0; 0; pi/6; 0];                       % set initial state
tspan = [0 x(1)];                              % set time span
[t z u indices] = hybrid_simulation(z0,p,tspan);
com = COM_all(z,p);
r = [com(2,end)-target, com(4,end)];