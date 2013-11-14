function stop = output_bounding_sim(x,z0,p)
disp(x);
ctrl.tf1 = x(1);
ctrl.tf2 = x(2);
ctrl.tf3 = x(3);
ctrl.tf4 = x(4);

ctrl.T1 = x(5:13);
ctrl.T3 = x(14:22);

ctrl.q2 = x(23:25);
ctrl.q4 = x(26:28);
tf = x(29);

tspan = [0 tf];                                 % set time span
[t z u indices] = hybrid_simulation(z0,ctrl,p,tspan); % run simulation

% Run the animation
R = z2R_bounding(z,p);                   %get the coordinates of the points to animate
num_sps = 8;                        %ugg, for now you have to enter this.  eventually grab from size(R)
C = [1 2; 2 5; 3 4; 4 4+num_sps];
for i=1:num_sps-1
    C = [C; 4+i 5+i]; %chain together spine
end
for i=2:num_sps-1
    C = [C; 4+i 4+num_sps+2*i-1; 4+i 4+num_sps+2*i]; %make vertebra
end
%%
speed = .25;                                 % set animation speed
cla                                         % clear axes

animate_simple(t,R,C,speed)                 % run animation
end
