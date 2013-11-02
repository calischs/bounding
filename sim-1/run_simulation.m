%phases:
% 1: no feet in contact
% 2: rear foot in contact only
% 3: front foot in contact only
% 4: both feet in contact
clear
setpath
p = parameters();
%with settings 11/2, phi = 2*psi
%initial: [x dx y dy th dth a1 da1 a2 da2 phi dphi]
z0 =      [0  0 0  0  pi/3   0  pi/6   0  pi/6   0   -pi    0]; 
iphase0 = 4; %rear foot in contact

tspan = [0 .5];                                 % set time span
[t z u indices] = hybrid_simulation(z0,p,tspan,iphase0); % run simulation

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
speed = .25;                                 % set animation speed
cla                                         % clear axes
animate_simple(t,R,C,speed)                 % run animation

%e = energy_all(z,p);
%com = COM_all(z,p);
%th = z(3,:);
%dth = z(4,:);
%work = cumtrapz(t,u.*dth);
%diff = e-work;
%disp(sprintf('Energy Difference: %.3f J',diff(end)-diff(1)));
%plot energy
%cla                                 % clear axes
%plot(t,[e;work;diff],'marker','.') % plot position trajectories
%xlabel('time (s)')
%ylabel('energy (J)')
%legend('Energy', 'Control Work', 'Difference')

%cla                                 % clear axes
%plot com trajectory
%plot(t,com(2,:),'marker','.');
%xlabel('time (s)');
%ylabel('y (m)');
%grid on;
%legend('COM trajectory','Location','Southeast');
