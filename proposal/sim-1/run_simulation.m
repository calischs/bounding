clear

% We can organize our code by filing things in different folders.  These
% folders need to be added to the Matlab path so that it can run the files
% inside them even when they are not the current folder listed at the top
% of the Matlab window.  For more information about the current folder, see
% http://www.mathworks.com/help/matlab/matlab_env/understanding-file-locations-in-matlab.html
% For more information about the Matlab path, see
% http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html
setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path

p = parameters();                           % get parameters from file
z0 = [0; 0; pi/6; 0];                       % set initial state
tspan = [0 2.];                              % set time span

x0 = [.45, .02];
f = @(x) residual(x); 
x = fsolve(f,x0);
disp(x);

tspan(2) = x(1);
p(4) = x(2);
p(5) = x(2);
%now run the simulation with the calculated values

[t z u indices] = hybrid_simulation(z0,p,tspan); % run simulation

% Run the animation
R = z2R_all(z,p);                           % get the coordinates of the points to animate
C = [1 2; 2 3];                             % connect points 1 to 2, 2 to 3 with lines
speed = .25;                                 % set animation speed
cla                                         % clear axes
animate_simple(t,R,C,speed)                 % run animation

e = energy_all(z,p);
com = COM_all(z,p);
th = z(3,:);
dth = z(4,:);
work = cumtrapz(t,u.*dth);
diff = e-work;
disp(sprintf('Energy Difference: %.3f J',diff(end)-diff(1)));
%plot energy
cla                                 % clear axes
plot(t,[e;work;diff],'marker','.') % plot position trajectories
xlabel('time (s)')
ylabel('energy (J)')
legend('Energy', 'Control Work', 'Difference')

%cla                                 % clear axes
%plot com trajectory
%plot(t,com(2,:),'marker','.');
%xlabel('time (s)');
%ylabel('y (m)');
%grid on;
%legend('COM trajectory','Location','Southeast');
