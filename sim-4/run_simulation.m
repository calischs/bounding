%clear
setpath
p = parameters();
history.x = [];
history.fval = [];

%%
%we're going to mimic the trajectories we've been creating for the physical
%robot here.  To parameterize, we have:
% T: the period of the stride.
% ramp: portion of the period spent ramping between values.
% stand: amplitude of leg standing
% bent: amplitude of leg bent
% spine_amp: amplitude of spine
%can include spine_zero and spine_shift later.
ctrl.T = .4;
ctrl.ramp = .08;
ctrl.stand = (90-70)* pi/180.;
ctrl.bent =  (90-50) * pi/180.;
ctrl.spine_amp = -25 * pi/180.;
ctrl.spine_shift = .25;

x0 = [ctrl.T];
lb = [.1];
ub = [2.];

%x0 = [ctrl.T, ctrl.ramp, ctrl.stand, ctrl.bent, ctrl.spine_amp];
%lb = [     0.,      0.,          0.,        0.,             0.];
%ub = [      1.,     .2,        pi/2,      pi/2,           pi/2];

[x,history] = runopt(x0,ctrl,lb,ub,p,history);

%% start here to avoid optimization
%x = x0;
%x = history.x(2,:);

ctrl.T = x(1);
%ctrl.ramp = x(2);
%ctrl.stand = x(3);
%ctrl.bent = x(4);
%ctrl.spine_amp = x(5);

tspan = [0 2*ctrl.T];                                 % set time span
[t z u indices iphases] = hybrid_simulation(ctrl,p,tspan); % run simulation
%% start here to just replay
% Run the animation
R = z2R_bounding(z,p);                   %get the coordinates of the points to animate
num_sps = 4;                            %ugg, for now you have to enter this.  eventually grab from size(R)
C = [1 2; 2 5; 3 4; 4 4+num_sps];
for i=1:num_sps-1
    C = [C; 4+i 5+i]; %chain together spine
   
end
for i=2:num_sps
    C = [C; 4+i 4+num_sps+2*i-1; 4+i 4+num_sps+2*i]; %make vertebra
end

for i=1:num_sps-1
   C = [C; 4+i 4+num_sps+2*(i+1)-1; 4+i 4+num_sps+2*(i+1)];
end

speed = .125;                                 % set animation speed
cla                                         % clear axes

%animate_simple(t,R,C,speed)                 % run animation
animate_simple(t,R,C,speed,'save')                 % run animation

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
