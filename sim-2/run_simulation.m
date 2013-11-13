%phases:
% 1: no feet in contact
% 2: rear foot in contact only
% 3: front foot in contact only
% 4: both feet in contact
clear
setpath
p = parameters();
%with settings 11/2, phi = 2*psi
%initial: [x dx y dy    th   dth    a1 da1    a2 da2     phi dphi]
z0 =      [0  0 0  0  pi/3   -1.5  pi/3   0  pi/6   0   -pi/3    0]; 

%decision variables:
% Set these manually? th0, dth0, a10, da10, a20, da20, phi0, dphi0
% tf1, T1: end of time of phase 2 (first phase), torque values this phase
% tf2: end of time of phase 1 (second phase), torque values this phase
% tf3, T3: end of time of phase 3 (third phase), torque values this phase
% tf4: end of time of phase 1 (fourth phase), torque values this phase

% T1 and T3 are vectors of length 3*t_n (3 for number of actuators, t_n is
% number of time steps in this segment.  So if q_ij is the torque of
% actuator i in step j,
% T1 = [q_11, q_21, q_31, q_21, q_22, q_23, q_31, q_32, q_33]

%q2 and q4 are generalized coordinate positions to run pd-control on.
% e.g., q2 = [a1d, a2d, phid]

% set guess
ctrl.tf1 = .75;
ctrl.tf2 = 1.25;
ctrl.tf3 = 1.55;
ctrl.tf4 = 1.6;
tf = 1.6;

%ctrl.T1 = [-1. .2 .25   -1. .02 .125   -1. .02 .125];
ctrl.T1 = [-1. pi/3 .2   -1. pi/6 .2   -.6 pi/6 .18];
ctrl.q2 = [pi/3,pi/6, 2*pi];
%ctrl.T3 = [.2 .2 -1. .2 .2 -1. .2 .2 -1.];
ctrl.T3 = [pi/3 -.8 -1.95   pi/3 -.8 -1.   pi/6 -.2 1.];

ctrl.q4 = [pi/6, pi/3, -pi/3];
x0 = [ctrl.tf1, ctrl.tf2, ctrl.tf3, ctrl.tf4, ctrl.T1, ctrl.T3, ctrl.q2, ctrl.q4, tf];
lb = [      .1,       .1,       .1,       .1, -2.*ones(1,length(ctrl.T1)+length(ctrl.T3)), [0.,0.,-pi], [0.,0.,-pi], .1];
ub = [      2.,       2.,       2.,       2., 2.*ones(1,length(ctrl.T1)+length(ctrl.T3)), [pi/2,pi/2,pi], [pi/2,pi/2,pi], 2.];

x = x0;
%outfunc = @(x,optimValues,state) output_bounding_sim(x,z0,p);
%output_bounding_sim(x0,z0,p);

%%
% % setup and solve nonlinear programming problem
  problem.objective = @(x) objective(x,z0,p);     % create anonymous function that returns objective
  problem.nonlcon = @(x) constraints(x,z0,p);     % create anonymous function that returns nonlinear constraints
  problem.x0 = x0;                                % initial guess for decision variables
  problem.lb = lb;                                % lower bound on decision variables
  problem.ub = ub;                                % upper bound on decision variables
  problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
  problem.Aeq = []; problem.beq = [];             % no linear equality constraints
  problem.options = optimset('Display','iter-detailed');   % set options
  problem.solver = 'fmincon';                     % required for some reason...
  problem.algorithm = 'interior-point';
%  x = fmincon(problem);                           % solve nonlinear programming problem

%% start here to avoid optimization
%x= [0.1046    1.3489    1.2875    1.9349    0.6970   -1.8107   -1.5375 -1.9989   -1.9999    1.9527];
%output_bounding_sim(x)

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
%%
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
