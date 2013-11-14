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
z0 =      [0  0 0  0  pi/4   0    pi/3   0  pi/3   0   -pi/3    0]; 


%decision variables:
% Set these manually? th0, dth0, a10, da10, a20, da20, phi0, dphi0
% tf1: end of time of phase 2 (first interval), 
% tf2: end of time of phase 1 (second interval), 
% tf3: end of time of phase 3 (third interval), 
% tf4: end of time of phase 1 (fourth interval),
%p1,p2,p3,p4,p5: position control points at boundaries of phases
%p12, 

% set guess, none of these times should be equal
ctrl.tf1 = .286;         %take off rear foot
ctrl.tf2 = .356;         %landing front foot
ctrl.tf3 = .47;          %take off front foot
ctrl.tf4 = .5;          %landing rear foot
tf = .5;               %end of sim

            %a1       a2         phi
ctrl.p1 =  [pi/4     pi/4        0.];       %start of simulation
ctrl.p12 = [0        pi/6        pi/2];       %halfway
ctrl.p2 =  [0.       pi/12       pi/2];     %take-off of rear foot
ctrl.p23 = [pi/24    pi/8       pi];     %halfway
ctrl.p3 =  [pi/12    pi/6       pi];     %landing on front foot
ctrl.p34 = [pi/12    pi/8       pi];     %halfway
ctrl.p4 =  [pi/6     pi/8         pi/2.];     %take-off of front foot
ctrl.p45 = [pi/9     pi/12      pi/2.];     %halfway
ctrl.p5 =  [pi/12.   pi/6        0.];        %landing of rear foot

ll = [0., 0., -2.*pi]; %set generalized coordinate limits
ul = [pi/2., pi/2., 2.*pi]; %set generalized coordinate limits

x0 = [ctrl.tf1, ctrl.tf2, ctrl.tf3, ctrl.tf4, tf, ctrl.p1, ctrl.p12,ctrl.p2, ctrl.p23, ctrl.p3, ctrl.p34, ctrl.p4, ctrl.p45, ctrl.p5];
lb = [      .1,       .1,       .1,       .1, .1,     ll,      ll,      ll,      ll, ll,      ll,      ll,      ll,       ll];
ub = [      2.,       2.,       2.,       2., 2.,     ul,      ul,      ul,      ul, ul,      ul,      ul,      ul,       ul];

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
tf = x(5);

ctrl.p1 =  x(6:8);
ctrl.p12 = x(9:11);
ctrl.p2 =  x(12:14);
ctrl.p23 = x(15:17);
ctrl.p3 =  x(18:20);
ctrl.p34 = x(21:23);
ctrl.p4 =  x(24:26);
ctrl.p45 = x(27:29);
ctrl.p5 =  x(30:32);

tspan = [0 tf];                                 % set time span
[t z u indices] = hybrid_simulation(z0,ctrl,p,tspan); % run simulation
%% start here to just replay
% Run the animation
R = z2R_bounding(z,p);                   %get the coordinates of the points to animate
num_sps = 8;                            %ugg, for now you have to enter this.  eventually grab from size(R)
C = [1 2; 2 5; 3 4; 4 4+num_sps];
for i=1:num_sps-1
    C = [C; 4+i 5+i]; %chain together spine
   
end
for i=2:num_sps-1
    C = [C; 4+i 4+num_sps+2*i-1; 4+i 4+num_sps+2*i]; %make vertebra
end

speed = .125;                                 % set animation speed
cla                                         % clear axes

animate_simple(t,R,C,speed)                 % run animation
%animate_simple(t,R,C,speed,'save')                 % run animation

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
