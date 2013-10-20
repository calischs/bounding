function derive_everything() 
name = 'all';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t y dy ddy th dth ddth tau Fy l c1 c2 m1 m2 mh I1 I2 g real

% Group them for later use.
q   = [y; th];      % generalized coordinates
dq  = [dy; dth];    % first time derivatives
ddq = [ddy; ddth];  % second time derivatives
u   = tau;          % control forces and moments
c   = Fy;           % constraint forces and moments
p   = [l; c1; c2; m1; m2; mh; I1; I2; g];  % parameters

%%% Calculate important vectors and their time derivatives.

% Define fundamental unit vectors.  The first element should be the
% horizontal (+x cartesian) component, the second should be the vertical (+y
% cartesian) component, and the third should be right-handed orthogonal.
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);

% Define other unit vectors for use in defining other vectors.
er1hat =  cos(th)*ihat + sin(th) * jhat;
er2hat = -cos(th)*ihat + sin(th) * jhat;

% A handy anonymous function for taking first and second time derivatives
% of vectors using the chain rule.  See Lecture 6 for more information. 
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% Define vectors to key points.
rf = y*jhat;
rcm1 = rf+c1*er1hat;
rk = rf+l*er1hat;
rcm2 = rk + c2*er2hat;
rh = rk + l*er2hat;

% Take time derivatives of vectors as required for kinetic energy terms.
drcm1 = ddt(rcm1);
drcm2 = ddt(rcm2);
drh   = ddt(rh);

%%% Calculate Kinetic Energy, Potential Energy, and Generalized Forces

% F2Q calculates the contribution of a force to all generalized forces
% for forces, F is the force vector and r is the position vector of the 
% point of force application
F2Q = @(F,r) simple(jacobian(r,q)'*(F)); 

% M2Q calculates the contribution of a moment to all generalized forces
% M is the moment vector and w is the angular velocity vector of the
% body on which the moment acts
M2Q = @(M,w) simple(jacobian(w,dq)'*(M)); 

% Define kinetic energies. See Lecture 6 formula for kinetic energy
% of a rigid body.
T1 = (1/2)*m1*dot(drcm1, drcm1) + (1/2)* I1 * dth^2;
T2 = (1/2)*m2*dot(drcm2, drcm2) + (1/2)* I2 * (-dth)^2;
Th = (1/2)*mh*dot(drh, drh);

% Define potential energies. See Lecture 6 formulas for gravitational 
% potential energy of rigid bodies and elastic potential energies of
% energy storage elements.
V1 = m1*g*dot(rcm1, jhat);
V2 = m2*g*dot(rcm2, jhat);
Vh = mh*g*dot(rh, jhat);

% Define contributions to generalized forces.  See Lecture 6 formulas for
% contributions to generalized forces.
QF = F2Q(Fy*jhat,rf);
Qtau = M2Q(-tau*khat, -dth*khat);

% Sum kinetic energy terms, potential energy terms, and generalized force
% contributions.
T = T1 + T2 + Th;
V = V1 + V2 + Vh;
Q = QF + Qtau;

% Assemble R, the array of cartesian coordinates of the points to be
% animated.  Point numbers correspond with the order in which they appear
% in the array.
R = [rf(1:2); rk(1:2); rh(1:2)]; % Point 1: foot. Point 2: knee. Point 3: hip.

% Calculate rcm, the location of the center of mass
rcm = (m1*rcm1 + m2*rcm2 + mh*rh)/(m1+m2+mh);

% Assemble C, the set of constraints
C = y;  % When y = 0, the constraint is satisfied because foot is on the ground

%% All the work is done!  Just turn the crank...
%%% Derive Energy Function and Equations of Motion
E = T+V;                                         % total system energy
L = T-V;                                         % the Lagrangian
gd = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;  % form the dynamics equations
gc = ddt(ddt(C));                                % form the constraint equations

% Baumgarte stabilization for constraint equations
% omega=100;    % stabilization should be slightly faster than the dynamics
% zeta = 1;     % for a critically damped system 
% gc = ddt(ddt(C))+ 2*zeta*omega*ddt(C) + omega^2*C; 

g = [gd; gc];   % the full set of equations to be solved includes both the dynamics and the constraints
x = [ddq; c];   % we want to solve for both the second time derivates of the generalized coordinates and the constraint forces

%%% Rearrange Equations of Motion. 
A = jacobian(g,x);
b = A*x - g;
bi = [jacobian(gd,ddq)*dq; zeros(size(c))];
% A\b yields the second time derivatives of the generalized coordinates and 
% the constraint forces for continuous dynamics.
% A\bi yields the first time derivatives of the generalized coordinates and 
% the constraint impulses for impulsive/discrete dynamics.
% See Lecture 8 slides for more information.

%%% Write functions to evaluate dynamics, etc...
z = sym(zeros(length([q;dq]),1)); % initialize the state vector
z(1:2:end,1) = q;  % the odd elements of state z are the generalized coordinates
z(2:2:end,1) = dq; % the even elements of state z are the first time derivatives of the generalized coordinates

% Write functions to a separate folder because we don't usually have to see them
directory = '../AutoDerived/';
% Write a function to evaluate the energy of the system given the current state and parameters
matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
% Write a function to evaluate the A matrix of the system given the current state and parameters
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
% Write a function to evaluate the b vector of the system given the current state, current control, and parameters
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u p});
% Write a function to evaluate the bi vector of the system given the state before impact and parameters
matlabFunction(bi,'file',[directory 'bi_' name], 'vars',{z p});

% The files generated below may need to be edited due to a
% matlabFunction bug that does not properly vectorize code when symbolic
% constants are written to a function file.
% Write a function to evaluate the X and Y coordinates of the points to be animated given the current state and parameters
matlabFunction(R,'file',[directory 'z2R_' name],'vars',{z p});
% Write a function to evaluate the X and Y coordinates and speeds of the center of mass given the current state and parameters
drcm = ddt(rcm);             % Calculate center of mass velocity vector
COM = [rcm(1:2); drcm(1:2)]; % Concatenate x and y coordinates and speeds of center of mass in array
matlabFunction(COM,'file',[directory 'COM_' name],'vars',{z p});
