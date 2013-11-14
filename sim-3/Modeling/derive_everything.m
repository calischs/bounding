function derive_everything() 
name = 'bounding';
%This simulation is for a bounding robot with three degrees of freedom: a
%rear extensional leg, a front extensional leg, and a bending spine.  We
%model the legs as in homeworks.  

%To actuate the spine, we are thinking of two opposing tendons attached to
%a motor takeup wheel of diameter $d$.  So a rotation $phi$ of the motor
%lengthens one tendon by $phi*d/2$ and shortens the opposing tendon by the
%same amount.  If the tendons have separation $w$ (each offset by $w/2$
%from the axis), the angle subtended by the bent spine $psi$ is given by
%$psi=(d/w)*phi$.  If we set up coordinate systems on the hip and shoulder,
%this same angle gives the relative rotation between them.  We use this to
%define helpful rotated coordinate systems for defining points on the
%robot.

%If the spine has length $L$, the radius of curvature is $L/psi$.  We equip
%the spine with a torsional spring, at this point defined somewhat
%arbitrarily as $torque = kappa * psi$.  We will have to determine the best
%ways to physcially implement the spine spring, and what force law it
%obeys.

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
% the robot has internal configuration variables a1,a2,phi
% as well as rigid body motion variables x,y,th
syms    t real                  %time
syms    x dx ddx real           %horizontal position of back foot
syms    y dy ddy real           %height of back foot
syms    th dth ddth real        %body pitch angle
syms    a1 da1 dda1 real        %back leg angle
syms    a2 da2 dda2 real        %front leg angle
syms    phi dphi ddphi real     %phi is spine motor angle
syms    psi dpsi real           %psi = (mtd/sep)*phi is spine bend angle, defined only for convenience...
syms    tau1 tau2 tau3 real     %control torques at back leg, front leg, and spine
syms    FX1 FY1 FX2 FY2 real    %constraint forces for back foot and front foot
syms    ma1 ma2 real            %mounting angles of back and front legs (0 -> perpendicular, + rotates ccw) 
syms    ll real                 %leg segment length
syms    c11 c12 c21 c22 real    %leg segment COM positions
syms    m11 m12 m21 m22 real    %leg segment masses
syms    I11 I12 I21 I22 real    %leg segment MOIs
syms    slength kappa ms sep mtd ma1 ma2 real %spine length, center of mass (along), spine spring constant, spine mass, tendon sep, motor takeup diam, mounting angles 1 and 2
syms    mmh mms g real      %motor mass (one at hip, one at shoulder), spine motor mass, gravity, type

% There are a different number of generalized coordinates (6) and
% generalized forces (7).  But, in each of the phases (flight, rear
% contact, front contact, and both contact), ...

q   = [x; y; th; a1; a2; phi];      % generalized coordinates
dq  = [dx; dy; dth; da1; da2; dphi];    % first time derivatives
ddq = [ddx; ddy; ddth; dda1; dda2; ddphi];  % second time derivatives
u   = [tau1; tau2; tau3];  % control forces and moments
c   = [FX1; FY1; FX2; FY2]; % constraint forces and moments
p   = [ ll; ...
        c11; c12; c21; c22; ...       
        m11; m12; m21; m22; ...
        I11; I12; I21; I22; ...
        slength; kappa; ms; sep; mtd; ma1; ma2; ...
        mmh; mms; g];  % parameters
%%% Calculate important vectors and their time derivatives.

% Define fundamental unit vectors.
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);

rotated_i = @(a) cos(a)*ihat + sin(a)*jhat;
rotated_j = @(a) -sin(a)*ihat + cos(a)*jhat;

psi = (mtd/sep)*phi; %transform between motor angle (gen coord) and subtended spine angle (useful)
dpsi = (mtd/sep)*dphi;
%hats are rotated coordinates
%i:from x, j: from y
%1:rear, 2:front
%s:end effector to mid joint (knee/elbow)
%r:mid joint to motor
i1hat = rotated_i(th);      %coordinate system of rear
j1hat = rotated_j(th);      %coordinate system of rear
rj1hat = rotated_j(th+ma1+a1);   %pointing knee to hip in j
sj1hat = rotated_j(th+ma1-a1);   %pointing foot to knee in j
rj2hat = rotated_j(th+psi+ma2+a2);  %pointing elbow to shoulder in j
sj2hat = rotated_j(th+psi+ma2-a2);  %pointing front foot to elbow in j

ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% Define vectors 
f1 = x*ihat + y*jhat;   %origin to rear foot
m1 = f1 + ll*sj1hat;     %origin to rear mid joint (knee)
h1 = m1 + ll*rj1hat;     %origin to rear hip

%this is clunky but it should work...
num_sps = 8.;
spine_points = [h1]; %this array will keep track of our discretization of the arc.
vertebra = [h1 + .5*sep*j1hat, h1 - .5*sep*j1hat]; %this array will keep track of points for drawing vertebra.
dl = (slength/num_sps);
for i=1:num_sps-1
    ang = i*psi/(num_sps-1);
    new_p = spine_points(:,end) + dl*rotated_i(th+ang);
    vertebra_vect = .5*sep*rotated_j(th+ang);
    spine_points = [spine_points new_p];
    vertebra = [vertebra new_p+vertebra_vect new_p-vertebra_vect];
end

h2 = spine_points(:,end); %front hip
m2 = h2 - ll*rj2hat; %front mid
f2 = m2 - ll*sj2hat; %front foot

%centers of masses
%rear shin, rear thigh, front shin, front thigh, spine
rcm11 = f1 + c11*sj1hat;
rcm12 = m1 + c12*rj1hat;
rcm21 = f2 + c21*sj2hat;
rcm22 = m2 + c22*rj2hat;
rm1 = h1; %motors
rm2 = h2;
rm_spine = spine_points(:,length(spine_points)/2); %spine points assumed even

rcms = sum(spine_points,2)/num_sps; %let's assume the spine consists of point masses at each spine point
Is = 0;
for i=1:length(spine_points)
    Is = Is + dot( spine_points(i) - rcms , spine_points(i) - rcms );
end
Is = ms*Is; %moment of inertia of spine relative to COM

% Take time derivatives of vectors as required for kinetic energy terms.
drcm11 = ddt(rcm11);
drcm12 = ddt(rcm12);
drcm21 = ddt(rcm21);
drcm22 = ddt(rcm22);
drcms   = ddt(rcms);
drm1 = ddt(h1);
drm2 = ddt(h2);
drms = ddt(rm_spine);

%%% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
% F2Q calculates the contribution of a force to all generalized forces
% for forces, F is the force vector and r is the position vector of the 
% point of force application
F2Q = @(F,r) simple(jacobian(r,q)'*(F)); 

% M2Q calculates the contribution of a moment to all generalized forces
% M is the moment vector and w is the angular velocity vector of the
% body on which the moment acts
M2Q = @(M,w) simple(jacobian(w,dq)'*(M)); 

% Define kinetic energies.
T = (1/2)*(     m11*dot(drcm11, drcm11) + ...
                m12*dot(drcm12, drcm12) + ...
                m21*dot(drcm21, drcm21) + ...
                m22*dot(drcm22, drcm22) + ...
                mmh*dot(drm1, drm1) + ...
                mmh*dot(drm2, drm2) + ...
                mms*dot(drms, drms) + ...
                ms*dot(drcms, drcms) + ...
                I11 * da1^2 + ...
                I12 * da1^2 + ...
                I21 * da2^2 + ...
                I22 * da2^2 + ...
                Is * dth^2 );

% Define potential energies. 
V = g*( m11*dot(rcm11, jhat) + ...
        m12*dot(rcm12, jhat) + ...
        m21*dot(rcm21, jhat) + ...
        m22*dot(rcm22, jhat) + ...
        mmh*dot(h1,jhat) + ...
        mmh*dot(h2,jhat) + ...
        mmh*dot(rm_spine,jhat) ...
    ) + ...
    (1/2)*kappa*psi^2;

% Define contributions to generalized forces.
QFX1 = F2Q(FX1*ihat,f1);
QFY1 = F2Q(FY1*jhat,f1);
QFX2 = F2Q(FX2*ihat,f2);
QFY2 = F2Q(FY2*jhat,f2);
Qtau1 = M2Q(-tau1*khat, -da1*khat);
Qtau2 = M2Q(-tau2*khat, -da2*khat);
Qtau3 = M2Q(-tau3*khat, -dphi*khat); 

% Sum generalized force contributions.
Q = QFX1+QFY1+QFX2+QFY2 + Qtau1+Qtau2+Qtau3;

% Assemble R, the array of cartesian coordinates of the points to be animated.
% 1:2 leaves out z coordinate
R = [f1(1:2); m1(1:2); f2(1:2); m2(1:2)];
%R = [R; reshape(spine_points(1:2,:),[],1)]; %including hip/shoulder explicitly is redundant..
R = [R; reshape(spine_points(1:2,:),[],1); reshape(vertebra(1:2,:),[],1)]; %including hip/shoulder explicitly is redundant..
%maybe eventually we should model a first and last segment w specified lengths

% Calculate rcm, the location of the center of mass
rcm = (m11*rcm11 + m12*rcm12 + m21*rcm21 + m22*rcm22 + mmh*h1 + mmh+h2 + ms*rcms)/(m11+m12+m21+m22+mmh+mmh+ms);

% Assemble C, the set of constraints
C = [x; y; f2(1); f2(2)]; %Note: since we enforce acceleration, not position, we don't need to save the step positions.

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
b = simplify(A*x - g); %without simplify, this doesn't clear FX2 and FY2 from the expression and throws error later.
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
%matlabFunction(E,'file',[directory 'energy_' name],'vars',{z p});
% Write a function to evaluate the A matrix of the system given the current
% state, parameters, and last steps taken
%matlabFunction(A,'file',[directory 'A_' name],'vars',{z p last_steps});
disp('Writing A function');
matlabFunction(A,'file',[directory 'A_' name],'vars',{z p});
% Write a function to evaluate the b vector of the system given the current state, current control, and parameters
disp('Writing b function');
matlabFunction(b,'file',[directory 'b_' name],'vars',{z u p});
% Write a function to evaluate the bi vector of the system given the state before impact and parameters
disp('Writing bi function');
matlabFunction(bi,'file',[directory 'bi_' name], 'vars',{z p});

% The files generated below may need to be edited due to a
% matlabFunction bug that does not properly vectorize code when symbolic
% constants are written to a function file.
% Write a function to evaluate the X and Y coordinates of the points to be animated given the current state and parameters
disp('Writing R function');
matlabFunction(R,'file',[directory 'z2R_' name],'vars',{z p});
% Write a function to evaluate the X and Y coordinates and speeds of the center of mass given the current state and parameters
disp('Writing COM function');
drcm = ddt(rcm);             % Calculate center of mass velocity vector
COM = [rcm(1:2); drcm(1:2)]; % Concatenate x and y coordinates and speeds of center of mass in array
%matlabFunction(COM,'file',[directory 'COM_' name],'vars',{z p});
