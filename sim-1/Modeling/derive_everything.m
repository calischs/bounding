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
syms    t ...               %time
        x dx ddx ...        %horizontal position of back foot
        y dy ddy ...        %height of back foot
        th dth ddth ...     %body pitch angle
        a1 da1 dda1 ...     %back leg angle
        a2 da2 dda2 ...     %front leg angle
        phi dphi ddphi ...  %phi is spine motor angle
        psi dpsi ...        %psi = (mtd/sep)*phi is spine bend angle, defined only for convenience...
        tau1 tau2 tau3 ...  %control torques at back leg, front leg, and spine
        Fx1 Fy1 Fx2 Fy2 ... %constraint forces for back foot and front foot
        ma1 ma2 ...         %mounting angles of back and front legs (0 -> perpendicular, + rotates ccw) 
        l ... %leg segment lengths
        c11 c12 c21 c22... %leg segment COM positions
        m11 m12 m21 m22 ... %leg segment masses
        I11 I12 I21 I22 ... %leg segment MOIs
        ls cs kappa ms sep mtd... %spine length, center of mass (along), spine spring constant, mass, tendon sep, motor takeup diam
        mmh g num_sps real           %motor mass (one at hip, one at shoulder), gravity, type

% I'm still not sure how to deal with indeterminant configurations (e.g.,
% when both feet are on the ground).  This possibility explains the
% difference in numbers of generalized coordinates (6) and generalized
% forces (7).  I hope that when writing the rest of the simulator this
% confusion clears up.

q   = [x; y; th; a1; a2; phi];      % generalized coordinates
dq  = [dx; dy; dth; da1; da2; dphi];    % first time derivatives
ddq = [ddx; ddy; ddth; dda1; dda2; ddphi];  % second time derivatives
u   = [tau1; tau2; tau3];  % control forces and moments
c   = [Fx1; Fy1; Fx2; Fy2];           % constraint forces and moments
p   = [ l; c; m; I; ...
        ls; cs; kappa; ms; sep; mtd; ...
        mmh; g; num_sps];  % parameters

%%% Calculate important vectors and their time derivatives.

% Define fundamental unit vectors.
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);

rotated_coordinates = @(a) [cos(a)*ihat + sin(a)*jhat, ...
                            -sin(a)*ihat + cos(a)*jhat];

psi = (mtd/sep)*phi; %transform between motor angle (gen coord) and subtended spine angle (useful)
dpsi = (mtd/sep)*dphi;
%hats are rotated coordinates
%i:from x, j: from y
%1:rear, 2:front
%s:end effector to mid joint (knee/elbow)
%r:mid joint to motor
[i1hat,j1hat] = rotated_coordinates(th);      %coordinate system of rear
[~,rj1hat] = rotated_coordinates(th+ma1+a1);   %pointing knee to hip in j
[~,sj1hat] = rotated_coordinates(th+ma1-a1);   %pointing foot to knee in j

%[i2hat,j2hat] = rotated_coordinates(th+psi);  %coordinate system of front
[~,rj2hat] = rotated_coordinates(th+psi+ma2+a2);  %pointing elbow to shoulder in j
[~,sj2hat] = rotated_coordinates(th+psi+ma2-a2);  %pointing front foot to elbow in j

ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% Define vectors 
f1 = x*ihat + y*jhat;   %origin to rear foot
m1 = f1 + l*sj1hat;     %origin to rear mid joint (knee)
h1 = m1 + l*rj1hat;     %origin to rear hip

%return point along spine arc given by paremter t in [0,1]
%spine_points = @(t) i1hat * (ls/psi) * sin(t*psi) + j1hat * (ls/psi) * (1-cos(t*psi));
%this is unsafe due to zero division, and ternary expressions don't play
%with anonymous functions in matlab...

%this is clunky but it should work...
spine_points = [h1]; %this array will keep track of our discretization of the arc.
vertebra = [h1 + .5*sep*j1hat, h1 - .5*sep*j1hat]; %this array will keep track of points for drawing vertebra.
for i=1:num_sps-1
    ang = i*psi/(num_sps-1);
    new_p = spine_points(end) + (ls/num_sps)*(cos(ang)*i1hat + sin(ang)*j1hat);
    vertebra_vect = .5*sep*( -sin(ang)*i1hat + cos(ang)*j1hat;
    spine_points = [spine_points new_p];
    vertebra = [vertebra new_p+vertebra_vect new_p-vertebra_vect];
end

h2 = spine_points(end); %front hip
m2 = h2 - l*rj2hat; %front mid
f2 = m2 - l*sj2hat; %front foot

%centers of masses
%rear shin, rear thigh, front shin, front thigh, spine
rcm11 = f1 + c11*sj1hat;
rcm12 = m1 + c12*rj1hat;
rcm21 = f2 + c21*sj2hat;
rcm22 = m2 + c22*rj2hat;
rm1 = h1; %motors
rm2 = h2;

rcms = sum(spine_points,2)/n; %let's assume the spine consists of point masses at each spine point
Is = ms*sum( dot(spine_points-rcms, spine_points-rcms) ); %moment of inertia of spine relative to COM

% Take time derivatives of vectors as required for kinetic energy terms.
drcm11 = ddt(rcm11);
drcm12 = ddt(rcm12);
drcm21 = ddt(rcm21);
drcm22 = ddt(rcm22);
drcms   = ddt(rcms);
drm1 = ddt(h1);
drm2 = ddt(h2);

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
        m22*dot(rcm22, jhat) ) + ...
    (1/2)*kappa*psi^2;

% Define contributions to generalized forces.
QFx1 = F2Q(Fx1*ihat,f1);
QFy1 = F2Q(Fy1*jhat,f1);
QFx2 = F2Q(Fx2*ihat,f2);
QFy2 = F2Q(Fy2*jhat,f2);
Qtau1 = M2Q(-tau1*khat, -da1*khat);
Qtau2 = M2Q(-tau2*khat, -da2*khat);
Qtau3 = M2Q(-tau3*khat, -dphi*khat); %not sure about this one.

% Sum generalized force contributions.
Q = QFx1+QFy1+QFx2+QFy2 + Qtau1+Qtau2+Qtau3;

% Assemble R, the array of cartesian coordinates of the points to be animated.
% 1:2 leaves out z coordinate
%R = [f1(1:2); m1(1:2); h1(1:2); f2(1:2); m2(1:2); h2(1:2); spine_points(1:2,:); vertebra(1:2,:)];
R = [f1(1:2); m1(1:2); f2(1:2); m2(1:2); spine_points(1:2,:); vertebra(1:2,:)]; %including hip/shoulder explicitly is redundant..
%maybe eventually we should model a first and last segment w specified lengths

% Calculate rcm, the location of the center of mass
rcm = (m11*rcm11 + m12*rcm12 + m21*rcm21 + m22*rcm22 + mh*h1 + mh+h2 + ms*rcms)/(m11+m12+m21+m22+mh+mh+ms);

% Assemble C, the set of constraints
C = [y; f2(2)]; %feet on ground

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
