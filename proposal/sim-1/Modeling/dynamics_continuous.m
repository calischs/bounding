function dz = dynamics_continuous(t,z,p,iphase)

u = control_laws(t,z,p,iphase); % get the control
A = A_all(z,p);                 % get the full A matrix
b = b_all(z,u,p);               % get the full b vector

if iphase == 2  
    % in flight phase
    % evaluate dz based on t, z p, and iphase
    A = A(1:2,1:2); %note, in flight, we only care about the generalized coordinates, y & th
    b = b(1:2);
else
    % in stance phase
    % evaluate dz based on t, z p, and iphase
    % keep all coordinates and constraints
end
x = A\b;                    % solve system for accelerations (and possibly contact forces)
dz(1:2:3,1) = z(2:2:4,1);   % set speeds from state
dz(2:2:4) = x(1:2);         % set accelerations from solution
end