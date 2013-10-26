function dz = dynamics_continuous(t,z,p,iphase)

%phases:
% 1: no feet in contact
% 2: rear foot in contact only
% 3: front foot in contact only
% 4: both feet in contact

u = control_laws(t,z,p,iphase); % get the control
A = A_bounding(z,p);                 % get the full A matrix
b = b_bounding(z,u,p);               % get the full b vector

% evaluate dz based on t, z p, and iphase
switch iphase
    case 1
        %flight
        A = A(1:6,1:6); %only care about x, y, th, a1, a2, phi
        b = b(1:6);
    case 2
        %rear contact
        i = [1,2,3,4,5,6,7,8];
        A = A(i,i); %care about x, y, th, a1, a2, phi, fx1, fy1
        b = b(i);
    case 3
        %front contact
        i = [1,2,3,4,5,6,9,10];
        A = A(i,i); %care about x, y, th, a1, a2, phi, fx2, fy2
        b = b(i);
    case 4
        %both contact
        i = [1,2,3,4,5,6,7,8,10];
        A = A(i,i); %care about x, y, th, a1, a2, phi, fx1, fy1, fy2 (leave out one contact force to avoid overconstraint...)
        b = b(i);
    otherwise
        %something wrong
        assert(false);
end

x = A\b; % solve system for accelerations (and possibly contact forces)

dz(1:2:12,1) = z(2:2:12,1);   % set speeds from state
dz(2:2:12,1) = x(1:6);         % set accelerations from solution

end