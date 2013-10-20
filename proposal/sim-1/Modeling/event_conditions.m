function [value,isterminal,direction] = event_conditions(t,z,p,iphase)
%event 1 is takeoff from stance
%event 2 is touchdown from flight

if iphase == 2                      % in flight phase
    value(2) = z(1);                % detect touchdown: z(1) = y
    isterminal(2) = 1;              % terminate integration
    direction(2) = -1;              % detect decreasing height
else                                % in stance phase
    u = control_laws(t,z,p,iphase); % get the control
	A = A_all(z,p);                 % get the full A matrix
    b = b_all(z,u,p);               % get the full b vector
    x = A\b;                        % solve system
    F = x(3);
    value(1) = F;                   % detect takeoff, F is contact force
    isterminal(1) = 1;              % terminate integration
    direction(1) = -1;              % detect decreasing contact force
end

%add an event for leg over-extension
%value(3) = pi/2 - z(3); %th
%isterminal(3) = 0;
%direction(3) = -1;

end