function [tout zout uout indices] = hybrid_simulation(z0,p,tspan)
t0 = tspan(1); tend = tspan(end);   % set initial and final times

%% Setup tolerance Options
inttol = 1e-6;  % set integration tolerances
iphase = 1;     % phase number is currently phase 1
sols = [];      % initialize array of solution structures
done = false;   % "done" flag is false (not raised)

while(t0 < tend && ~done)   % could raise flag "done" to stop integration before time is up
    
    % now we need to include the number of the phase in the call to the event function
    opts = odeset('Events', @(t,z) event_conditions(t,z,p,iphase), 'abstol',inttol,'reltol',inttol);
    f = @(t,z) dynamics_continuous(t, z, p, iphase);    % we also include the number of the phase in the call to the EoM function
    sol = ode45(f, [t0 tend], z0, opts);    % integate until an event happens or time runs out
    sol.iphase = iphase;                    % store the phase number in the solution structure
    t0 = sol.x(end);                        % reset the integration initial time
     
    if isfield(sol,'ie') && ~isempty(sol.ie)
        z0 = dynamics_discrete(sol.ye(:,end),p,sol.ie(end),iphase);   % run the discrete dynamics function
        switch iphase
            case 1 %flight
                if any(sol.ie == 1) %rear touchdown
                    iphase = 2; %change to rear contact
                elseif any(sol.ie == 2) %front touchdown
                    iphase = 3; %change to front contact
                end
            case 2 %rear contact only
                if any(sol.ie == 3) %rear takeoff
                    iphase = 1; %change to flight
                elseif any(sol.ie == 2) %front touchdown
                    iphase = 4; %change to both contact
                end
            case 3 %front contact only
                if any(sol.ie == 1) %rear touchdown
                    iphase = 4; %both contact
                elseif any(sol.ie == 4) %front takeoff
                    iphase = 1; %flight
                end
            case 4 %both contact
                if any(sol.ie == 3) %rear takeoff
                    iphase = 3; %front contact only
                elseif any(sol.ie == 4) %front takeoff
                    iphase = 2; %rear contact only
    else
        sol.ie = []; sol.xe = []; sol.ye = [];      % leave this just in case no event occured
    end
    
    sols = [sols; sol];                             % append solution structure to array
end

% assemble the results from each solution structure in the array
tout = []; zout = []; uout = []; indices = [];
for ii = 1:length(sols)
    sol = sols(ii);                                     % get the next solution structure
    iphase = sol.iphase;                                % get the phase number
    sol.x(1) = sol.x(1)+1e-6;                           % the time starts just a little after the last time ended (required for animation)
    tout = [tout sol.x];                                % append time points
    zout = [zout sol.y];                                % append states
    uout = [uout control_laws(sol.x,sol.y,p,iphase)];   % append controls
    indices = [indices length(tout)];                   % append indices at which phases end
end

end
