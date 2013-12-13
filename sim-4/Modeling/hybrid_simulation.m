function [tout zout uout indices, iphases] = hybrid_simulation(ctrl,p,tspan)
t0 = tspan(1); tend = tspan(end);   % set initial and final times

%% Setup tolerance Options
%phases:
% 1: no feet in contact
% 2: rear foot in contact only
% 3: front foot in contact only
% 4: both feet in contact
inttol = 1e-6;  % set integration tolerances
iphase = 4; %starting phase, contact    
sols = [];      % initialize array of solution structures
done = false;   % "done" flag is false (not raised)

%need to compute initial theta value from the stand position we assume
%based on ctrl.stand, ctrl.bent, and parameters:
spine_length = .75*p(14);
leg_segment_length = p(1);
a1 = ctrl.stand;
a2 = ctrl.bent;
zth0 = -atan2( (cos(a1) - cos(a2))*2*leg_segment_length,  spine_length );
%initial: [x dx y dy   th   dth      a1 da1    a2   da2   phi dphi]
z0 =      [0  0 0  0   zth0      0.  a1  0.   a2    0.    0    0.]; 

%pre calculate the array for control
f = 500; %frequence
spine_p = ctrl.spine_amp;
spine_m = -ctrl.spine_amp;
ramp = ctrl.ramp;
w = .5*(1-2*ramp);
bent = ctrl.bent;
stand = ctrl.stand;
spine_shift = round(ctrl.spine_shift*f*ctrl.T);
a1 = [0., .5*w, .5*w+ramp, 1.5*w+ramp, 1.5*w+2*ramp, 1.;...
	  stand, stand, bent, bent, stand, stand];
a2 = [0., .5*w, .5*w+ramp, 1.5*w+ramp, 1.5*w+2*ramp, 1.;...
	  bent, bent, stand, stand, bent, bent];
phi = [0., .5*w, .5*w+ramp, 1.5*w+ramp, 1.5*w+2*ramp, 1.;...
	   spine_m,   spine_m,      spine_p,         spine_p,            spine_m,      spine_m];
a1(1,:) = a1(1,:)*ctrl.T;
a2(1,:) = a2(1,:)*ctrl.T;
phi(1,:) = phi(1,:)*ctrl.T;
t = 0:1/f:ctrl.T;

a1_v = interp1(a1(1,:)',a1(2,:)',t);
a2_v = interp1(a2(1,:)',a2(2,:)',t);
phi_v = interp1(phi(1,:)',phi(2,:)',t);

a1_v = smooth(t,a1_v,.2,'lowess');
a2_v = smooth(t,a2_v,.2,'lowess');
phi_v = smooth(t,phi_v,.2,'lowess');
phi_v = circshift(phi_v,[-spine_shift,0]);

hold on
plot(a1(1,:)',a1(2,:)');
plot(t,a1_v);
plot(a2(1,:)',a2(2,:)');
plot(t,a2_v);
plot(phi(1,:)',phi(2,:)');
plot(t,phi_v);

ctrl.t = t;
ctrl.a1_v = a1_v;
ctrl.a2_v = a2_v;
ctrl.phi_v = phi_v;


while(t0 < tend && ~done)   % could raise flag "done" to stop integration before time is up
    
    % now we need to include the number of the phase in the call to the event function
    opts = odeset('Events', @(t,z) event_conditions(t,z,ctrl,p,iphase), 'abstol',inttol,'reltol',inttol);
    f = @(t,z) dynamics_continuous(t, z, ctrl, p, iphase);
    sol = ode45(f, [t0 tend], z0, opts);    % integate until an event happens or time runs out
    sol.iphase = iphase;                    % store the phase number in the solution structure
    t0 = sol.x(end);                        % reset the integration initial time
    
    switch iphase
        case 1
            disp('flight');
        case 2
            disp('rear contact');
        case 3
            disp('front contact');             
        case 4
            disp('both contact');
    end
            
    if isfield(sol,'ie') && ~isempty(sol.ie)
        z0 = dynamics_discrete(sol.ye(:,end),p,sol.ie(end),iphase);   % run the discrete dynamics function
        R = z2R_bounding(z0,p);
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
                end
            otherwise
                %shouldn't be here
                assert(false);
        end
    else
        sol.ie = []; sol.xe = []; sol.ye = [];      % leave this just in case no event occured
    end
    
    sols = [sols; sol];                             % append solution structure to array
end

% assemble the results from each solution structure in the array
tout = []; zout = []; uout = []; indices = [];iphases = [];
for ii = 1:length(sols)
    sol = sols(ii);                                     % get the next solution structure
    iphase = sol.iphase;                                % get the phase number
    sol.x(1) = sol.x(1)+1e-6;                           % the time starts just a little after the last time ended (required for animation)
    tout = [tout sol.x];                                % append time points
    zout = [zout sol.y];                                % append states
    uout = [uout control_laws(sol.x,sol.y,ctrl,p,iphase)];   % append controls
    indices = [indices length(tout)];                   % append indices at which phases end
    iphases = [iphases iphase];
end

end
