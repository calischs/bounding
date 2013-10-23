function [value,isterminal,direction] = event_conditions(t,z,p,iphase)
%event 1 is rear touchdown
%event 2 is front touchdown
%event 3 is rear takeoff
%event 4 is front takeoffs

R = z2R_all(z,p); %get coordinates from state.
u = control_laws(t,z,p,iphase); % get the controls at this instant

switch iphase
    case 1
        % in flight, events 1 and 2 are possible
        %rear touchdown (1)
        value(1) = z(3); %z(3) = y
        isterminal(1) = 1;
        direction(1) = -1;
        %front touchdown (2)
        value(2) = R(2,3); %R(2,3) = front foot y
        isterminal(2) = 1;
        direction(2) = -1;
    case 2
        % in only rear contact events 2 and 3 are possible
        %front touchdown (2)
        value(2) = R(2,3); %R(2,3) = front foot y
        isterminal(2) = 1;
        direction(2) = -1;
        
        %rear takeoff (3)
        i = [1,2,3,4,5,6,7,8]; %in rear contact, we care about x, y, th, a1, a2, phi, fx1, fy1
        A = A_all(z,p)(i,i);
        b = b_all(z,u,p)(i);
        x = A\b;
        value(3) = x(end); %rear ground reaction force Fy1
        isterminal(3) = 1;
        direction(3) = -1;
    case 3
        % in only front contact events 1 and 4 are possible 
        % rear touchdown (1)
        value(1) = z(3); %z(3) = y
        isterminal(1) = 1;
        direction(1) = -1;
        % front takeoff (4)
        i = [1,2,3,4,5,6,9,10]; %in front contact, we care about x, y, th, a1, a2, phi, fx2, fy2
        A = A_all(z,p)(i,i);
        b = b_all(z,u,p)(i);
        x = A\b;
        value(3) = x(end); %front ground reaction force Fy2
        isterminal(3) = 1;
        direction(3) = -1;        
    case 4
        % in both contact events 3 and 4 are possible
        i = [1,2,3,4,5,6,7,8,10]; %in both contact, we care about x, y, th, a1, a2, phi, fx1, fy1, fy2
        A = A_all(z,p)(i,i);
        b = b_all(z,u,p)(i);
        x = A\b;
        % rear takeoff (3)
        value(3) = x(8); %rear ground reaction force Fy1
        isterminal(3) = 1;
        direction(3) = -1;    
        % front takeoff (3)
        value(3) = x(end); %front ground reaction force Fy2
        isterminal(3) = 1;
        direction(3) = -1;    
    otherwise
        %problem
        assert(false);
end
end