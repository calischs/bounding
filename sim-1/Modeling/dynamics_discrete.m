function zplus = dynamics_discrete(zm,p,ie,iphase)
% phase 1: no feet in contact
% phase 2: rear foot in contact only
% phase 3: front foot in contact only
% phase 4: both feet in contact

%event 1 is rear touchdown
%event 2 is front touchdown
%event 3 is rear takeoff
%event 4 is front takeoff

A = A_all(zm,p);                            % evaluate A matrix
bi = bi_all(zm,p);                          % evaluate impact b vector (bhat)

switch iphase
    case 1
        % flight
        if ie==1: %rear touchdown
            i = [1,2,3,4,5,6,7,8]; %we care about x,y,th,a1,a2,phi
        elseif ie==2: %front touchdown
            i = [1,2,3,4,5,6,9,10]; %we care about x,y,th,a1,a2,phi,fx2,fy2
        end
    case 2
        % rear contact
        if ie==3: %rear takeoff
            i = [1,2,3,4,5,6]; %we care about x,y,th,a1,a2,phi,fx1,fy1
        elseif ie==2: %front touchdown
            i = [1,2,3,4,5,6,7,8,10]; %we care about x,y,th,a1,a2,phi,fx1,fy1,fy2
        end
    case 3
        % front contact
        if ie==1: %rear touchdown
            i = [1,2,3,4,5,6,7,8,10]; %x,y,th,a1,a2,phi,fx1,fy1,fy2
        elseif ie==4: %front takeoff
            i = [1,2,3,4,5,6]; %x,y,th,a1,a2,phi
        end
    case 4
        % both contact
        if ie==3: %rear takeoff
            i = [1,2,3,4,5,6,9,10]; %x,y,th,a1,a2,phi,fx2,fy2
        elseif ie==4: %front takeoff
            i = [1,2,3,4,5,6,7,8]; %x,y,th,a1,a2,phi,fx1,fy1
        end
    otherwise
        %problem
        assert(false);
end
        
A = A(i,i);
bi = bi(i);

xi = A\bi;                        % solve for speeds after impact (and possibly impact impulse)
zplus(1:2:12) = zm(1:2:12);       % set the positions the same as they were
zplus(2:2:12) = xi(1:6);          % set the speeds after impact
                                  % don't need to do anything with impulse, if we have it.
end