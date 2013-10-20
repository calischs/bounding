function zplus = dynamics_discrete(zm,p,ie)
A = A_all(zm,p);                            % evaluate A matrix
bi = bi_all(zm,p);                          % evaluate impact b vector (bhat)

if ie == 2              % if the second event (touchdown) occured
    %must satisfy constraints so keep everything.
    disp('touchdown');
else                    % if the first event (takeoff) occured
    %need not satisfy constraint, so keep only first two equations.
    A = A(1:2,1:2); 
    bi = bi(1:2);  
    disp('takeoff');
end

xi = A\bi;                            % solve for speeds after impact (and possibly impact impulse)
zplus(1:2:4) = zm(1:2:4);             % set the positions the same as they were
zplus(2:2:4) = xi(1:2);               % set the speeds after impact
                                      % don't need to do anything with impulse, if we have it.
end