function [cineq ceq] = constraints(x,z0,p)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% cineq - an array of values of nonlinear inequality constraint functions.  
%         The constraints are satisfied when these values are less than zero.
% ceq   - an array of values of nonlinear equality constraint functions.
%         The constraints are satisfied when these values are equal to zero.
    ctrl = struct();
    ctrl.tf1 = x(1);
    ctrl.tf2 = x(2);
    ctrl.tf3 = x(3);
    ctrl.tf4 = x(4);
    ctrl.tf5 = x(5);

    ctrl.p1 = x(6:9);
    ctrl.p2 = x(9:12);
    ctrl.p3 = x(12:15);
    ctrl.p4 = x(15:18);
    ctrl.p5 = x(18:21);

    tspan = [0, ctrl.tf5];
    [tout zout uout indices] = hybrid_simulation(z0,ctrl,p,tspan);
    %com_dy = COM_bounding(zout,p);
    %com_dy = com_dy(4,:);
    
    a1 = zout(7,:);
    a2 = zout(9,:);
    phi = zout(11,:);
    %maybe also think about keeping th in limits
    
    cineq = [-min(a1), max(a1-pi/2), -min(a2), max(a2-pi/2), -min(a2+pi/2), max(a2-pi/2)]; %enforce join limits                                     
    %equality constraints:
    %1. we want tf1 to be the end of first interval
    %2. we want tf2 to be the end of second interval
    %3. we want tf3 to be the end of third interval
    %4. we want tf4 to be the end of fourth interval

    ceq_phases = [];
    if length(indices) > 1 %last index reserved for end of simulation, so strictly greater
        ceq_phases = [ceq_phases ctrl.tf1 - tout(indices(1))];
    end
    if length(indices) > 2 
        ceq_phases = [ceq_phases ctrl.tf2 - tout(indices(2))];
    end
    if length(indices) > 3
        ceq_phases = [ceq_phases ctrl.tf3 - tout(indices(3))];
    end
    if length(indices) > 4
        ceq_phases = [ceq_phases ctrl.tf4 - tout(indices(4))];
    end
    
    %5. we want state at end of simulation to be same as state at beginning
    %(periodicity).  This gives 12 constraints.
    
    ceq_state = zout(end) - z0; %enforce periodicity
    
    ceq = [ceq_phases, ceq_state];                                            
                                                                
end