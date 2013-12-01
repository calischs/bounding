function [f,cineq,ceq] = computeall(x,ctrl,p)
    ctrl.T = x(1);
    %ctrl.ramp = x(2);
    %ctrl.stand = x(3);
    %ctrl.bent = x(4);
    %ctrl.spine_amp = x(5);
    
    tspan = [0, 3*ctrl.T];
    [tout zout uout indices iphases] = hybrid_simulation(ctrl,p,tspan);

    %%%
    %objective
    %%%
    com = COM_bounding(zout,p);     %COM
    f = com(1,end);                %pick out x
    %f = norm(zout(2:12,end)' - z0(2:12)); %make periodic in all state variables except x
    
    %%%
    %constraints
    %%%
    a1 = zout(7,:);
    a2 = zout(9,:);
    phi = zout(11,:);    
    cineq = [-min(a1), max(a1-pi/2), -min(a2), max(a2-pi/2), -min(phi+pi), max(phi-pi)]; %enforce joint limits                                     
    
    %equality constraints:
    %ceq_phases = [norm(zout(2:12,end)' - z0(2:12))]; %make periodic in all but x
    ceq = [];
                                                                
end