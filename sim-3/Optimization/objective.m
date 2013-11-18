function f = objective(x,z0,p)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% f - scalar value of the function (to be minimized) evaluated for the
%     provided values of the decision variables.

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
    com = COM_bounding(zout,p);   %COM
    f = -com(1,end);         %pick out negative x

end