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

    ctrl.T1 = x(5:13);
    ctrl.T3 = x(14:22);
    ctrl.q2 = x(23:25);
    ctrl.q4 = x(26:28);
    tf = x(29);
    tspan = [0, tf];
    [tout zout uout indices] = hybrid_simulation(z0,ctrl,p,tspan);
    com = COM_bounding(zout,p);   %COM
    f = -com(1,end);         %pick out negative x

end