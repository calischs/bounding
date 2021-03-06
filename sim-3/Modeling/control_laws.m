function u = control_laws(t,z,ctrl,p,iphase)
%phases:
% 1: no feet in contact
% 2: rear foot in contact only
% 3: front foot in contact only
% 4: both feet in contact

%position control only.

    function t_out = clip_torque(t_in,max_torque)
        if t_in>max_torque
            t_out = max_torque*ones(size(t_in));
        elseif t_in<-max_torque
            t_out = -max_torque*ones(size(t_in));
        else
            t_out = t_in;
        end
    end

a1_max = .6;
a2_max = .6;
phi_max = .6;

    function [u1,u2,u3] = pd_control(a1d,a2d,phid)
        k = 6;                      % stiffness (N/rad)
        b = .2;                     % damping (N/(rad/s))
        
        a1 = z(7,:);                % rear leg angle
        da1 = z(8,:);               % rear leg angular velocity
        u1 = clip_torque(-k*(a1-a1d) - b*da1, a1_max);   % apply PD control  
        
        a2 = z(9,:);                % front leg angle
        da2 = z(10,:);              % front leg angular velocity
        u2 = clip_torque(-k*(a2-a2d) - b*da2, a2_max);   % apply PD control 
        
        phi = z(11,:);              % spine motor angle
        dphi = z(12,:);             % spine motor angular velocity
        u3 = clip_torque(-k*(phi-phid) - b*dphi, phi_max);   % apply PD control 
    end

    function [p1,p2,p3] = get_control(t_i,t_f,p_i,p_m,p_f)
        ctrl.t = linspace(t_i,t_f,3); %time is in first segment        
        p1 = interp1(ctrl.t, [p_i(1), p_m(1), p_f(1)], t, 'linear','extrap');
        p2 = interp1(ctrl.t, [p_i(2), p_m(2), p_f(2)], t, 'linear','extrap');
        p3 = interp1(ctrl.t, [p_i(3), p_m(3), p_f(3)], t, 'linear','extrap');
    end

switch iphase
    case 2          % 2: rear foot in contact only
        if t < ctrl.tf3 %really this should be tf1, but we add a buffer for safety.
            [p1,p2,p3] = get_control(0., ctrl.tf1, ctrl.p1, ctrl.p12, ctrl.p2);
        else
            [p1,p2,p3] = get_control(ctrl.tf4, ctrl.tf5, ctrl.p1, ctrl.p12, ctrl.p2); %control is periodic
        end
    case 1          % 1: no feet in contact
        if t < ctrl.tf2                             %first flight phase
            [p1,p2,p3] = get_control(ctrl.tf1, ctrl.tf2, ctrl.p2, ctrl.p23, ctrl.p3);
        else                                        %second flight phase
            [p1,p2,p3] = get_control(ctrl.tf3, ctrl.tf4, ctrl.p4, ctrl.p45, ctrl.p5);
        end
    
    case 3          % 3: front foot in contact only
        [p1,p2,p3] = get_control(ctrl.tf2, ctrl.tf3, ctrl.p3, ctrl.p34, ctrl.p4);
        
    case 4          % 4: both feet in contact, should not get here.
        p1 = pi/4;
        p2 = pi/4;
        p3 = pi/4;
        %somewhat arbitrary
end
[u1,u2,u3] = pd_control(p1,p2,p3);
u = [u1;u2;u3];
end
