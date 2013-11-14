function u = control_laws(t,z,ctrl,p,iphase)
%phases:
% 1: no feet in contact
% 2: rear foot in contact only
% 3: front foot in contact only
% 4: both feet in contact

a1_max = .3;
a2_max = .3;
phi_max = .3;

    function t_out = clip_torque(t_in,max_torque)
        if t_in>max_torque
            t_out = max_torque*ones(size(t_in));
        elseif t_in<-max_torque
            t_out = -max_torque*ones(size(t_in));
        else
            t_out = t_in;
        end
    end

    function u1 = pd_control_1(a1d)
        k = 5;                      % stiffness (N/rad)
        b = .5;                     % damping (N/(rad/s))
        a1 = z(7,:);                % rear leg angle
        da1 = z(8,:);               % rear leg angular velocity
        u1 = clip_torque(-k*(a1-a1d) - b*da1, a1_max);   % apply PD control         
    end
    function u2 = pd_control_2(a2d)
        k = 5;                      % stiffness (N/rad)
        b = .5;                     % damping (N/(rad/s))
        a2 = z(9,:);                % front leg angle
        da2 = z(10,:);              % front leg angular velocity
        u2 = clip_torque(-k*(a2-a2d) - b*da2, a2_max);   % apply PD control         
    end

    function [u1,u2,u3] = pd_control(a1d,a2d,phid)
        k = 5;                      % stiffness (N/rad)
        b = .5;                     % damping (N/(rad/s))
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


%we aim to have a phase sequence (2, 1, 3, 1, ...)

switch iphase
    case 1          % 1: no feet in contact
        %stuff
        if t < ctrl.tf2                                 %first flight phase
            [u1,u2,u3] = pd_control(ctrl.q2(1),ctrl.q2(2),ctrl.q2(3));   %extend legs, hyperextend spine
        else                                            %second flight phase
            [u1,u2,u3] = pd_control(ctrl.q4(1),ctrl.q4(2),ctrl.q4(3));    %shorten rear leg, extend front leg, curl spine
        end
    case 2          % 2: rear foot in contact only
        ctrl.t = linspace(0,ctrl.tf1,length(ctrl.T1(1:3:end))); %time is in first segment
        u1 = interp1(ctrl.t,ctrl.T1(1:3:end),t,'linear','extrap');
        %u2 = interp1(ctrl.t,ctrl.T1(2:3:end),t,'linear','extrap');
        u2 = pd_control_2(interp1(ctrl.t,ctrl.T1(2:3:end),t,'linear','extrap'));
        u3 = interp1(ctrl.t,ctrl.T1(3:3:end),t,'linear','extrap');
    case 3          % 3: front foot in contact only
        ctrl.t = linspace(ctrl.tf2,ctrl.tf3,length(ctrl.T3(1:3:end))); %time is in first segment
        %u1 = interp1(ctrl.t,ctrl.T3(1:3:end),t,'linear','extrap');
        u1 = pd_control_1( interp1(ctrl.t,ctrl.T3(1:3:end),t,'linear','extrap') );
        u2 = interp1(ctrl.t,ctrl.T3(2:3:end),t,'linear','extrap');
        u3 = interp1(ctrl.t,ctrl.T3(3:3:end),t,'linear','extrap');    
    case 4          % 4: both feet in contact, should not get here.
        [u1,u2,u3] = pd_control(pi/4,pi/4,pi/4); %somewhat arbitrary
end
u = [u1;u2;u3];
end
