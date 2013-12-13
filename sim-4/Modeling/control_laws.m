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
        k = 40;                      % stiffness (N/rad)
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
    
        %enforce joint limits
        if(a1<0)
            u1 = 100;
        elseif(a1>pi/2)
            u1 = -100;
        end
        if(a2<0)
            u2 = 100;
        elseif(a2>pi/2)
            u2 = -100;
        end
    end

a1_int = interp1(ctrl.t, ctrl.a1_v, mod(t,ctrl.T));
a2_int = interp1(ctrl.t, ctrl.a2_v, mod(t,ctrl.T));
phi_int = interp1(ctrl.t, ctrl.phi_v, mod(t,ctrl.T));

[u1,u2,u3] = pd_control(a1_int,a2_int,phi_int);

u = [u1;u2;u3];
end
