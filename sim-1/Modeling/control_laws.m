function u = control_laws(t,z,p,iphase)
%phases:
% 1: no feet in contact
% 2: rear foot in contact only
% 3: front foot in contact only
% 4: both feet in contact

%x dx y dy th dth a1 da1 a2 da2 phi dphi
y = z(2,:);
a1 = z(7,:);
da1 = z(8,:);
a2 = z(9,:);
da2 = z(10,:);
phi = z(11,:);
dphi = z(12,:);
leg_desired = pi/4; %desired leg position in flight, and partial stance
spine_desired = pi/4;
max_torque = 2.;

if iphase == 1 || iphase==2 || iphase==3              % during flight, or partial stance
    leg_1_torque = min(max_torque, 1.*(leg_desired-a1) - .1*(da1) ); %proportional with damping, clipped to max_torque
    leg_2_torque = min(max_torque, 1.*(leg_desired-a2) - .1*(da2) ); %proportional with damping, clipped to max_torque
    spine_torque = min(max_torque, 1.*(spine_desired-phi) - .1*(dphi) ); %proportional with damping, clipped to max_torque
else                % during full stance
    leg_1_torque = ( max_torque + 10*heaviside(-a1+.01) ); %exert maximum effort, and impose limit to prevent falling through the floor.
    leg_2_torque = ( max_torque + 10*heaviside(-a2+.01) ); %exert maximum effort, and impose limit to prevent falling through the floor.
    spine_torque = min(max_torque, 1.*(spine_desired-phi) - .1*(dphi) ); %proportional with damping, clipped to max_torque
end
u = [leg_1_torque; leg_2_torque; spine_torque];
end
