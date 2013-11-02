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
leg_desired_flight = pi/12; %desired leg position in flight, and partial stance
leg_desired_stance = pi/3;
spine_desired = 0;
max_torque = 5.;

%simple control
%if iphase == 1 || iphase==2 || iphase==3 
%    leg_desired = leg_desired_flight;
%else
%    leg_desired = leg_desired_stance;
%end
if rem(t,1)<.25
    leg_desired_1 = leg_desired_stance;
    leg_desired_2 = leg_desired_flight;
elseif rem(t,1)<.5
    leg_desired_1 = leg_desired_stance;
    leg_desired_2 = leg_desired_stance;        
elseif rem(t,1)<.75
    leg_desired_1 = leg_desired_flight;
    leg_desired_2 = leg_desired_stance;  
else
    leg_desired_1 = leg_desired_flight;
    leg_desired_2 = leg_desired_stance;
end

leg_1_torque = min(max_torque, 3.*(leg_desired_1-a1) - .1*(da1) );
leg_2_torque = min(max_torque, 3.*(leg_desired_2-a2) - .1*(da2) );
%spine_torque = min(max_torque, 1.*(spine_desired*sin(2*pi*t)-phi) - .1*(dphi) );
spine_torque = min(max_torque, 1.*(spine_desired-phi) - .1*(dphi) );

u = [leg_1_torque; leg_2_torque; spine_torque];
end
