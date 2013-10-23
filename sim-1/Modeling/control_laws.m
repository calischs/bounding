function u = control_laws(t,z,p,iphase)
%phases:
% 1: no feet in contact
% 2: rear foot in contact only
% 3: front foot in contact only
% 4: both feet in contact

y = z(1,:);
th = z(3,:);
dth = z(4,:);
th_desired = pi/4; %desired position in flight, and partial stance
max_torque = 2.;

if iphase == 1 || iphase==2 || iphase==3              % during flight, or partial stance
    u = min(max_torque, 1.*(th_desired-th) - .1*(dth) ); %proportional with damping, clipped to max_torque
else                % during full stance
    u = ( max_torque + 10*heaviside(-th+.01) ); %exert maximum effort, and impose limit to prevent falling through the floor.
end

end
