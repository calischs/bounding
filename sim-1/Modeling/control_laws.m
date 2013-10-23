function u = control_laws(t,z,p,iphase)
y = z(1,:);
th = z(3,:);
dth = z(4,:);
th_desired = pi/4; %desired position in flight
max_torque = 2.;

if iphase == 2              % during flight
    u = min(max_torque, 1.*(th_desired-th) - .1*(dth) ); %proportional with damping, clipped to max_torque
else                % during stance
    u = ( max_torque + 10*heaviside(-th+.01) ); %exert maximum effort, and impose limit to prevent falling through the floor.
end

end
