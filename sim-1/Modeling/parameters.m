function p = parameters() 
 l = .25;           % each segments .25m for total leg length 1/2 human scale
 m1  = .4*l;        % assume ABS ~1g/cm^3, 1.25 cm x 3 cm cross section
 m2 = .4*l;
 mh = .1;           % assume motor ~half weight of ours
 I1 = m1*l^2/12;    % even mass distribution
 I2 = m2*l^2/12;
 c1 = l/2;          % even mass distribution
 c2 = l/2;
 g = 9.81;          % earth gravity
 num_sps = 8;       %number of spine points, including hip and shoulder
 p = [l; c1; c2; m1; m2; mh; I1; I2; g; num_sps];
end