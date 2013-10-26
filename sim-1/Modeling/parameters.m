function p = parameters() 
 l = .25;           % each segments .25m for total leg length 1/2 human scale
 m11  = .4*l;        % assume ABS ~1g/cm^3, 1.25 cm x 3 cm cross section
 m12 = .4*l;
 m21 = .4*l;
 m22 = .4*l;
 mh = .1;           % assume motor ~half weight of ours
 I11 = m11*l^2/12;    % even mass distribution
 I12 = m12*l^2/12;
 I21 = m21*l^2/12;
 I22 = m22*l^2/12;
 c11 = l/2;          % even mass distribution
 c12 = l/2;          % even mass distribution
 c21 = l/2;
 c22 = l/2;
 ls = .5;       %spine length
 kappa = 0*.1;    %spine torsion stiffness constant
 ms = .4*ls;    %spine mass
 sep = .1;      %separation of tendons
 mtd = .05;     %motor takeup diameter
 ma1 = 0;       %0 is perpendicular mounting angle
 ma2 = 0;
 g = 9.81;          % earth gravity
 num_sps = 8;       %number of spine points, including hip and shoulder
 p = [l;...
     c11; c12; c21; c22;...
     m11; m12; m21; m22;...
     I11; I12; I21; I22;...
     ls; kappa; ms; sep; mtd; ma1; ma2;...
     mh; g; num_sps];
end