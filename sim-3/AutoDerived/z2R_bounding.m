function R = z2R_bounding(in1,in2)
%Z2R_BOUNDING
%    R = Z2R_BOUNDING(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    17-Nov-2013 21:04:16

a1 = in1(7,:);
a2 = in1(9,:);
ll = in2(1,:);
ma1 = in2(19,:);
ma2 = in2(20,:);
mtd = in2(18,:);
phi = in1(11,:);
sep = in2(17,:);
slength = in2(14,:);
th = in1(5,:);
x = in1(1,:);
y = in1(3,:);
t2 = a1-ma1-th;
t3 = 1.0./sep;
t4 = sin(t2);
t5 = ll.*t4;
t6 = mtd.*phi.*t3;
t7 = t6+th;
t8 = mtd.*phi.*t3.*(1.0./3.0);
t9 = t8+th;
t10 = mtd.*phi.*t3.*(2.0./3.0);
t11 = t10+th;
t12 = cos(t2);
t13 = ll.*t12;
t14 = a2-ma2-t6-th;
t15 = a1+ma1+th;
t16 = a2+ma2+t6+th;
t17 = cos(t7);
t18 = slength.*t17.*(1.0./4.0);
t19 = cos(t9);
t20 = slength.*t19.*(1.0./4.0);
t21 = cos(t11);
t22 = slength.*t21.*(1.0./4.0);
t23 = sin(t15);
t24 = sin(t16);
t25 = ll.*t24;
t26 = sin(t7);
t27 = slength.*t26.*(1.0./4.0);
t28 = sin(t9);
t29 = slength.*t28.*(1.0./4.0);
t30 = sin(t11);
t31 = slength.*t30.*(1.0./4.0);
t32 = cos(t15);
t33 = ll.*t32;
t34 = cos(t16);
t35 = sin(th);
t36 = cos(th);
t37 = sep.*t36.*(1.0./2.0);
t38 = slength.*t19.*(1.0./8.0);
t39 = sep.*t19.*(1.0./2.0);
t40 = slength.*t28.*(1.0./8.0);
t41 = slength.*t21.*(1.0./8.0);
t42 = sep.*t21.*(1.0./2.0);
t43 = slength.*t30.*(1.0./8.0);
t44 = slength.*t17.*(1.0./8.0);
t45 = sep.*t17.*(1.0./2.0);
t46 = slength.*t26.*(1.0./8.0);
R = [x;y;t5+x;t13+y;t5+t18+t20+t22+t25+x-ll.*t23-ll.*sin(t14);t13+t27+t29+t31+t33+y-ll.*t34-ll.*cos(t14);t5+t18+t20+t22+t25+x-ll.*t23;t13+t27+t29+t31+t33+y-ll.*t34;t5+x-ll.*t23;t13+t33+y;t5+t20+x-ll.*t23;t13+t29+t33+y;t5+t20+t22+x-ll.*t23;t13+t29+t31+t33+y;t5+t18+t20+t22+x-ll.*t23;t13+t27+t29+t31+t33+y;t5+x-ll.*t23-sep.*t35.*(1.0./2.0);t13+t33+t37+y;t5+x-ll.*t23+sep.*t35.*(1.0./2.0);t13+t33-t37+y;t5+t38+x-ll.*t23-sep.*t28.*(1.0./2.0);t13+t33+t39+t40+y;t5+t38+x-ll.*t23+sep.*t28.*(1.0./2.0);t13+t33-t39+t40+y;t5+t20+t41+x-ll.*t23-sep.*t30.*(1.0./2.0);t13+t29+t33+t42+t43+y;t5+t20+t41+x-ll.*t23+sep.*t30.*(1.0./2.0);t13+t29+t33-t42+t43+y;t5+t20+t22+t44+x-ll.*t23-sep.*t26.*(1.0./2.0);t13+t29+t31+t33+t45+t46+y;t5+t20+t22+t44+x-ll.*t23+sep.*t26.*(1.0./2.0);t13+t29+t31+t33-t45+t46+y];