function R = z2R_bounding(in1,in2)
%Z2R_BOUNDING
%    R = Z2R_BOUNDING(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    26-Oct-2013 03:13:21

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
t8 = mtd.*phi.*t3.*(1.0./5.0);
t9 = t8+th;
t10 = mtd.*phi.*t3.*(2.0./5.0);
t11 = t10+th;
t12 = mtd.*phi.*t3.*(3.0./5.0);
t13 = t12+th;
t14 = mtd.*phi.*t3.*(4.0./5.0);
t15 = t14+th;
t16 = cos(t2);
t17 = ll.*t16;
t18 = a2-ma2-t6-th;
t19 = a1+ma1+th;
t20 = a2+ma2+t6+th;
t21 = cos(t7);
t22 = slength.*t21.*(1.0./6.0);
t23 = cos(t9);
t24 = slength.*t23.*(1.0./6.0);
t25 = cos(t11);
t26 = slength.*t25.*(1.0./6.0);
t27 = cos(t13);
t28 = slength.*t27.*(1.0./6.0);
t29 = cos(t15);
t30 = slength.*t29.*(1.0./6.0);
t31 = sin(t19);
t32 = sin(t20);
t33 = ll.*t32;
t34 = sin(t7);
t35 = slength.*t34.*(1.0./6.0);
t36 = sin(t9);
t37 = slength.*t36.*(1.0./6.0);
t38 = sin(t11);
t39 = slength.*t38.*(1.0./6.0);
t40 = sin(t13);
t41 = slength.*t40.*(1.0./6.0);
t42 = sin(t15);
t43 = slength.*t42.*(1.0./6.0);
t44 = cos(t19);
t45 = ll.*t44;
t46 = cos(t20);
t47 = sin(th);
t48 = cos(th);
t49 = sep.*t48.*(1.0./2.0);
t50 = cos(t8);
t51 = sin(t8);
t52 = t47.*t50;
t53 = t48.*t51;
t54 = t52+t53;
t55 = t47.*t51;
t56 = cos(t10);
t57 = sin(t10);
t58 = t47.*t56;
t59 = t48.*t57;
t60 = t58+t59;
t61 = t47.*t57;
t62 = cos(t12);
t63 = sin(t12);
t64 = t47.*t62;
t65 = t48.*t63;
t66 = t64+t65;
t67 = t47.*t63;
t68 = cos(t14);
t69 = sin(t14);
t70 = t47.*t68;
t71 = t48.*t69;
t72 = t70+t71;
t73 = t47.*t69;
t74 = cos(t6);
t75 = sin(t6);
t76 = t47.*t74;
t77 = t48.*t75;
t78 = t76+t77;
t79 = t47.*t75;
R = [x;y;t5+x;t17+y;t5+t22+t24+t26+t28+t30+t33+x-ll.*t31-ll.*sin(t18);t17+t35+t37+t39+t41+t43+t45+y-ll.*t46-ll.*cos(t18);t5+t22+t24+t26+t28+t30+t33+x-ll.*t31;t17+t35+t37+t39+t41+t43+t45+y-ll.*t46;t5+x-ll.*t31;t17+t45+y;t5+t24+x-ll.*t31;t17+t37+t45+y;t5+t24+t26+x-ll.*t31;t17+t37+t39+t45+y;t5+t24+t26+t28+x-ll.*t31;t17+t37+t39+t41+t45+y;t5+t24+t26+t28+t30+x-ll.*t31;t17+t37+t39+t41+t43+t45+y;t5+t22+t24+t26+t28+t30+x-ll.*t31;t17+t35+t37+t39+t41+t43+t45+y;t5+x-ll.*t31-sep.*t47.*(1.0./2.0);t17+t45+t49+y;t5+x-ll.*t31+sep.*t47.*(1.0./2.0);t17+t45-t49+y;t5+t24+x-ll.*t31-sep.*t54.*(1.0./2.0);t17+t37+t45+y-sep.*(t55-t48.*t50).*(1.0./2.0);t5+t24+x-ll.*t31+sep.*t54.*(1.0./2.0);t17+t37+t45+y+sep.*(t55-t48.*t50).*(1.0./2.0);t5+t24+t26+x-ll.*t31-sep.*t60.*(1.0./2.0);t17+t37+t39+t45+y-sep.*(t61-t48.*t56).*(1.0./2.0);t5+t24+t26+x-ll.*t31+sep.*t60.*(1.0./2.0);t17+t37+t39+t45+y+sep.*(t61-t48.*t56).*(1.0./2.0);t5+t24+t26+t28+x-ll.*t31-sep.*t66.*(1.0./2.0);t17+t37+t39+t41+t45+y-sep.*(t67-t48.*t62).*(1.0./2.0);t5+t24+t26+t28+x-ll.*t31+sep.*t66.*(1.0./2.0);t17+t37+t39+t41+t45+y+sep.*(t67-t48.*t62).*(1.0./2.0);t5+t24+t26+t28+t30+x-ll.*t31-sep.*t72.*(1.0./2.0);t17+t37+t39+t41+t43+t45+y-sep.*(t73-t48.*t68).*(1.0./2.0);t5+t24+t26+t28+t30+x-ll.*t31+sep.*t72.*(1.0./2.0);t17+t37+t39+t41+t43+t45+y+sep.*(t73-t48.*t68).*(1.0./2.0);t5+t22+t24+t26+t28+t30+x-ll.*t31-sep.*t78.*(1.0./2.0);t17+t35+t37+t39+t41+t43+t45+y-sep.*(t79-t48.*t74).*(1.0./2.0);t5+t22+t24+t26+t28+t30+x-ll.*t31+sep.*t78.*(1.0./2.0);t17+t35+t37+t39+t41+t43+t45+y+sep.*(t79-t48.*t74).*(1.0./2.0)];
