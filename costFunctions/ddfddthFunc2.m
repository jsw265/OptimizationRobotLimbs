function ddfddth = ddfddthFunc2(in1,in2,in3,effOffset,in5)
%DDFDDTHFUNC2
%    DDFDDTH = DDFDDTHFUNC2(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 18:59:06

L1 = in2(1,:);
L2 = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
thd = in5(3,:);
xb = in3(1,:);
xd = in5(1,:);
yb = in3(2,:);
yd = in5(2,:);
t2 = th1+th2;
t3 = cos(t2);
t4 = L2.*t3;
t5 = cos(th1);
t6 = L1.*t5;
t7 = sin(t2);
t8 = L2.*t7;
t9 = sin(th1);
t10 = L1.*t9;
t11 = sin(effOffset);
t12 = cos(effOffset);
t15 = cos(thd);
t16 = t3.*t12;
t17 = t7.*t11;
t18 = t16-t17;
t19 = t15.*t18;
t20 = sin(thd);
t21 = t3.*t11;
t22 = t7.*t12;
t23 = t21+t22;
t24 = t20.*t23;
t13 = t19+t24;
t14 = t4+t6;
t25 = t8+t10;
t26 = t13.^2;
t27 = t26.*4.0;
t28 = t19+t24-1.0;
t29 = t8+t10+yb-yd;
t30 = t4+t6+xb-xd;
t31 = L2.*t3.*t14.*2.0;
t32 = L2.*t7.*t25.*2.0;
t35 = t13.*t28.*4.0;
t36 = L2.*t7.*t29.*2.0;
t37 = L2.*t3.*t30.*2.0;
t33 = t27+t31+t32-t35-t36-t37;
t34 = L2.^2;
ddfddth = reshape([t27-t13.*t28.*4.0-t14.*t30.*2.0-t25.*t29.*2.0+t14.^2.*2.0+t25.^2.*2.0,t33,t33,t27-t35-t36-t37+t3.^2.*t34.*2.0+t7.^2.*t34.*2.0],[2,2]);
