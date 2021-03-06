function dfdth = dfdthFunc4(in1,in2,in3,effOffset,in5)
%DFDTHFUNC4
%    DFDTH = DFDTHFUNC4(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 18:59:15

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
L4 = in2(4,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
thd = in5(3,:);
xb = in3(1,:);
xd = in5(1,:);
yb = in3(2,:);
yd = in5(2,:);
t2 = th1+th2+th3+th4;
t3 = th1+th2;
t4 = th1+th2+th3;
t5 = cos(t2);
t6 = L4.*t5;
t7 = cos(t3);
t8 = L2.*t7;
t9 = cos(th1);
t10 = L1.*t9;
t11 = cos(t4);
t12 = L3.*t11;
t13 = sin(t2);
t14 = L4.*t13;
t15 = sin(t3);
t16 = L2.*t15;
t17 = sin(th1);
t18 = L1.*t17;
t19 = sin(t4);
t20 = L3.*t19;
t21 = sin(effOffset);
t22 = cos(effOffset);
t23 = cos(thd);
t24 = t5.*t21;
t25 = t13.*t22;
t26 = t24+t25;
t27 = sin(thd);
t28 = t5.*t22;
t31 = t13.*t21;
t29 = t28-t31;
t30 = t23.*t26;
t38 = t27.*t29;
t32 = t30-t38;
t33 = t23.*t29;
t34 = t26.*t27;
t35 = t6+t8+t10+t12+xb-xd;
t36 = t14+t16+t18+t20+yb-yd;
t37 = t33+t34;
t39 = t32.*t37.*4.0;
t40 = t33+t34-1.0;
dfdth = [t39+t36.*(t6+t8+t10+t12).*2.0-t35.*(t14+t16+t18+t20).*2.0-t32.*t40.*4.0,t39-t32.*t40.*4.0+t36.*(t6+t8+t12).*2.0-t35.*(t14+t16+t20).*2.0,t39+t36.*(t6+t12).*2.0-t35.*(t14+t20).*2.0-t32.*t40.*4.0,t39-t32.*t40.*4.0+L4.*t5.*t36.*2.0-L4.*t13.*t35.*2.0];
