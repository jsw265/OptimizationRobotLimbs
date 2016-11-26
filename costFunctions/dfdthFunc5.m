function dfdth = dfdthFunc5(in1,in2,in3,effOffset,in5)
%DFDTHFUNC5
%    DFDTH = DFDTHFUNC5(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 18:59:24

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
L4 = in2(4,:);
L5 = in2(5,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
thd = in5(3,:);
xb = in3(1,:);
xd = in5(1,:);
yb = in3(2,:);
yd = in5(2,:);
t2 = th1+th2+th3+th4+th5;
t3 = cos(t2);
t4 = sin(effOffset);
t5 = sin(t2);
t6 = cos(effOffset);
t7 = cos(thd);
t8 = t3.*t4;
t9 = t5.*t6;
t10 = t8+t9;
t11 = sin(thd);
t12 = t3.*t6;
t15 = t4.*t5;
t13 = t12-t15;
t14 = t7.*t10;
t43 = t11.*t13;
t16 = t14-t43;
t17 = t7.*t13;
t18 = t10.*t11;
t19 = th1+th2+th3+th4;
t20 = th1+th2;
t21 = th1+th2+th3;
t22 = cos(t19);
t23 = L4.*t22;
t24 = cos(t20);
t25 = L2.*t24;
t26 = cos(th1);
t27 = L1.*t26;
t28 = L5.*t3;
t29 = cos(t21);
t30 = L3.*t29;
t31 = sin(t19);
t32 = L4.*t31;
t33 = sin(t20);
t34 = L2.*t33;
t35 = sin(th1);
t36 = L1.*t35;
t37 = L5.*t5;
t38 = sin(t21);
t39 = L3.*t38;
t40 = t23+t25+t27+t28+t30+xb-xd;
t41 = t32+t34+t36+t37+t39+yb-yd;
t42 = t17+t18;
t44 = t16.*t42.*4.0;
t45 = t17+t18-1.0;
dfdth = [t44-t16.*t45.*4.0+t41.*(t23+t25+t27+t28+t30).*2.0-t40.*(t32+t34+t36+t37+t39).*2.0,t44+t41.*(t23+t25+t28+t30).*2.0-t40.*(t32+t34+t37+t39).*2.0-t16.*t45.*4.0,t44-t16.*t45.*4.0+t41.*(t23+t28+t30).*2.0-t40.*(t32+t37+t39).*2.0,t44+t41.*(t23+t28).*2.0-t40.*(t32+t37).*2.0-t16.*t45.*4.0,t44-t16.*t45.*4.0+L5.*t3.*t41.*2.0-L5.*t5.*t40.*2.0];
