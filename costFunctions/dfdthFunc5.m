function dfdth = dfdthFunc5(in1,in2,in3,effOffset,in5)
%DFDTHFUNC5
%    DFDTH = DFDTHFUNC5(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 17:00:33

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
t2 = th1+th2+th3+th4;
t3 = th1+th2;
t4 = th1+th2+th3+th4+th5;
t5 = th1+th2+th3;
t6 = effOffset+th1+th2+th3+th4+th5;
t7 = cos(t6);
t8 = sin(thd);
t9 = sin(t6);
t10 = cos(thd);
t11 = t7.*t8;
t38 = t9.*t10;
t12 = t11-t38;
t13 = t7.*t10;
t14 = t8.*t9;
t15 = cos(t2);
t16 = L4.*t15;
t17 = cos(t3);
t18 = L2.*t17;
t19 = cos(th1);
t20 = L1.*t19;
t21 = cos(t4);
t22 = L5.*t21;
t23 = cos(t5);
t24 = L3.*t23;
t25 = sin(t2);
t26 = L4.*t25;
t27 = sin(t3);
t28 = L2.*t27;
t29 = sin(th1);
t30 = L1.*t29;
t31 = sin(t4);
t32 = L5.*t31;
t33 = sin(t5);
t34 = L3.*t33;
t35 = t16+t18+t20+t22+t24+xb-xd;
t36 = t26+t28+t30+t32+t34+yb-yd;
t37 = t13+t14;
t39 = t13+t14-1.0;
t40 = t12.*t39.*4.0;
dfdth = [t40-t12.*t37.*4.0+t36.*(t16+t18+t20+t22+t24).*2.0-t35.*(t26+t28+t30+t32+t34).*2.0,t40+t36.*(t16+t18+t22+t24).*2.0-t35.*(t26+t28+t32+t34).*2.0-t12.*t37.*4.0,t40-t12.*t37.*4.0+t36.*(t16+t22+t24).*2.0-t35.*(t26+t32+t34).*2.0,t40+t36.*(t16+t22).*2.0-t35.*(t26+t32).*2.0-t12.*t37.*4.0,t40-t12.*t37.*4.0+L5.*t21.*t36.*2.0-L5.*t31.*t35.*2.0];
