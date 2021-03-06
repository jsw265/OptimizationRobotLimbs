function dfdth = dfdthFunc3(in1,in2,in3,effOffset,in5)
%DFDTHFUNC3
%    DFDTH = DFDTHFUNC3(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 18:59:08

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
thd = in5(3,:);
xb = in3(1,:);
xd = in5(1,:);
yb = in3(2,:);
yd = in5(2,:);
t2 = th1+th2+th3;
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
t32 = t11.*t13;
t16 = t14-t32;
t17 = t7.*t13;
t18 = t10.*t11;
t19 = th1+th2;
t20 = cos(t19);
t21 = L2.*t20;
t22 = cos(th1);
t23 = L1.*t22;
t24 = L3.*t3;
t25 = sin(t19);
t26 = L2.*t25;
t27 = sin(th1);
t28 = L1.*t27;
t29 = L3.*t5;
t30 = t21+t23+t24+xb-xd;
t31 = t17+t18;
t33 = t16.*t31.*4.0;
t34 = t17+t18-1.0;
t35 = t26+t28+t29+yb-yd;
dfdth = [t33-t16.*t34.*4.0+t35.*(t21+t23+t24).*2.0-t30.*(t26+t28+t29).*2.0,t33+t35.*(t21+t24).*2.0-t30.*(t26+t29).*2.0-t16.*t34.*4.0,t33-t16.*t34.*4.0-L3.*t5.*t30.*2.0+L3.*t3.*t35.*2.0];
