function dfdth = dfdthFunc2(in1,in2,in3,in4)
%DFDTHFUNC2
%    DFDTH = DFDTHFUNC2(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Nov-2016 09:25:28

L1 = in2(1,:);
L2 = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
thd = in4(3,:);
xb = in3(1,:);
xd = in4(1,:);
yb = in3(2,:);
yd = in4(2,:);
t2 = th1+th2;
t3 = cos(t2);
t4 = L2.*t3;
t5 = cos(th1);
t6 = L1.*t5;
t7 = sin(t2);
t8 = L2.*t7;
t9 = sin(th1);
t10 = L1.*t9;
t11 = sin(thd);
t12 = cos(thd);
t13 = t3.*t11;
t18 = t7.*t12;
t14 = t13-t18;
t15 = t7.*t11;
t16 = t3.*t12;
t17 = t15+t16;
t19 = t15+t16-1.0;
t20 = t14.*t19.*4.0;
t21 = t8+t10+yb-yd;
t22 = t4+t6+xb-xd;
dfdth = [t20+t21.*(t4+t6).*2.0-t22.*(t8+t10).*2.0-t14.*t17.*4.0,t20-t14.*t17.*4.0+L2.*t3.*t21.*2.0-L2.*t7.*t22.*2.0];
