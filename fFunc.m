function f = fFunc(in1,in2,in3,in4)
%FFUNC
%    F = FFUNC(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    28-Oct-2016 16:41:47

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
thd = in4(3,:);
xb = in3(1,:);
xd = in4(1,:);
yb = in3(2,:);
yd = in4(2,:);
t2 = th1+th2+th3;
t4 = cos(t2);
t5 = cos(thd);
t6 = sin(t2);
t7 = sin(thd);
t3 = t4.*t7-t5.*t6;
t8 = t4.*t5+t6.*t7-1.0;
t10 = th1+th2;
t9 = xb-xd+L3.*t4+L2.*cos(t10)+L1.*cos(th1);
t11 = yb-yd+L3.*t6+L2.*sin(t10)+L1.*sin(th1);
f = t3.^2.*2.0+t8.^2.*2.0+t9.^2+t11.^2;