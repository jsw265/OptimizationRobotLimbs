function f = fFunc4(in1,in2,in3,effOffset,in5)
%FFUNC4
%    F = FFUNC4(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 17:00:25

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
t10 = th1+th2+th3+th4;
t11 = th1+th2;
t12 = th1+th2+th3;
t2 = yb-yd+L2.*sin(t11)+L4.*sin(t10)+L3.*sin(t12)+L1.*sin(th1);
t3 = effOffset+th1+th2+th3+th4;
t5 = cos(thd);
t6 = cos(t3);
t7 = sin(t3);
t8 = sin(thd);
t4 = -t5.*t7+t6.*t8;
t9 = t5.*t6+t7.*t8-1.0;
t13 = xb-xd+L2.*cos(t11)+L4.*cos(t10)+L3.*cos(t12)+L1.*cos(th1);
f = t2.^2+t4.^2.*2.0+t9.^2.*2.0+t13.^2;
