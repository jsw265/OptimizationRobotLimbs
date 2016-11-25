function dfdl = dfdlFunc4(in1,in2,in3,effOffset,in5)
%DFDLFUNC4
%    DFDL = DFDLFUNC4(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 17:00:26

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
L4 = in2(4,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
xb = in3(1,:);
xd = in5(1,:);
yb = in3(2,:);
yd = in5(2,:);
t2 = sin(th1);
t3 = th1+th2+th3+th4;
t4 = th1+th2;
t5 = cos(th1);
t6 = th1+th2+th3;
t7 = cos(t4);
t8 = cos(t3);
t9 = L4.*t8;
t10 = L2.*t7;
t11 = L1.*t5;
t12 = cos(t6);
t13 = L3.*t12;
t14 = t9+t10+t11+t13+xb-xd;
t15 = sin(t4);
t16 = sin(t3);
t17 = L4.*t16;
t18 = L2.*t15;
t19 = L1.*t2;
t20 = sin(t6);
t21 = L3.*t20;
t22 = t17+t18+t19+t21+yb-yd;
dfdl = [t5.*t14.*2.0+t2.*t22.*2.0,t7.*t14.*2.0+t15.*t22.*2.0,t12.*t14.*2.0+t20.*t22.*2.0,t8.*t14.*2.0+t16.*t22.*2.0];
