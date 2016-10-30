function dfdl = dfdlFunc(in1,in2,in3,in4)
%DFDLFUNC
%    DFDL = DFDLFUNC(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    29-Oct-2016 20:10:14

L1 = in2(1,:);
L2 = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
xb = in3(1,:);
xd = in4(1,:);
yb = in3(2,:);
yd = in4(2,:);
t2 = cos(th1);
t3 = th1+th2;
t4 = sin(th1);
t5 = cos(t3);
t6 = L2.*t5;
t7 = L1.*t2;
t8 = t6+t7+xb-xd;
t9 = sin(t3);
t10 = L2.*t9;
t11 = L1.*t4;
t12 = t10+t11+yb-yd;
dfdl = [t2.*t8.*2.0+t4.*t12.*2.0,t5.*t8.*2.0+t9.*t12.*2.0];
