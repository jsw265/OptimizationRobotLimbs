function f = fFunc1(th1,L1,in3,in4)
%FFUNC1
%    F = FFUNC1(TH1,L1,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Nov-2016 09:25:26

thd = in4(3,:);
xb = in3(1,:);
xd = in4(1,:);
yb = in3(2,:);
yd = in4(2,:);
t4 = cos(th1);
t2 = xb-xd+L1.*t4;
t5 = sin(th1);
t3 = yb-yd+L1.*t5;
t7 = cos(thd);
t8 = sin(thd);
t6 = -t4.*t8+t5.*t7;
t9 = t4.*t7+t5.*t8-1.0;
f = t2.^2+t3.^2+t6.^2.*2.0+t9.^2.*2.0;