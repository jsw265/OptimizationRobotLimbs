function dfdth = dfdthFunc1(th1,L1,in3,effOffset,in5)
%DFDTHFUNC1
%    DFDTH = DFDTHFUNC1(TH1,L1,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 17:00:12

thd = in5(3,:);
xb = in3(1,:);
xd = in5(1,:);
yb = in3(2,:);
yd = in5(2,:);
t2 = effOffset+th1;
t3 = cos(t2);
t4 = sin(thd);
t5 = sin(t2);
t6 = cos(thd);
t7 = t3.*t4;
t8 = t7-t5.*t6;
t9 = t3.*t6;
t10 = t4.*t5;
t11 = cos(th1);
t12 = sin(th1);
dfdth = t8.*(t9+t10).*-4.0+t8.*(t9+t10-1.0).*4.0-L1.*t12.*(xb-xd+L1.*t11).*2.0+L1.*t11.*(yb-yd+L1.*t12).*2.0;
