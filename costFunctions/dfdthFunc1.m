function dfdth = dfdthFunc1(th1,L1,in3,effOffset,in5)
%DFDTHFUNC1
%    DFDTH = DFDTHFUNC1(TH1,L1,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 18:59:01

thd = in5(3,:);
xb = in3(1,:);
xd = in5(1,:);
yb = in3(2,:);
yd = in5(2,:);
t2 = cos(effOffset);
t3 = sin(th1);
t4 = cos(th1);
t5 = sin(effOffset);
t6 = cos(thd);
t7 = t2.*t3;
t8 = t4.*t5;
t9 = t7+t8;
t10 = sin(thd);
t11 = t2.*t4;
t14 = t3.*t5;
t12 = t11-t14;
t13 = t6.*t9;
t15 = t13-t10.*t12;
t16 = t6.*t12;
t17 = t9.*t10;
dfdth = t15.*(t16+t17).*4.0-t15.*(t16+t17-1.0).*4.0-L1.*t3.*(xb-xd+L1.*t4).*2.0+L1.*t4.*(yb-yd+L1.*t3).*2.0;
