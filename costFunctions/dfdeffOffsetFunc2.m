function dfdeffOffset = dfdeffOffsetFunc2(in1,in2,in3,effOffset,in5)
%DFDEFFOFFSETFUNC2
%    DFDEFFOFFSET = DFDEFFOFFSETFUNC2(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 17:00:16

th1 = in1(1,:);
th2 = in1(2,:);
thd = in5(3,:);
t2 = effOffset+th1+th2;
t3 = cos(t2);
t4 = sin(thd);
t5 = sin(t2);
t6 = cos(thd);
t7 = t3.*t4;
t8 = t7-t5.*t6;
t9 = t3.*t6;
t10 = t4.*t5;
dfdeffOffset = t8.*(t9+t10).*-4.0+t8.*(t9+t10-1.0).*4.0;
