function dfdeffOffset = dfdeffOffsetFunc4(in1,in2,in3,effOffset,in5)
%DFDEFFOFFSETFUNC4
%    DFDEFFOFFSET = DFDEFFOFFSETFUNC4(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 17:00:27

th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
thd = in5(3,:);
t2 = effOffset+th1+th2+th3+th4;
t3 = sin(t2);
t4 = cos(thd);
t5 = sin(thd);
t6 = cos(t2);
t7 = t3.*t4;
t8 = t7-t5.*t6;
t9 = t4.*t6;
t10 = t3.*t5;
dfdeffOffset = t8.*(t9+t10).*4.0-t8.*(t9+t10-1.0).*4.0;
