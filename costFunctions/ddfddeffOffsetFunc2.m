function ddfddeffOffset = ddfddeffOffsetFunc2(in1,in2,in3,effOffset,in5)
%DDFDDEFFOFFSETFUNC2
%    DDFDDEFFOFFSET = DDFDDEFFOFFSETFUNC2(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 18:59:06

th1 = in1(1,:);
th2 = in1(2,:);
thd = in5(3,:);
t2 = th1+th2;
t3 = cos(t2);
t4 = sin(effOffset);
t5 = sin(t2);
t6 = cos(effOffset);
t8 = cos(thd);
t9 = t3.*t6;
t10 = t4.*t5;
t11 = t9-t10;
t12 = t8.*t11;
t13 = sin(thd);
t14 = t3.*t4;
t15 = t5.*t6;
t16 = t14+t15;
t17 = t13.*t16;
t7 = t12+t17;
ddfddeffOffset = t7.*(t12+t17-1.0).*-4.0+t7.^2.*4.0;
