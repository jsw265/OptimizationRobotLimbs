function ddfddeffOffset = ddfddeffOffsetFunc6(in1,in2,in3,effOffset,in5)
%DDFDDEFFOFFSETFUNC6
%    DDFDDEFFOFFSET = DDFDDEFFOFFSETFUNC6(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    26-Nov-2016 14:24:04

th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
th6 = in1(6,:);
thd = in5(3,:);
t2 = th1+th2+th3+th4+th5+th6;
t3 = cos(t2);
t4 = sin(effOffset);
t5 = sin(t2);
t6 = cos(effOffset);
t7 = cos(thd);
t8 = t3.*t6;
t16 = t4.*t5;
t9 = t8-t16;
t10 = t7.*t9;
t11 = sin(thd);
t12 = t3.*t4;
t13 = t5.*t6;
t14 = t12+t13;
t15 = t11.*t14;
t17 = t10+t15;
ddfddeffOffset = t17.*(t10+t15-1.0).*-4.0+t17.^2.*4.0;