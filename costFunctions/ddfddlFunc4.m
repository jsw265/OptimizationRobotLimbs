function ddfddl = ddfddlFunc4(in1,in2,in3,in4)
%DDFDDLFUNC4
%    DDFDDL = DDFDDLFUNC4(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Nov-2016 09:25:34

th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = th1+th2+th3;
t6 = th1+th2+th3+th4;
t7 = sin(t4);
t8 = t3.*t7.*2.0;
t9 = cos(t4);
t10 = t2.*t9.*2.0;
t11 = t8+t10;
t12 = cos(t5);
t13 = sin(t5);
t14 = cos(t6);
t15 = sin(t6);
t16 = t2.*t12.*2.0;
t17 = t3.*t13.*2.0;
t18 = t16+t17;
t19 = t9.*t12.*2.0;
t20 = t7.*t13.*2.0;
t21 = t19+t20;
t22 = t2.*t14.*2.0;
t23 = t3.*t15.*2.0;
t24 = t22+t23;
t25 = t9.*t14.*2.0;
t26 = t7.*t15.*2.0;
t27 = t25+t26;
t28 = t12.*t14.*2.0;
t29 = t13.*t15.*2.0;
t30 = t28+t29;
ddfddl = reshape([t2.^2.*2.0+t3.^2.*2.0,t11,t18,t24,t11,t7.^2.*2.0+t9.^2.*2.0,t21,t27,t18,t21,t12.^2.*2.0+t13.^2.*2.0,t30,t24,t27,t30,t14.^2.*2.0+t15.^2.*2.0],[4,4]);