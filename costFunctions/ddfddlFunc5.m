function ddfddl = ddfddlFunc5(in1,in2,in3,effOffset,in5)
%DDFDDLFUNC5
%    DDFDDL = DDFDDLFUNC5(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 18:59:32

th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
t2 = cos(th1);
t3 = sin(th1);
t4 = th1+th2;
t5 = th1+th2+th3;
t6 = th1+th2+th3+th4;
t7 = th1+th2+th3+th4+th5;
t8 = sin(t4);
t9 = t3.*t8.*2.0;
t10 = cos(t4);
t11 = t2.*t10.*2.0;
t12 = t9+t11;
t13 = cos(t5);
t14 = sin(t5);
t15 = cos(t6);
t16 = sin(t6);
t17 = cos(t7);
t18 = sin(t7);
t19 = t2.*t13.*2.0;
t20 = t3.*t14.*2.0;
t21 = t19+t20;
t22 = t10.*t13.*2.0;
t23 = t8.*t14.*2.0;
t24 = t22+t23;
t25 = t2.*t15.*2.0;
t26 = t3.*t16.*2.0;
t27 = t25+t26;
t28 = t10.*t15.*2.0;
t29 = t8.*t16.*2.0;
t30 = t28+t29;
t31 = t13.*t15.*2.0;
t32 = t14.*t16.*2.0;
t33 = t31+t32;
t34 = t2.*t17.*2.0;
t35 = t3.*t18.*2.0;
t36 = t34+t35;
t37 = t10.*t17.*2.0;
t38 = t8.*t18.*2.0;
t39 = t37+t38;
t40 = t13.*t17.*2.0;
t41 = t14.*t18.*2.0;
t42 = t40+t41;
t43 = t16.*t18.*2.0;
t44 = t15.*t17.*2.0;
t45 = t43+t44;
ddfddl = reshape([t2.^2.*2.0+t3.^2.*2.0,t12,t21,t27,t36,t12,t8.^2.*2.0+t10.^2.*2.0,t24,t30,t39,t21,t24,t13.^2.*2.0+t14.^2.*2.0,t33,t42,t27,t30,t33,t15.^2.*2.0+t16.^2.*2.0,t45,t36,t39,t42,t45,t17.^2.*2.0+t18.^2.*2.0],[5,5]);
