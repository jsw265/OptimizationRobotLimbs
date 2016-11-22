function df_torquedth = df_torquedthFunc2(in1,in2,in3)
%DF_TORQUEDTHFUNC2
%    DF_TORQUEDTH = DF_TORQUEDTHFUNC2(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Nov-2016 10:04:28

L1 = in2(1,:);
L2 = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
yb = in3(2,:);
t2 = conj(th1);
t3 = conj(th2);
t4 = t2+t3;
t5 = cos(t2);
t6 = L1.*(1.7e1./4.0e1);
t7 = t6+9.0./2.5e1;
t8 = L1.*(9.0./2.5e1);
t9 = L1.^2;
t10 = t9.*(1.7e1./8.0e1);
t11 = t8+t10;
t12 = yb.*4.16925;
t13 = L1.*4.16925;
t14 = t13+3.5316;
t15 = sin(t2);
t16 = 1.0./t7.^2;
t17 = 1.0./t7;
t18 = cos(t4);
t19 = sin(t4);
t20 = L2.^2;
t21 = L2.*t18.*(1.0./2.0);
t22 = L1.*t5;
t23 = t21+t22;
t24 = L2.*t23.*4.16925;
t25 = t5.*t11.*t14.*t17;
t26 = t24+t25;
t27 = L2.*t19.*4.16925;
t28 = L1.*t15.*4.16925;
t29 = t12+t27+t28;
t30 = t20.^2;
df_torquedth = [(L2.*t5.*4.16925+t14.*(t5-t5.*t11.*t16.*(1.7e1./4.0e1))+t5.*t11.*t17.*4.16925).*(t12+L2.*t15.*4.16925+t14.*(t15-t11.*t15.*t16.*(1.7e1./4.0e1))+t11.*t15.*t17.*4.16925).*2.0+t29.*(L1.*t5.*4.16925+L2.*t18.*4.16925).*2.0-t26.*(L2.*(L1.*t15+L2.*t19.*(1.0./2.0)).*4.16925+t11.*t14.*t15.*t17).*2.0-t18.*t19.*t30.*8.69132278125,L2.*t18.*t29.*8.3385-t19.*t20.*t26.*4.16925-t18.*t19.*t30.*8.69132278125];