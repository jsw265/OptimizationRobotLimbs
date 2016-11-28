function ddf_torqueddl = ddf_torqueddlFunc2(in1,in2,in3)
%DDF_TORQUEDDLFUNC2
%    DDF_TORQUEDDL = DDF_TORQUEDDLFUNC2(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    23-Nov-2016 12:02:52

L1 = in2(1,:);
L2 = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
yb = in3(2,:);
t2 = conj(th1);
t3 = cos(t2);
t4 = L1.*(1.7e1./4.0e1);
t5 = t4+9.0./2.5e1;
t6 = 1.0./t5;
t7 = L1.*4.16925;
t8 = t7+3.5316;
t9 = L1.*(9.0./2.5e1);
t10 = L1.^2;
t11 = t10.*(1.7e1./8.0e1);
t12 = t9+t11;
t13 = 1.0./t5.^2;
t33 = t3.*t8;
t34 = L2.*t3.*4.16925;
t35 = t3.*t6.*t12.*4.16925;
t36 = t3.*t8.*t12.*t13.*(1.7e1./4.0e1);
t14 = t33+t34+t35-t36;
t15 = sin(t2);
t16 = 1.0./t5.^3;
t27 = t15.*8.3385;
t28 = t6.*t15.*(1.7e1./4.0e1);
t29 = t12.*t15.*t16.*(2.89e2./8.0e2);
t30 = t28-t29;
t31 = t8.*t30;
t32 = t12.*t13.*t15.*3.5438625;
t17 = -t27+t31+t32;
t18 = conj(th2);
t19 = t2+t18;
t20 = cos(t19);
t21 = L2.*t20.*(1.0./2.0);
t22 = L1.*t3;
t23 = t21+t22;
t24 = L2.*t23.*4.16925;
t25 = t3.*t6.*t8.*t12;
t26 = t24+t25;
t37 = sin(t19);
t38 = t15.*t37.*3.4765291125e1;
t39 = t3.*t26.*8.3385;
t40 = L2.*t20.*4.16925;
t41 = L1.*t3.*4.16925;
t42 = t40+t41;
t43 = t14.*t42.*2.0;
t44 = t38+t39+t43-t15.*t17.*8.3385;
t45 = t15.^2;
t46 = t45.*3.4765291125e1;
ddf_torqueddl = reshape([t46+t26.*(t3.*8.3385-t3.*t6.*t8.*(1.7e1./4.0e1)-t3.*t12.*t13.*3.5438625+t3.*t8.*t12.*t16.*(2.89e2./8.0e2)).*2.0+(t6.*t15.*(-5.31579375)+t8.*(t13.*t15.*5.41875e-1-1.0./t5.^4.*t12.*t15.*4.6059375e-1)+t12.*t15.*t16.*4.5184246875).*(yb.*4.16925+L2.*t15.*4.16925+t8.*(t15-t12.*t13.*t15.*(1.7e1./4.0e1))+t6.*t12.*t15.*4.16925).*2.0+t14.^2.*2.0+t17.^2.*2.0,t44,t44,t46+t20.*t26.*8.3385+t37.^2.*3.4765291125e1+t42.^2.*2.0+L2.^2.*t20.^2.*5.21479366875e1],[2,2]);
