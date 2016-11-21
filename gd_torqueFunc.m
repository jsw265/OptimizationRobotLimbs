function dtau = gd_torqueFunc(in1,in2,in3)
%GD_TORQUEFUNC
%    DTAU = GD_TORQUEFUNC(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    21-Nov-2016 15:47:04

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
t2 = conj(th1);
t3 = conj(th2);
t4 = t2+t3;
t5 = sin(t4);
t6 = sin(t2);
t7 = L1.*t6;
t8 = conj(th3);
t9 = t2+t3+t8;
t10 = sin(t9);
t11 = L3.*t10.*(1.0./2.0);
t12 = L2.*t5;
t13 = L2.*(1.7e1./4.0e1);
t14 = t13+9.0./2.5e1;
t15 = 1.0./t14;
t16 = L2.*4.16925;
t17 = t16+3.5316;
t18 = L2.*(9.0./2.5e1);
t19 = L2.^2;
t20 = t19.*(1.7e1./8.0e1);
t21 = t18+t20;
t22 = L1.*4.16925;
t23 = t22+3.5316;
t24 = cos(t2);
t25 = L1.*(1.7e1./4.0e1);
t26 = t25+9.0./2.5e1;
t27 = 1.0./t26;
t28 = L1.*(9.0./2.5e1);
t29 = L1.^2;
t30 = t29.*(1.7e1./8.0e1);
t31 = t28+t30;
t32 = cos(t4);
t33 = L1.*t24.*4.16925;
t34 = t11+t12;
t36 = L3.*t34.*4.16925;
t37 = t5.*t15.*t17.*t21;
t35 = -t36-t37;
t38 = L3.^2;
t39 = L3.*t32.*4.16925;
t40 = t15.*t21.*t32.*4.16925;
t41 = 1.0./t14.^2;
t42 = L2.*t32.*4.16925;
t43 = cos(t9);
t44 = L3.*t43.*4.16925;
t45 = 1.0./t26.^2;
t46 = t17.*t24;
t47 = L3.*t24.*4.16925;
t48 = t24.*t27.*t31.*4.16925;
t49 = t6.*4.16925;
t53 = t21.*t32.*t41.*(1.7e1./4.0e1);
t50 = t32-t53;
t51 = t17.*t50;
t52 = t33+t39+t40+t51;
t54 = t33+t42+t44;
t55 = t42+t44;
t56 = t5.*4.16925;
dtau = reshape([-t17.*(t7+t5.*t15.*t21)-L3.*(t7+t11+t12).*4.16925-t6.*t23.*t27.*t31,t35,t10.*t38.*(-2.084625),t46+t47+t48+t23.*(t24-t24.*t31.*t45.*(1.7e1./4.0e1)),t52,t54,t35,t35,t10.*t38.*(-2.084625),0.0,t39+t40+t51,t55,t10.*t38.*(-2.084625),t10.*t38.*(-2.084625),t10.*t38.*(-2.084625),0.0,0.0,t44,t46+t47+t48+t23.*t24-t23.*t24.*t31.*t45.*(1.7e1./4.0e1),0.0,0.0,t6.*8.3385-t23.*(t6.*t27.*(1.7e1./4.0e1)-t6.*1.0./t26.^3.*t31.*(2.89e2./8.0e2))-t6.*t31.*t45.*3.5438625,t49,t49,t52,t39+t40+t17.*t32-t17.*t21.*t32.*t41.*(1.7e1./4.0e1),0.0,t49,t5.*8.3385-t17.*(t5.*t15.*(1.7e1./4.0e1)-t5.*1.0./t14.^3.*t21.*(2.89e2./8.0e2))-t5.*t21.*t41.*3.5438625,t56,t54,t55,t44,t49,t56,t10.*4.16925],[6,6]);
