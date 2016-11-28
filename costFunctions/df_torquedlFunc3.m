function df_torquedl = df_torquedlFunc3(in1,in2,in3)
%DF_TORQUEDLFUNC3
%    DF_TORQUEDL = DF_TORQUEDLFUNC3(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    23-Nov-2016 12:03:00

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
yb = in3(2,:);
t2 = conj(th1);
t3 = conj(th2);
t4 = sin(t2);
t5 = yb.*4.16925;
t6 = t2+t3;
t7 = sin(t6);
t8 = L1.*t4.*4.16925;
t9 = L2.*(1.7e1./4.0e1);
t10 = t9+9.0./2.5e1;
t11 = L2.*(9.0./2.5e1);
t12 = L2.^2;
t13 = t12.*(1.7e1./8.0e1);
t14 = t11+t13;
t15 = L2.*4.16925;
t16 = t15+3.5316;
t17 = 1.0./t10;
t18 = cos(t6);
t19 = cos(t2);
t20 = L1.*t19;
t21 = conj(th3);
t22 = t2+t3+t21;
t23 = L1.*4.16925;
t24 = t23+3.5316;
t25 = L1.*(1.7e1./4.0e1);
t26 = t25+9.0./2.5e1;
t27 = 1.0./t26;
t28 = L1.*(9.0./2.5e1);
t29 = L1.^2;
t30 = t29.*(1.7e1./8.0e1);
t31 = t28+t30;
t32 = 1.0./t26.^2;
t33 = t14.*t17.*t18;
t34 = t20+t33;
t35 = t16.*t34;
t36 = L2.*t18;
t37 = cos(t22);
t38 = L3.*t37.*(1.0./2.0);
t39 = t20+t36+t38;
t40 = L3.*t39.*4.16925;
t41 = t19.*t24.*t27.*t31;
t42 = t35+t40+t41;
t43 = 1.0./t10.^2;
t44 = L3.*t18.*4.16925;
t45 = t14.*t17.*t18.*4.16925;
t46 = sin(t22);
t47 = L3.*t46.*4.16925;
t48 = L2.*t7.*4.16925;
t49 = t5+t8+t47+t48;
t50 = L3.*t7.*4.16925;
t62 = t7.*t14.*t43.*(1.7e1./4.0e1);
t51 = t7-t62;
t52 = t16.*t51;
t53 = t7.*t14.*t17.*4.16925;
t54 = t5+t8+t50+t52+t53;
t70 = t4.*t31.*t32.*(1.7e1./4.0e1);
t55 = t4-t70;
t56 = t24.*t55;
t57 = t4.*t16;
t58 = L3.*t4.*4.16925;
t59 = t4.*t27.*t31.*4.16925;
t60 = t5+t56+t57+t58+t59;
t61 = L1.*t19.*4.16925;
t63 = L2.*t18.*4.16925;
t64 = L3.*t37.*4.16925;
t65 = t36+t38;
t66 = L3.*t65.*4.16925;
t67 = t14.*t16.*t17.*t18;
t68 = t66+t67;
t69 = L3.^2;
t71 = t4.*t60.*8.3385;
df_torquedl = [t4.*t49.*8.3385+t4.*t54.*8.3385-t60.*(t4.*(-8.3385)+t24.*(t4.*t27.*(1.7e1./4.0e1)-t4.*1.0./t26.^3.*t31.*(2.89e2./8.0e2))+t4.*t31.*t32.*3.5438625).*2.0+t42.*(L3.*t19.*4.16925+t16.*t19+t19.*t24+t19.*t27.*t31.*4.16925-t19.*t24.*t31.*t32.*(1.7e1./4.0e1)).*2.0,t71+t7.*t49.*8.3385-t54.*(t7.*(-8.3385)+t16.*(t7.*t17.*(1.7e1./4.0e1)-t7.*1.0./t10.^3.*t14.*(2.89e2./8.0e2))+t7.*t14.*t43.*3.5438625).*2.0+t68.*(t44+t45+t16.*t18-t14.*t16.*t18.*t43.*(1.7e1./4.0e1)).*2.0+t42.*(t44+t45+t61+t16.*(t18-t14.*t18.*t43.*(1.7e1./4.0e1))).*2.0,t71+t68.*(t63+t64).*2.0+t7.*t54.*8.3385+t46.*t49.*8.3385+t42.*(t61+t63+t64).*2.0+L3.*t37.^2.*t69.*1.73826455625e1];
