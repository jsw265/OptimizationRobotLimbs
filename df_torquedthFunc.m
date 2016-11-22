function df_torquedth = df_torquedthFunc(in1,in2,in3)
%DF_TORQUEDTHFUNC
%    DF_TORQUEDTH = DF_TORQUEDTHFUNC(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    21-Nov-2016 15:47:05

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
yb = in3(2,:);
t2 = conj(th1);
t3 = conj(th2);
t4 = t2+t3;
t5 = cos(t4);
t6 = L2.*(1.7e1./4.0e1);
t7 = t6+9.0./2.5e1;
t8 = L2.*(9.0./2.5e1);
t9 = L2.^2;
t10 = t9.*(1.7e1./8.0e1);
t11 = t8+t10;
t12 = L2.*4.16925;
t13 = t12+3.5316;
t14 = sin(t4);
t15 = 1.0./t7.^2;
t16 = 1.0./t7;
t17 = cos(t2);
t18 = L1.*t17.*4.16925;
t19 = yb.*4.16925;
t20 = conj(th3);
t21 = t2+t3+t20;
t22 = sin(t2);
t23 = L1.*t22.*4.16925;
t24 = sin(t21);
t25 = cos(t21);
t26 = L1.*(1.7e1./4.0e1);
t27 = t26+9.0./2.5e1;
t28 = L1.*(9.0./2.5e1);
t29 = L1.^2;
t30 = t29.*(1.7e1./8.0e1);
t31 = t28+t30;
t32 = L1.*4.16925;
t33 = t32+3.5316;
t34 = 1.0./t27.^2;
t35 = 1.0./t27;
t36 = L2.*t5;
t37 = L1.*t17;
t38 = L3.*t25.*(1.0./2.0);
t39 = L3.*t24.*(1.0./2.0);
t40 = L2.*t14;
t41 = L1.*t22;
t42 = L3.^2;
t43 = L3.*t5.*4.16925;
t44 = t5-t5.*t11.*t15.*(1.7e1./4.0e1);
t45 = t13.*t44;
t46 = t5.*t11.*t16.*4.16925;
t47 = L3.*t14.*4.16925;
t48 = t14-t11.*t14.*t15.*(1.7e1./4.0e1);
t49 = t13.*t48;
t50 = t11.*t14.*t16.*4.16925;
t51 = t19+t23+t47+t49+t50;
t52 = L2.*t5.*4.16925;
t53 = L3.*t25.*4.16925;
t54 = L3.*t24.*4.16925;
t55 = L2.*t14.*4.16925;
t56 = t19+t23+t54+t55;
t57 = t39+t40;
t58 = L3.*t57.*4.16925;
t59 = t11.*t13.*t14.*t16;
t60 = t58+t59;
t61 = t36+t38;
t62 = L3.*t61.*4.16925;
t63 = t5.*t11.*t13.*t16;
t64 = t62+t63;
t65 = t5.*t11.*t16;
t66 = t37+t65;
t67 = t13.*t66;
t68 = t36+t37+t38;
t69 = L3.*t68.*4.16925;
t70 = t17.*t31.*t33.*t35;
t71 = t67+t69+t70;
t72 = t42.^2;
df_torquedth = [t51.*(t18+t43+t45+t46).*2.0-t71.*(t13.*(t41+t11.*t14.*t16)+L3.*(t39+t40+t41).*4.16925+t22.*t31.*t33.*t35).*2.0-t60.*t64.*2.0+t56.*(t18+t52+t53).*2.0+(L3.*t17.*4.16925+t13.*t17+t33.*(t17-t17.*t31.*t34.*(1.7e1./4.0e1))+t17.*t31.*t35.*4.16925).*(t19+L3.*t22.*4.16925+t13.*t22+t33.*(t22-t22.*t31.*t34.*(1.7e1./4.0e1))+t22.*t31.*t35.*4.16925).*2.0-t24.*t25.*t72.*8.69132278125,t56.*(t52+t53).*2.0-t60.*t64.*2.0-t60.*t71.*2.0+t51.*(t43+t45+t46).*2.0-t24.*t25.*t72.*8.69132278125,L3.*t25.*t56.*8.3385-t24.*t25.*t72.*8.69132278125-t24.*t42.*t64.*4.16925-t24.*t42.*t71.*4.16925];
