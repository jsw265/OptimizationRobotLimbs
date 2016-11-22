function df_torquedl = df_torquedlFunc4(in1,in2,in3)
%DF_TORQUEDLFUNC4
%    DF_TORQUEDL = DF_TORQUEDLFUNC4(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Nov-2016 10:05:10

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
L4 = in2(4,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
yb = in3(2,:);
t2 = conj(th1);
t3 = conj(th2);
t4 = t2+t3;
t5 = sin(t4);
t6 = sin(t2);
t7 = L2.*(1.7e1./4.0e1);
t8 = t7+9.0./2.5e1;
t9 = L2.*(9.0./2.5e1);
t10 = L2.^2;
t11 = t10.*(1.7e1./8.0e1);
t12 = t9+t11;
t13 = L2.*4.16925;
t14 = t13+3.5316;
t15 = 1.0./t8;
t16 = cos(t4);
t17 = cos(t2);
t18 = L1.*t17;
t19 = conj(th3);
t20 = L3.*4.16925;
t21 = t20+3.5316;
t22 = L2.*t16;
t23 = t2+t3+t19;
t24 = cos(t23);
t25 = L1.*4.16925;
t26 = t25+3.5316;
t27 = L1.*(1.7e1./4.0e1);
t28 = t27+9.0./2.5e1;
t29 = 1.0./t28;
t30 = L1.*(9.0./2.5e1);
t31 = L1.^2;
t32 = t31.*(1.7e1./8.0e1);
t33 = t30+t32;
t34 = 1.0./t28.^2;
t35 = yb.*4.16925;
t36 = sin(t23);
t37 = L3.*(1.7e1./4.0e1);
t38 = t37+9.0./2.5e1;
t39 = L3.*(9.0./2.5e1);
t40 = L3.^2;
t41 = t40.*(1.7e1./8.0e1);
t42 = t39+t41;
t43 = L1.*t6.*4.16925;
t44 = 1.0./t38;
t45 = L2.*t5.*4.16925;
t46 = conj(th4);
t47 = t2+t3+t19+t46;
t48 = L3.*t36.*4.16925;
t49 = sin(t47);
t50 = L4.*t49.*4.16925;
t51 = t35+t43+t45+t48+t50;
t92 = t6.*t33.*t34.*(1.7e1./4.0e1);
t52 = t6-t92;
t53 = t26.*t52;
t54 = t6.*t14;
t55 = t6.*t21;
t56 = L4.*t6.*4.16925;
t57 = t6.*t29.*t33.*4.16925;
t58 = t35+t53+t54+t55+t56+t57;
t59 = t12.*t15.*t16;
t60 = t18+t59;
t61 = t14.*t60;
t62 = cos(t47);
t63 = L4.*t62.*(1.0./2.0);
t64 = L3.*t24;
t65 = t18+t22+t63+t64;
t66 = L4.*t65.*4.16925;
t67 = t24.*t42.*t44;
t68 = t18+t22+t67;
t69 = t21.*t68;
t70 = t17.*t26.*t29.*t33;
t71 = t61+t66+t69+t70;
t72 = 1.0./t8.^2;
t73 = L4.*t5.*4.16925;
t88 = t5.*t12.*t72.*(1.7e1./4.0e1);
t74 = t5-t88;
t75 = t14.*t74;
t76 = t5.*t21;
t77 = t5.*t12.*t15.*4.16925;
t78 = t35+t43+t73+t75+t76+t77;
t79 = L4.*t36.*4.16925;
t80 = 1.0./t38.^2;
t94 = t36.*t42.*t80.*(1.7e1./4.0e1);
t81 = t36-t94;
t82 = t21.*t81;
t83 = t36.*t42.*t44.*4.16925;
t84 = t35+t43+t45+t79+t82+t83;
t85 = L4.*t16.*4.16925;
t86 = t16.*t21;
t87 = t12.*t15.*t16.*4.16925;
t89 = L1.*t17.*4.16925;
t90 = L4.*t24.*4.16925;
t91 = t24.*t42.*t44.*4.16925;
t93 = t6.*t58.*8.3385;
t95 = t22+t67;
t96 = t21.*t95;
t97 = t22+t63+t64;
t98 = L4.*t97.*4.16925;
t99 = t12.*t14.*t15.*t16;
t100 = t96+t98+t99;
t101 = L2.*t16.*4.16925;
t102 = t24-t24.*t42.*t80.*(1.7e1./4.0e1);
t103 = t21.*t102;
t104 = t5.*t78.*8.3385;
t105 = L4.^2;
t106 = L4.*t62.*4.16925;
t107 = L3.*t24.*4.16925;
t108 = t63+t64;
t109 = L4.*t108.*4.16925;
t110 = t21.*t24.*t42.*t44;
t111 = t109+t110;
df_torquedl = [t71.*(L4.*t17.*4.16925+t14.*t17+t17.*t21+t17.*t26+t17.*t29.*t33.*4.16925-t17.*t26.*t33.*t34.*(1.7e1./4.0e1)).*2.0+t6.*t51.*8.3385+t6.*t78.*8.3385+t6.*t84.*8.3385-t58.*(t6.*(-8.3385)+t26.*(t6.*t29.*(1.7e1./4.0e1)-t6.*1.0./t28.^3.*t33.*(2.89e2./8.0e2))+t6.*t33.*t34.*3.5438625).*2.0,t93+t100.*(t85+t86+t87+t14.*t16-t12.*t14.*t16.*t72.*(1.7e1./4.0e1)).*2.0+t5.*t51.*8.3385+t5.*t84.*8.3385-t78.*(t5.*(-8.3385)+t14.*(t5.*t15.*(1.7e1./4.0e1)-t5.*1.0./t8.^3.*t12.*(2.89e2./8.0e2))+t5.*t12.*t72.*3.5438625).*2.0+t71.*(t85+t86+t87+t89+t14.*(t16-t12.*t16.*t72.*(1.7e1./4.0e1))).*2.0,t93+t104+t100.*(t90+t91+t101+t103).*2.0+t36.*t51.*8.3385-t84.*(t36.*(-8.3385)+t21.*(t36.*t44.*(1.7e1./4.0e1)-t36.*1.0./t38.^3.*t42.*(2.89e2./8.0e2))+t36.*t42.*t80.*3.5438625).*2.0+t71.*(t89+t90+t91+t101+t103).*2.0+t111.*(t90+t91+t21.*t24-t21.*t24.*t42.*t80.*(1.7e1./4.0e1)).*2.0,t93+t104+t71.*(t89+t101+t106+t107).*2.0+t111.*(t106+t107).*2.0+t49.*t51.*8.3385+t36.*t84.*8.3385+t100.*(t101+t106+t107).*2.0+L4.*t62.^2.*t105.*1.73826455625e1];