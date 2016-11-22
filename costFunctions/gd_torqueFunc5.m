function dtau = gd_torqueFunc5(in1,in2,in3)
%GD_TORQUEFUNC5
%    DTAU = GD_TORQUEFUNC5(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Nov-2016 10:05:28

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
L4 = in2(4,:);
L5 = in2(5,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
t2 = conj(th1);
t3 = conj(th2);
t4 = conj(th3);
t5 = sin(t2);
t6 = L1.*t5;
t7 = t2+t3;
t8 = sin(t7);
t9 = L2.*t8;
t10 = t2+t3+t4;
t11 = sin(t10);
t12 = L3.*t11;
t13 = conj(th4);
t14 = t2+t3+t4+t13;
t15 = sin(t14);
t16 = L4.*4.16925;
t17 = t16+3.5316;
t18 = L4.*(1.7e1./4.0e1);
t19 = t18+9.0./2.5e1;
t20 = 1.0./t19;
t21 = L4.*(9.0./2.5e1);
t22 = L4.^2;
t23 = t22.*(1.7e1./8.0e1);
t24 = t21+t23;
t25 = t15.*t20.*t24;
t26 = L3.*4.16925;
t27 = t26+3.5316;
t28 = L3.*(1.7e1./4.0e1);
t29 = t28+9.0./2.5e1;
t30 = 1.0./t29;
t31 = L3.*(9.0./2.5e1);
t32 = L3.^2;
t33 = t32.*(1.7e1./8.0e1);
t34 = t31+t33;
t35 = t11.*t30.*t34;
t36 = conj(th5);
t37 = t2+t3+t4+t13+t36;
t38 = sin(t37);
t39 = L5.*t38.*(1.0./2.0);
t40 = L4.*t15;
t41 = L2.*(1.7e1./4.0e1);
t42 = t41+9.0./2.5e1;
t43 = 1.0./t42;
t44 = L2.*4.16925;
t45 = t44+3.5316;
t46 = L2.*(9.0./2.5e1);
t47 = L2.^2;
t48 = t47.*(1.7e1./8.0e1);
t49 = t46+t48;
t50 = L1.*4.16925;
t51 = t50+3.5316;
t52 = cos(t2);
t53 = L1.*(1.7e1./4.0e1);
t54 = t53+9.0./2.5e1;
t55 = 1.0./t54;
t56 = L1.*(9.0./2.5e1);
t57 = L1.^2;
t58 = t57.*(1.7e1./8.0e1);
t59 = t56+t58;
t60 = cos(t7);
t61 = cos(t10);
t62 = L1.*t52.*4.16925;
t63 = L2.*t60.*4.16925;
t64 = cos(t14);
t65 = L3.*t61.*4.16925;
t66 = t9+t12+t25;
t67 = t9+t35;
t68 = t9+t12+t39+t40;
t70 = t17.*t66;
t71 = t27.*t67;
t72 = L5.*t68.*4.16925;
t73 = t8.*t43.*t45.*t49;
t69 = -t70-t71-t72-t73;
t74 = t12+t39+t40;
t75 = t12+t25;
t99 = L5.*t74.*4.16925;
t100 = t17.*t75;
t101 = t11.*t27.*t30.*t34;
t76 = -t99-t100-t101;
t77 = t39+t40;
t102 = L5.*t77.*4.16925;
t103 = t15.*t17.*t20.*t24;
t78 = -t102-t103;
t79 = L5.^2;
t80 = L5.*t60.*4.16925;
t81 = t27.*t60;
t82 = t17.*t60;
t83 = t43.*t49.*t60.*4.16925;
t84 = 1.0./t42.^2;
t85 = 1.0./t29.^2;
t117 = t34.*t61.*t85.*(1.7e1./4.0e1);
t86 = t61-t117;
t87 = t27.*t86;
t88 = t17.*t61;
t89 = L5.*t61.*4.16925;
t90 = t30.*t34.*t61.*4.16925;
t91 = L5.*t64.*4.16925;
t92 = 1.0./t19.^2;
t104 = t24.*t64.*t92.*(1.7e1./4.0e1);
t93 = t64-t104;
t94 = t17.*t93;
t95 = t20.*t24.*t64.*4.16925;
t96 = cos(t37);
t97 = L5.*t96.*4.16925;
t98 = L4.*t64.*4.16925;
t105 = 1.0./t54.^2;
t106 = t45.*t52;
t107 = t27.*t52;
t108 = t17.*t52;
t109 = L5.*t52.*4.16925;
t110 = t52.*t55.*t59.*4.16925;
t111 = t5.*4.16925;
t115 = t49.*t60.*t84.*(1.7e1./4.0e1);
t112 = t60-t115;
t113 = t45.*t112;
t114 = t62+t80+t81+t82+t83+t113;
t116 = t8.*4.16925;
t118 = t62+t63+t87+t88+t89+t90;
t119 = t63+t87+t88+t89+t90;
t120 = t11.*4.16925;
t121 = t62+t63+t65+t91+t94+t95;
t122 = t63+t65+t91+t94+t95;
t123 = t65+t91+t94+t95;
t124 = t62+t63+t65+t97+t98;
t125 = t63+t65+t97+t98;
t126 = t65+t97+t98;
t127 = t97+t98;
t128 = t15.*4.16925;
dtau = reshape([-t17.*(t6+t9+t12+t25)-L5.*(t6+t9+t12+t39+t40).*4.16925-t45.*(t6+t8.*t43.*t49)-t27.*(t6+t9+t35)-t5.*t51.*t55.*t59,t69,t76,t78,t38.*t79.*(-2.084625),t106+t107+t108+t109+t110+t51.*(t52-t52.*t59.*t105.*(1.7e1./4.0e1)),t114,t118,t121,t124,t69,t69,t76,t78,t38.*t79.*(-2.084625),0.0,t80+t81+t82+t83+t113,t119,t122,t125,t76,t76,t76,t78,t38.*t79.*(-2.084625),0.0,0.0,t87+t88+t89+t90,t123,t126,t78,t78,t78,t78,t38.*t79.*(-2.084625),0.0,0.0,0.0,t91+t94+t95,t127,t38.*t79.*(-2.084625),t38.*t79.*(-2.084625),t38.*t79.*(-2.084625),t38.*t79.*(-2.084625),t38.*t79.*(-2.084625),0.0,0.0,0.0,0.0,t97,t106+t107+t108+t109+t110+t51.*t52-t51.*t52.*t59.*t105.*(1.7e1./4.0e1),0.0,0.0,0.0,0.0,t5.*8.3385-t51.*(t5.*t55.*(1.7e1./4.0e1)-t5.*1.0./t54.^3.*t59.*(2.89e2./8.0e2))-t5.*t59.*t105.*3.5438625,t111,t111,t111,t111,t114,t80+t81+t82+t83+t45.*t60-t45.*t49.*t60.*t84.*(1.7e1./4.0e1),0.0,0.0,0.0,t111,t8.*8.3385-t45.*(t8.*t43.*(1.7e1./4.0e1)-t8.*1.0./t42.^3.*t49.*(2.89e2./8.0e2))-t8.*t49.*t84.*3.5438625,t116,t116,t116,t118,t119,t88+t89+t90+t27.*t61-t27.*t34.*t61.*t85.*(1.7e1./4.0e1),0.0,0.0,t111,t116,t11.*8.3385-t27.*(t11.*t30.*(1.7e1./4.0e1)-t11.*1.0./t29.^3.*t34.*(2.89e2./8.0e2))-t11.*t34.*t85.*3.5438625,t120,t120,t121,t122,t123,t91+t95+t17.*t64-t17.*t24.*t64.*t92.*(1.7e1./4.0e1),0.0,t111,t116,t120,t15.*8.3385-t17.*(t15.*t20.*(1.7e1./4.0e1)-t15.*1.0./t19.^3.*t24.*(2.89e2./8.0e2))-t15.*t24.*t92.*3.5438625,t128,t124,t125,t126,t127,t97,t111,t116,t120,t128,t38.*4.16925],[10,10]);
