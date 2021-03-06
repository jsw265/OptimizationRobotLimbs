function df_torquedl = df_torquedlFunc6(in1,in2,in3)
%DF_TORQUEDLFUNC6
%    DF_TORQUEDL = DF_TORQUEDLFUNC6(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    29-Nov-2016 11:47:55

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
L4 = in2(4,:);
L5 = in2(5,:);
L6 = in2(6,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
th5 = in1(5,:);
th6 = in1(6,:);
yb = in3(2,:);
t2 = conj(th1);
t3 = sin(t2);
t4 = L1.*(1.7e1./4.0e1);
t5 = t4+9.0./2.5e1;
t6 = L1.*(9.0./2.5e1);
t7 = L1.^2;
t8 = t7.*(1.7e1./8.0e1);
t9 = t6+t8;
t10 = L1.*4.16925;
t11 = t10+3.5316;
t12 = 1.0./t5.^2;
t13 = 1.0./t5;
t14 = yb.*4.16925;
t15 = conj(th2);
t16 = conj(th3);
t17 = conj(th4);
t18 = conj(th5);
t19 = t2+t15+t16;
t20 = sin(t19);
t21 = t2+t15;
t22 = sin(t21);
t23 = L2.*t22.*4.16925;
t24 = L3.*4.16925;
t25 = t24+3.5316;
t26 = L4.*4.16925;
t27 = t26+3.5316;
t28 = L5.*4.16925;
t29 = t28+3.5316;
t30 = L1.*t3.*4.16925;
t31 = L3.*(1.7e1./4.0e1);
t32 = t31+9.0./2.5e1;
t33 = L3.*(9.0./2.5e1);
t34 = L3.^2;
t35 = t34.*(1.7e1./8.0e1);
t36 = t33+t35;
t37 = L3.*t20.*4.16925;
t38 = t2+t15+t16+t17+t18;
t39 = sin(t38);
t40 = t2+t15+t16+t17;
t41 = sin(t40);
t42 = L4.*t41.*4.16925;
t43 = L5.*(1.7e1./4.0e1);
t44 = t43+9.0./2.5e1;
t45 = L5.*(9.0./2.5e1);
t46 = L5.^2;
t47 = t46.*(1.7e1./8.0e1);
t48 = t45+t47;
t49 = L4.*(1.7e1./4.0e1);
t50 = t49+9.0./2.5e1;
t51 = L4.*(9.0./2.5e1);
t52 = L4.^2;
t53 = t52.*(1.7e1./8.0e1);
t54 = t51+t53;
t55 = L2.*4.16925;
t56 = t55+3.5316;
t57 = cos(t21);
t58 = cos(t2);
t59 = L1.*t58;
t60 = 1.0./t44;
t61 = L2.*t57;
t62 = cos(t19);
t63 = L3.*t62;
t64 = cos(t40);
t65 = 1.0./t50;
t66 = conj(th6);
t67 = t2+t15+t16+t17+t18+t66;
t68 = cos(t38);
t69 = L4.*t64;
t70 = 1.0./t32;
t71 = L2.*(1.7e1./4.0e1);
t72 = t71+9.0./2.5e1;
t73 = L2.*(9.0./2.5e1);
t74 = L2.^2;
t75 = t74.*(1.7e1./8.0e1);
t76 = t73+t75;
t77 = 1.0./t72;
t146 = t3.*t9.*t12.*(1.7e1./4.0e1);
t78 = t3-t146;
t79 = t11.*t78;
t80 = t3.*t56;
t81 = t3.*t25;
t82 = t3.*t27;
t83 = t3.*t29;
t84 = L6.*t3.*4.16925;
t85 = t3.*t9.*t13.*4.16925;
t86 = t14+t79+t80+t81+t82+t83+t84+t85;
t87 = cos(t67);
t88 = L6.*t87.*(1.0./2.0);
t89 = L5.*t68;
t90 = t36.*t62.*t70;
t91 = t48.*t60.*t68;
t92 = t54.*t64.*t65;
t93 = 1.0./t72.^2;
t94 = sin(t67);
t95 = L6.*t94.*4.16925;
t96 = L5.*t39.*4.16925;
t97 = t14+t23+t30+t37+t42+t95+t96;
t98 = L6.*t20.*4.16925;
t99 = 1.0./t32.^2;
t150 = t20.*t36.*t99.*(1.7e1./4.0e1);
t100 = t20-t150;
t101 = t25.*t100;
t102 = t20.*t27;
t103 = t20.*t29;
t104 = t20.*t36.*t70.*4.16925;
t105 = t14+t23+t30+t98+t101+t102+t103+t104;
t106 = L6.*t39.*4.16925;
t107 = 1.0./t44.^2;
t149 = t39.*t48.*t107.*(1.7e1./4.0e1);
t108 = t39-t149;
t109 = t29.*t108;
t110 = t39.*t48.*t60.*4.16925;
t111 = t14+t23+t30+t37+t42+t106+t109+t110;
t112 = L6.*t22.*4.16925;
t145 = t22.*t76.*t93.*(1.7e1./4.0e1);
t113 = t22-t145;
t114 = t56.*t113;
t115 = t22.*t25;
t116 = t22.*t27;
t117 = t22.*t29;
t118 = t22.*t76.*t77.*4.16925;
t119 = t14+t30+t112+t114+t115+t116+t117+t118;
t120 = 1.0./t50.^2;
t151 = t41.*t54.*t120.*(1.7e1./4.0e1);
t121 = t41-t151;
t122 = t27.*t121;
t123 = t29.*t41;
t124 = L6.*t41.*4.16925;
t125 = t41.*t54.*t65.*4.16925;
t126 = t14+t23+t30+t37+t122+t123+t124+t125;
t127 = t57.*t76.*t77;
t128 = t59+t127;
t129 = t56.*t128;
t130 = t59+t61+t63+t69+t91;
t131 = t29.*t130;
t132 = t59+t61+t63+t92;
t133 = t27.*t132;
t134 = t59+t61+t63+t69+t88+t89;
t135 = L6.*t134.*4.16925;
t136 = t59+t61+t90;
t137 = t25.*t136;
t138 = t9.*t11.*t13.*t58;
t139 = t129+t131+t133+t135+t137+t138;
t140 = L6.*t57.*4.16925;
t141 = t25.*t57;
t142 = t27.*t57;
t143 = t29.*t57;
t144 = t57.*t76.*t77.*4.16925;
t147 = t3.*t86.*8.3385;
t148 = L1.*t58.*4.16925;
t152 = t27.*t62;
t153 = t29.*t62;
t154 = L6.*t62.*4.16925;
t155 = t36.*t62.*t70.*4.16925;
t156 = t61+t63+t69+t88+t89;
t157 = L6.*t156.*4.16925;
t158 = t61+t90;
t159 = t25.*t158;
t160 = t61+t63+t69+t91;
t161 = t29.*t160;
t162 = t61+t63+t92;
t163 = t27.*t162;
t164 = t56.*t57.*t76.*t77;
t165 = t157+t159+t161+t163+t164;
t166 = L2.*t57.*4.16925;
t167 = t62-t36.*t62.*t99.*(1.7e1./4.0e1);
t168 = t25.*t167;
t169 = t22.*t119.*8.3385;
t170 = t29.*t64;
t171 = L6.*t64.*4.16925;
t172 = t54.*t64.*t65.*4.16925;
t184 = t54.*t64.*t120.*(1.7e1./4.0e1);
t173 = t64-t184;
t174 = t27.*t173;
t175 = L3.*t62.*4.16925;
t176 = t63+t69+t91;
t177 = t29.*t176;
t178 = t63+t92;
t179 = t27.*t178;
t180 = t63+t69+t88+t89;
t181 = L6.*t180.*4.16925;
t182 = t25.*t36.*t62.*t70;
t183 = t177+t179+t181+t182;
t185 = t20.*t105.*8.3385;
t197 = t48.*t68.*t107.*(1.7e1./4.0e1);
t186 = t68-t197;
t187 = t29.*t186;
t188 = L6.*t68.*4.16925;
t189 = L4.*t64.*4.16925;
t190 = t48.*t60.*t68.*4.16925;
t191 = t69+t91;
t192 = t29.*t191;
t193 = t69+t88+t89;
t194 = L6.*t193.*4.16925;
t195 = t27.*t54.*t64.*t65;
t196 = t192+t194+t195;
t198 = L6.^2;
t199 = L6.*t87.*4.16925;
t200 = L5.*t68.*4.16925;
t201 = t88+t89;
t202 = L6.*t201.*4.16925;
t203 = t29.*t48.*t60.*t68;
t204 = t202+t203;
t205 = t41.*t126.*8.3385;
df_torquedl = [t3.*t97.*8.3385+t3.*t105.*8.3385+t3.*t111.*8.3385+t3.*t119.*8.3385+t3.*t126.*8.3385-t86.*(t3.*(-8.3385)+t11.*(t3.*t13.*(1.7e1./4.0e1)-t3.*1.0./t5.^3.*t9.*(2.89e2./8.0e2))+t3.*t9.*t12.*3.5438625).*2.0+t139.*(L6.*t58.*4.16925+t11.*t58+t25.*t58+t27.*t58+t29.*t58+t56.*t58+t9.*t13.*t58.*4.16925-t9.*t11.*t12.*t58.*(1.7e1./4.0e1)).*2.0,t147+t22.*t97.*8.3385+t22.*t105.*8.3385+t22.*t111.*8.3385+t22.*t126.*8.3385+t139.*(t140+t141+t142+t143+t144+t148+t56.*(t57-t57.*t76.*t93.*(1.7e1./4.0e1))).*2.0-t119.*(t22.*(-8.3385)+t56.*(t22.*t77.*(1.7e1./4.0e1)-t22.*1.0./t72.^3.*t76.*(2.89e2./8.0e2))+t22.*t76.*t93.*3.5438625).*2.0+t165.*(t140+t141+t142+t143+t144+t56.*t57-t56.*t57.*t76.*t93.*(1.7e1./4.0e1)).*2.0,t147+t169+t20.*t97.*8.3385+t20.*t111.*8.3385+t20.*t126.*8.3385-t105.*(t20.*(-8.3385)+t25.*(t20.*t70.*(1.7e1./4.0e1)-t20.*1.0./t32.^3.*t36.*(2.89e2./8.0e2))+t20.*t36.*t99.*3.5438625).*2.0+t139.*(t148+t152+t153+t154+t155+t166+t168).*2.0+t183.*(t152+t153+t154+t155+t25.*t62-t25.*t36.*t62.*t99.*(1.7e1./4.0e1)).*2.0+t165.*(t152+t153+t154+t155+t166+t168).*2.0,t147+t169+t185+t196.*(t170+t171+t172+t27.*t64-t27.*t54.*t64.*t120.*(1.7e1./4.0e1)).*2.0+t41.*t97.*8.3385+t41.*t111.*8.3385-t126.*(t41.*(-8.3385)+t27.*(t41.*t65.*(1.7e1./4.0e1)-t41.*1.0./t50.^3.*t54.*(2.89e2./8.0e2))+t41.*t54.*t120.*3.5438625).*2.0+t139.*(t148+t166+t170+t171+t172+t174+t175).*2.0+t183.*(t170+t171+t172+t174+t175).*2.0+t165.*(t166+t170+t171+t172+t174+t175).*2.0,t147+t169+t185+t205+t196.*(t187+t188+t189+t190).*2.0+t39.*t97.*8.3385-t111.*(t39.*(-8.3385)+t29.*(t39.*t60.*(1.7e1./4.0e1)-t39.*1.0./t44.^3.*t48.*(2.89e2./8.0e2))+t39.*t48.*t107.*3.5438625).*2.0+t139.*(t148+t166+t175+t187+t188+t189+t190).*2.0+t183.*(t175+t187+t188+t189+t190).*2.0+t204.*(t188+t190+t29.*t68-t29.*t48.*t68.*t107.*(1.7e1./4.0e1)).*2.0+t165.*(t166+t175+t187+t188+t189+t190).*2.0,t147+t169+t185+t205+t183.*(t175+t189+t199+t200).*2.0+t204.*(t199+t200).*2.0+t39.*t111.*8.3385+t94.*t97.*8.3385+t165.*(t166+t175+t189+t199+t200).*2.0+t196.*(t189+t199+t200).*2.0+t139.*(t148+t166+t175+t189+t199+t200).*2.0+L6.*t87.^2.*t198.*1.73826455625e1];
