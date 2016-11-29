function ddf_torqueddth = ddf_torqueddthFunc3(in1,in2,in3)
%DDF_TORQUEDDTHFUNC3
%    DDF_TORQUEDDTH = DDF_TORQUEDDTHFUNC3(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    29-Nov-2016 11:40:48

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
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
t13 = t3-t3.*t9.*t12.*(1.7e1./4.0e1);
t14 = t11.*t13;
t15 = L2.*4.16925;
t16 = t15+3.5316;
t17 = t3.*t16;
t18 = L3.*t3.*4.16925;
t19 = 1.0./t5;
t20 = t3.*t9.*t19.*4.16925;
t21 = conj(th2);
t22 = t2+t21;
t23 = cos(t22);
t25 = conj(th3);
t26 = t2+t21+t25;
t33 = L2.*(1.7e1./4.0e1);
t34 = t33+9.0./2.5e1;
t35 = 1.0./t34;
t36 = L2.*(9.0./2.5e1);
t37 = L2.^2;
t38 = t37.*(1.7e1./8.0e1);
t39 = t36+t38;
t54 = cos(t26);
t58 = L2.*t23;
t60 = L3.*t54.*(1.0./2.0);
t69 = t58+t60;
t70 = L3.*t69.*4.16925;
t71 = t16.*t23.*t35.*t39;
t24 = t70+t71;
t27 = yb.*4.16925;
t28 = sin(t26);
t29 = L3.*t28.*4.16925;
t30 = sin(t22);
t31 = L2.*t30.*4.16925;
t32 = L1.*t3.*4.16925;
t40 = L1.*t3;
t47 = L3.*t28.*(1.0./2.0);
t48 = L2.*t30;
t77 = t30.*t35.*t39;
t78 = t40+t77;
t79 = t16.*t78;
t80 = t40+t47+t48;
t81 = L3.*t80.*4.16925;
t82 = t3.*t9.*t11.*t19;
t41 = t79+t81+t82;
t42 = L3.*t30.*4.16925;
t43 = 1.0./t34.^2;
t83 = t30.*t39.*t43.*(1.7e1./4.0e1);
t44 = t30-t83;
t45 = t16.*t44;
t46 = t30.*t35.*t39.*4.16925;
t74 = t47+t48;
t75 = L3.*t74.*4.16925;
t76 = t16.*t30.*t35.*t39;
t49 = t75+t76;
t50 = cos(t2);
t51 = L3.*t50.*4.16925+t16.*t50+t11.*(t50-t9.*t12.*t50.*(1.7e1./4.0e1))+t9.*t19.*t50.*4.16925;
t55 = L1.*t50.*4.16925;
t62 = L3.*t23.*4.16925;
t63 = t23.*t39.*t43.*(1.7e1./4.0e1);
t64 = t23-t63;
t65 = t16.*t64;
t66 = t23.*t35.*t39.*4.16925;
t52 = t55+t62+t65+t66;
t53 = L3.^2;
t67 = L2.*t23.*4.16925;
t68 = L3.*t54.*4.16925;
t56 = t55+t67+t68;
t57 = t53.^2;
t59 = L1.*t50;
t90 = t23.*t35.*t39;
t91 = t59+t90;
t92 = t16.*t91;
t93 = t58+t59+t60;
t94 = L3.*t93.*4.16925;
t95 = t9.*t11.*t19.*t50;
t61 = t92+t94+t95;
t72 = t24.^2;
t73 = t27+t29+t31+t32;
t84 = t27+t32+t42+t45+t46;
t85 = t49.^2;
t86 = t85.*2.0;
t87 = t54.^2;
t88 = t28.^2;
t89 = t57.*t88.*8.69132278125;
t96 = t62+t65+t66;
t97 = t52.*t96.*2.0;
t98 = t67+t68;
t99 = t56.*t98.*2.0;
t100 = t29+t31;
t101 = t41.*t49.*2.0;
t102 = t42+t45+t46;
t104 = t72.*2.0;
t105 = t73.*t100.*2.0;
t106 = t84.*t102.*2.0;
t107 = t57.*t87.*8.69132278125;
t108 = t24.*t61.*2.0;
t103 = t86+t89+t97+t99+t101-t104-t105-t106-t107-t108;
t109 = L3.*t54.*t56.*8.3385;
t110 = t28.*t41.*t53.*4.16925;
t111 = t28.*t49.*t53.*4.16925;
t112 = L3.*t54.*t98.*8.3385;
t113 = t28.*t49.*t53.*8.3385;
t115 = t24.*t53.*t54.*4.16925;
t116 = L3.*t28.*t73.*8.3385;
t117 = t53.*t54.*t61.*4.16925;
t114 = t89-t107+t112+t113-t115-t116-t117;
ddf_torqueddth = reshape([t72.*-2.0+t86+t89-t84.*(t32+t42+t45+t46).*2.0-(t14+t17+t18+t20).*(t14+t17+t18+t20+t27).*2.0-t57.*t87.*8.69132278125-t73.*(t29+t31+t32).*2.0+t41.^2.*2.0+t51.^2.*2.0+t52.^2.*2.0+t56.^2.*2.0-t61.^2.*2.0,t103,t89-t107+t109+t110+t111-L3.*t28.*t73.*8.3385-t24.*t53.*t54.*4.16925-t53.*t54.*t61.*4.16925,t103,t85.*4.0+t89-t104-t105-t106-t107-t108+t96.^2.*2.0+t98.^2.*2.0,t114,t89+t109+t110+t111-t57.*t87.*8.69132278125-L3.*t28.*t73.*8.3385-t24.*t53.*t54.*4.16925-t53.*t54.*t61.*4.16925,t114,-t107-t115-t116-t117+t53.*t87.*3.4765291125e1+t57.*t88.*2.607396834375e1],[3,3]);
