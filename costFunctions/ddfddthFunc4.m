function ddfddth = ddfddthFunc4(in1,in2,in3,effOffset,in5)
%DDFDDTHFUNC4
%    DDFDDTH = DDFDDTHFUNC4(IN1,IN2,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 17:45:35

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
L4 = in2(4,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
th4 = in1(4,:);
thd = in5(3,:);
xb = in3(1,:);
xd = in5(1,:);
yb = in3(2,:);
yd = in5(2,:);
t2 = th1+th2+th3+th4;
t3 = cos(t2);
t4 = sin(effOffset);
t5 = sin(t2);
t6 = cos(effOffset);
t26 = cos(thd);
t27 = t3.*t6;
t28 = t4.*t5;
t29 = t27-t28;
t30 = t26.*t29;
t31 = sin(thd);
t32 = t3.*t4;
t33 = t5.*t6;
t34 = t32+t33;
t35 = t31.*t34;
t7 = t30+t35;
t8 = L4.*t3;
t9 = th1+th2;
t10 = cos(t9);
t11 = L2.*t10;
t12 = cos(th1);
t13 = L1.*t12;
t14 = th1+th2+th3;
t15 = cos(t14);
t16 = L3.*t15;
t17 = t8+t11+t13+t16;
t18 = L4.*t5;
t19 = sin(t9);
t20 = L2.*t19;
t21 = sin(th1);
t22 = L1.*t21;
t23 = sin(t14);
t24 = L3.*t23;
t25 = t18+t20+t22+t24;
t36 = t18+t20+t22+t24+yb-yd;
t37 = t7.^2;
t38 = t37.*4.0;
t39 = t8+t11+t16;
t40 = t8+t11+t13+t16+xb-xd;
t41 = t18+t20+t24;
t42 = t30+t35-1.0;
t43 = t8+t16;
t44 = t18+t24;
t45 = t17.*t39.*2.0;
t46 = t25.*t41.*2.0;
t48 = t36.*t41.*2.0;
t49 = t39.*t40.*2.0;
t50 = t7.*t42.*4.0;
t47 = t38+t45+t46-t48-t49-t50;
t51 = t17.*t43.*2.0;
t52 = t25.*t44.*2.0;
t53 = t39.*t43.*2.0;
t54 = t41.*t44.*2.0;
t56 = t40.*t43.*2.0;
t57 = t36.*t44.*2.0;
t55 = t38-t50+t53+t54-t56-t57;
t58 = L4.*t3.*t17.*2.0;
t59 = L4.*t5.*t25.*2.0;
t60 = L4.*t3.*t39.*2.0;
t61 = L4.*t5.*t41.*2.0;
t63 = L4.*t5.*t36.*2.0;
t65 = L4.*t3.*t40.*2.0;
t62 = t38-t50+t60+t61-t63-t65;
t64 = L4.*t3.*t43.*2.0;
t66 = L4.*t5.*t44.*2.0;
t67 = L4.^2;
ddfddth = reshape([t38-t7.*t42.*4.0-t17.*t40.*2.0-t25.*t36.*2.0+t17.^2.*2.0+t25.^2.*2.0,t47,t38-t50+t51+t52-t36.*t44.*2.0-t40.*t43.*2.0,t38-t50+t58+t59-L4.*t5.*t36.*2.0-L4.*t3.*t40.*2.0,t47,t38-t48-t49-t50+t39.^2.*2.0+t41.^2.*2.0,t55,t62,t38+t51+t52-t7.*t42.*4.0-t36.*t44.*2.0-t40.*t43.*2.0,t55,t38-t50-t56-t57+t43.^2.*2.0+t44.^2.*2.0,t38-t50-t63+t64-t65+t66,t38+t58+t59-t7.*t42.*4.0-L4.*t5.*t36.*2.0-L4.*t3.*t40.*2.0,t62,t38-t50+t64+t66-L4.*t5.*t36.*2.0-L4.*t3.*t40.*2.0,t38-t50-t63-t65+t3.^2.*t67.*2.0+t5.^2.*t67.*2.0],[4,4]);
