function ddfddth = ddfddthFunc(in1,in2,in3,in4)
%DDFDDTHFUNC
%    DDFDDTH = DDFDDTHFUNC(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    08-Nov-2016 09:45:55

L1 = in2(1,:);
L2 = in2(2,:);
L3 = in2(3,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
thd = in4(3,:);
xb = in3(1,:);
xd = in4(1,:);
yb = in3(2,:);
yd = in4(2,:);
t2 = th1+th2;
t3 = sin(t2);
t4 = L2.*t3;
t5 = sin(th1);
t6 = L1.*t5;
t7 = th1+th2+th3;
t8 = sin(t7);
t9 = L3.*t8;
t11 = cos(t7);
t14 = cos(t2);
t15 = L2.*t14;
t16 = cos(th1);
t17 = L1.*t16;
t18 = L3.*t11;
t10 = t15+t17+t18;
t19 = cos(thd);
t20 = t11.*t19;
t21 = sin(thd);
t22 = t8.*t21;
t12 = t20+t22;
t13 = t4+t6+t9;
t23 = t15+t17+t18+xb-xd;
t24 = t4+t9;
t25 = t4+t6+t9+yb-yd;
t26 = t12.^2;
t27 = t26.*4.0;
t28 = t20+t22-1.0;
t29 = t15+t18;
t30 = t13.*t24.*2.0;
t31 = t10.*t29.*2.0;
t33 = t23.*t29.*2.0;
t34 = t24.*t25.*2.0;
t35 = t12.*t28.*4.0;
t32 = t27+t30+t31-t33-t34-t35;
t36 = L3.*t8.*t13.*2.0;
t37 = L3.*t10.*t11.*2.0;
t38 = L3.*t11.*t29.*2.0;
t39 = L3.*t8.*t24.*2.0;
t42 = L3.*t11.*t23.*2.0;
t43 = L3.*t8.*t25.*2.0;
t40 = t27-t35+t38+t39-t42-t43;
t41 = L3.^2;
ddfddth = reshape([t27-t10.*t23.*2.0-t13.*t25.*2.0-t12.*t28.*4.0+t10.^2.*2.0+t13.^2.*2.0,t32,t27-t35+t36+t37-L3.*t8.*t25.*2.0-L3.*t11.*t23.*2.0,t32,t27-t33-t34-t35+t24.^2.*2.0+t29.^2.*2.0,t40,t27+t36+t37-t12.*t28.*4.0-L3.*t8.*t25.*2.0-L3.*t11.*t23.*2.0,t40,t27-t35-t42-t43+t8.^2.*t41.*2.0+t11.^2.*t41.*2.0],[3,3]);
