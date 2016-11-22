function g_st = fkFunc5(in1,in2,in3)
%FKFUNC5
%    G_ST = FKFUNC5(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Nov-2016 09:25:36

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
xb = in3(1,:);
yb = in3(2,:);
zb = in3(3,:);
t2 = th1+th2;
t3 = th1+th2+th3;
t4 = th1+th2+th3+th4;
t5 = th1+th2+th3+th4+th5;
t6 = cos(th1);
t7 = cos(t2);
t8 = L1.*t6;
t9 = L2.*t7;
t10 = cos(t3);
t11 = cos(t4);
t12 = L3.*t10;
t13 = L4.*t11;
t14 = cos(t5);
t15 = sin(th1);
t16 = sin(t2);
t17 = sin(t3);
t18 = sin(t4);
t19 = sin(t5);
t20 = L1.*t15;
t21 = L2.*t16;
t22 = L3.*t17;
t23 = L4.*t18;
g_st = reshape([1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,xb,yb,zb,1.0,t6,t15,0.0,0.0,-t15,t6,0.0,0.0,0.0,0.0,1.0,0.0,t8+xb,t20+yb,zb,1.0,t7,t16,0.0,0.0,-t16,t7,0.0,0.0,0.0,0.0,1.0,0.0,t8+t9+xb,t20+t21+yb,zb,1.0,t10,t17,0.0,0.0,-t17,t10,0.0,0.0,0.0,0.0,1.0,0.0,t8+t9+t12+xb,t20+t21+t22+yb,zb,1.0,t11,t18,0.0,0.0,-t18,t11,0.0,0.0,0.0,0.0,1.0,0.0,t8+t9+t12+t13+xb,t20+t21+t22+t23+yb,zb,1.0,t14,t19,0.0,0.0,-t19,t14,0.0,0.0,0.0,0.0,1.0,0.0,t8+t9+t12+t13+xb+L5.*t14,t20+t21+t22+t23+yb+L5.*t19,zb,1.0],[4,4,6]);