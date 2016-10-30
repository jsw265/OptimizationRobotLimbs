function g_st = fkFunc(in1,in2,in3)
%FKFUNC
%    G_ST = FKFUNC(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    29-Oct-2016 20:10:12

L1 = in2(1,:);
L2 = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
xb = in3(1,:);
yb = in3(2,:);
zb = in3(3,:);
t2 = th1+th2;
t3 = cos(th1);
t4 = cos(t2);
t5 = L1.*t3;
t6 = sin(th1);
t7 = sin(t2);
t8 = L1.*t6;
g_st = reshape([1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,xb,yb,zb,1.0,t3,t6,0.0,0.0,-t6,t3,0.0,0.0,0.0,0.0,1.0,0.0,t5+xb,t8+yb,zb,1.0,t4,t7,0.0,0.0,-t7,t4,0.0,0.0,0.0,0.0,1.0,0.0,t5+xb+L2.*t4,t8+yb+L2.*t7,zb,1.0],[4,4,3]);
