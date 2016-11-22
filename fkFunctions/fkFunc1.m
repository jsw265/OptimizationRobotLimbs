function g_st = fkFunc1(th1,L1,in3)
%FKFUNC1
%    G_ST = FKFUNC1(TH1,L1,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Nov-2016 09:25:25

xb = in3(1,:);
yb = in3(2,:);
zb = in3(3,:);
t2 = cos(th1);
t3 = sin(th1);
g_st = reshape([1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,xb,yb,zb,1.0,t2,t3,0.0,0.0,-t3,t2,0.0,0.0,0.0,0.0,1.0,0.0,xb+L1.*t2,yb+L1.*t3,zb,1.0],[4,4,2]);
