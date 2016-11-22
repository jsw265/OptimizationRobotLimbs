function dtau = gd_torqueFunc1(th1,L1,in3)
%GD_TORQUEFUNC1
%    DTAU = GD_TORQUEFUNC1(TH1,L1,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Nov-2016 10:04:26

t2 = conj(th1);
t3 = cos(t2);
t4 = L1.*t3.*4.16925;
t5 = sin(t2);
dtau = reshape([L1.^2.*t5.*(-2.084625),t4,t4,t5.*4.16925],[2,2]);