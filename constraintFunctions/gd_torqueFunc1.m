function gd_tau = gd_torqueFunc1(th1,L1,in3)
%GD_TORQUEFUNC1
%    GD_TAU = GD_TORQUEFUNC1(TH1,L1,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    29-Nov-2016 11:40:37

gd_tau = [L1.^2.*sin(th1).*(-2.084625),L1.*cos(th1).*4.16925];
