function ddf_torqueddl = ddf_torqueddlFunc1(th1,L1,in3)
%DDF_TORQUEDDLFUNC1
%    DDF_TORQUEDDL = DDF_TORQUEDDLFUNC1(TH1,L1,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    29-Nov-2016 11:40:38

t3 = conj(th1);
t2 = cos(t3);
t4 = sin(t3);
ddf_torqueddl = t4.^2.*3.4765291125e1+L1.^2.*t2.^2.*5.21479366875e1;
