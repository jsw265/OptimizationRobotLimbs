function df_torquedth = df_torquedthFunc1(th1,L1,in3)
%DF_TORQUEDTHFUNC1
%    DF_TORQUEDTH = DF_TORQUEDTHFUNC1(TH1,L1,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    29-Nov-2016 11:40:38

yb = in3(2,:);
t2 = conj(th1);
t3 = L1.^2;
t4 = cos(t2);
t5 = sin(t2);
df_torquedth = t3.^2.*t4.*t5.*(-8.69132278125)+L1.*t4.*(yb.*4.16925+L1.*t5.*4.16925).*8.3385;
