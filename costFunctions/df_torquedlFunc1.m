function df_torquedl = df_torquedlFunc1(th1,L1,in3)
%DF_TORQUEDLFUNC1
%    DF_TORQUEDL = DF_TORQUEDLFUNC1(TH1,L1,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Nov-2016 10:04:27

yb = in3(2,:);
t2 = L1.^2;
t4 = conj(th1);
t3 = cos(t4);
t5 = sin(t4);
df_torquedl = t5.*(yb.*4.16925+L1.*t5.*4.16925).*8.3385+L1.*t2.*t3.^2.*1.73826455625e1;