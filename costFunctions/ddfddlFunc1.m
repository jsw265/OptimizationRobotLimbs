function ddfddl = ddfddlFunc1(th1,L1,in3,effOffset,in5)
%DDFDDLFUNC1
%    DDFDDL = DDFDDLFUNC1(TH1,L1,IN3,EFFOFFSET,IN5)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    25-Nov-2016 18:59:02

t2 = cos(th1);
t3 = sin(th1);
ddfddl = t2.^2.*2.0+t3.^2.*2.0;
