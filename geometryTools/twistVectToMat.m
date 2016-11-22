function out = twistVectToMat(in)
% 6x1 vector "in" is a twist
out = [skew(in(4:6)) in(1:3); [0 0 0 0]]; 

end