function [f, df, ddf] = objFunc(x,p)
% function [f, df, ddf] = objFunc(x,p)
% objective function of input x [n, 1] which returns:
% f [1, 1] = objective cost value
% df [1, n] = cost gradient
% df [n,n] = cost Hessian
% Given Parameters structure p

f = 0;
df = []; % to do: implement
ddf = []; % to do: implement

% we want to reduce the sum of the squares of the error
th = reshape(x(1:p.nPoses*p.nJoints), [p.nJoints, p.nPoses]);
lengths = x(p.nPoses*p.nJoints+1 : end);
rb = [0;0;0];
for i = 1:p.nPoses
   g_st = fkFunc(th(:,i), lengths, rb); % This depends on the existance of an fkFunc function
xEff = g_st(1,4,end);
yEff = g_st(2,4,end);
REff = g_st(1:3,1:3, end);
Rd = rotmat([0;0;1], p.thd(i));
M = eye(3) - REff*Rd.';
f_i = (p.xd(i) - xEff)^2 + (p.yd(i) - yEff)^2 + sum(sum(M.^2));
   
f = f + f_i;

end



end