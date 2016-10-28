function [f, df, ddf] = objFunc(x,p)
% function [f, df, ddf] = objFunc(x,p)
% objective function of input x [n, 1] which returns:
% f [1, 1] = objective cost value
% df [1, n] = cost gradient
% df [n,n] = cost Hessian
% Given Parameters structure p


f = 0;
df = zeros(1,p.nPoses*p.nJoints+p.nJoints); 
ddf = zeros(p.nPoses*p.nJoints+p.nJoints, p.nPoses*p.nJoints+p.nJoints); 


% we want to reduce the sum of the squares of the error
th = reshape(x(1:p.nPoses*p.nJoints), [p.nJoints, p.nPoses]);
lengths = x(p.nPoses*p.nJoints+1 : end);
rb = [0;0;0];
Td = [p.xd p.yd p.thd].';
for i = 1:p.nPoses
    
%     % calculate the cost from the fk:
%    g_st = fkFunc(th(:,i), lengths, rb); % This depends on the existance of an fkFunc function
% xEff = g_st(1,4,end);
% yEff = g_st(2,4,end);
% REff = g_st(1:3,1:3, end);
% Rd = rotmat([0;0;1], p.thd(i));
% M = eye(3) - REff*Rd.';
% f_i = (p.xd(i) - xEff)^2 + (p.yd(i) - yEff)^2 + sum(sum(M.^2));

% alternately calculate the cost from premade file
f_i = fFunc(th(:,i), lengths, rb, Td(:,i));
f = f + f_i;


dfdth_i = dfdthFunc(th(:,i), lengths, rb, Td(:,i));
dfdl_i = dfdlFunc(th(:,i), lengths, rb, Td(:,i));
df_i = zeros(1,p.nPoses*p.nJoints+p.nJoints);
df_i( (i*p.nJoints - (p.nJoints-1)):(i*p.nJoints) ) = dfdth_i;
df_i( p.nJoints*p.nPoses+1:end) = dfdl_i;
df = df + df_i;

ddfddth_i = ddfddthFunc(th(:,i), lengths, rb, Td(:,i));
ddfddl_i = ddfddlFunc(th(:,i), lengths, rb, Td(:,i));
ddf_i = zeros(p.nPoses*p.nJoints+p.nJoints,p.nPoses*p.nJoints+p.nJoints);

%% To do: check this, since I am not sure I got it right
ddf_i( (i*p.nJoints - (p.nJoints-1)):(i*p.nJoints), ...
      (i*p.nJoints - (p.nJoints-1)):(i*p.nJoints) ) = ddfddth_i;
ddf_i( p.nJoints*p.nPoses+1:end, ...
      p.nJoints*p.nPoses+1:end) = ddfddl_i;
ddf = ddf + ddf_i;

  
end

plotResults(x, p)

end






