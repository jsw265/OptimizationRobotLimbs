function [f, df, ddf] = objFunc(x,p)
% function [f, df, ddf] = objFunc(x,p)
% objective function of input x [n, 1] which returns:
% f [1, 1] = objective cost value
% df [1, n] = cost gradient
% df [n,n] = cost Hessian
% Given Parameters structure p


f = 0;
nVars = length(x);
df = zeros(1,nVars); 
ddf = zeros(nVars,nVars); 


% we want to reduce the sum of the squares of the error
th = reshape(x(1:p.nPoses*p.nJoints), [p.nJoints, p.nPoses]);
lengths = x(p.nPoses*p.nJoints+ (1 : p.nJoints));

if p.variableBase
rb = [x(p.nJoints*p.nPoses+p.nJoints+(1:2)); 0];
else
rb = [0;0;0];
end

if p.variableBase&&p.variableEnd
        effVarInd = p.nJoints*p.nPoses+p.nJoints +3;
    effOffset = x(effVarInd);
elseif p.variableBase&&(~p.variableEnd)
            effVarInd = p.nJoints*p.nPoses+p.nJoints +1;
    effOffset = x(effVarInd);
else
    effOffset = 0;
    effVarInd = [];
end

Td = [p.xd p.yd p.thd].';
if p.positionErrorObjectiveWeighting
for i = 1:p.nPoses
    

% calculate the cost from premade file
f_i = fFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
f = f + p.positionErrorObjectiveWeighting*f_i;

dfdth_i = dfdthFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
dfdl_i = dfdlFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
df_i = zeros(1,p.nPoses*p.nJoints+p.nJoints);
df_i( (i*p.nJoints - (p.nJoints-1)):(i*p.nJoints) ) = dfdth_i;
df_i( p.nJoints*p.nPoses+1:end) = dfdl_i;
df = df + p.positionErrorObjectiveWeighting*df_i;

ddfddth_i = ddfddthFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
ddfddl_i = ddfddlFunc(th(:,i), lengths, rb, effOffset, Td(:,i));

ddf_i = zeros(nVars,nVars);
%% To do: check this, since I am not sure I got it right
ddf_i( (i*p.nJoints - (p.nJoints-1)):(i*p.nJoints), ...
      (i*p.nJoints - (p.nJoints-1)):(i*p.nJoints) ) = ddfddth_i;
ddf_i( p.nJoints*p.nPoses+(1:p.nJoints), ...
      p.nJoints*p.nPoses+(1:p.nJoints)) = ddfddl_i;
ddf = ddf + p.positionErrorObjectiveWeighting*ddf_i;

if p.variableBase
    inds = p.nJoints*p.nPoses+p.nJoints+(1:2);
    df(inds)= df(inds)+...
        dfdrbFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
    ddf(inds,inds)= ddf(inds,inds)+...
        ddfddrbFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
end
if p.variableEnd
       df(effVarInd ) = df(effVarInd) + dfdeffOffsetFunc(th(:,i), lengths, rb, effOffset, Td(:,i));

end

end


  


%% evaluate extra objective 1: torques from gravity
if p.useTorqueObjective
    f_torque_i = f_torqueFunc(th(:,i), lengths, rb);
    f = f+p.useTorqueObjective*f_torque_i;
    df_torquedth_i = dfdthFunc(th(:,i), lengths, rb, Td(:,i));
    df_torquedl_i = dfdlFunc(th(:,i), lengths, rb, Td(:,i));
    df_torque_i = zeros(1,p.nPoses*p.nJoints+p.nJoints);
    df_torque_i( (i*p.nJoints - (p.nJoints-1)):(i*p.nJoints) ) = df_torquedth_i;
    df_torque_i( p.nJoints*p.nPoses+1:end) = df_torquedl_i;
    df = df + p.useTorqueObjective*df_torque_i;
end

if p.lengthObjectiveWeighting
    % Linearly penalizing component length
    
    df_fromLength = zeros(size(df));
    df_fromLength(p.nPoses*p.nJoints+(1:p.nJoints)) = p.lengthObjectiveWeighting;
    
    f = f + p.lengthObjectiveWeighting*sum(x(p.nPoses*p.nJoints+(1:p.nJoints)));
    df = df + df_fromLength;
    %ddf = ddf; All zeroes from linear objective function
end

if p.jointSmoothingWeighting
    
    % penalize moving much between poses, using the metric ||I-R1*R2.'||_F 
    % see "jointSmoothingSym" to see where the math comes from,
    diff_th = diff(th, 1, 2);
    diff_th = diff_th(:).';
    
    % add objective scalar
    f = f  - sum(sum(cos(diff_th))) * p.jointSmoothingWeighting;
    dfJoints = zeros(1, p.nPoses*p.nJoints);
    dfJoints(1:p.nJoints*(p.nPoses-1)) = sin(diff_th);
    dfJoints((p.nJoints+1):p.nJoints*p.nPoses) = dfJoints((p.nJoints+1):p.nJoints*p.nPoses)...
         - sin(diff_th);
    
    % add gradient component
    df(1:p.nJoints*p.nPoses) = df(1:p.nJoints*p.nPoses) + dfJoints*p.jointSmoothingWeighting;
    
    % add hessian component
    ddfJoints =  diag([cos(diff_th) zeros(1,p.nJoints)]) +diag([ zeros(1,p.nJoints) cos(diff_th)]) ...
        - diag(cos(diff_th),p.nJoints) - diag(cos(diff_th),p.nJoints).';
    ddf(1:p.nJoints*p.nPoses, 1:p.nJoints*p.nPoses) = ddf(1:p.nJoints*p.nPoses, 1:p.nJoints*p.nPoses) ...
        + ddfJoints*p.jointSmoothingWeighting;
    
end


end



plotResults(x, p)

end






