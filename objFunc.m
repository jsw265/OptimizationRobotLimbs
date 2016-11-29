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
nJ = p.nJoints;
nP = p.nPoses;

% we want to reduce the sum of the squares of the error
th = reshape(x(1:nP*nJ), [nJ, nP]);
lengths = x(nP*nJ+ (1 : nJ));

if p.variableBase
rb = [x(nJ*nP+nJ+(1:2)); 0];
else
rb = [0;0;0];
end

if p.variableBase&&p.variableEnd
        effVarInd = nJ*nP+nJ +3;
    effOffset = x(effVarInd);
elseif p.variableEnd&&(~p.variableBase)
            effVarInd = nJ*nP+nJ +1;
    effOffset = x(effVarInd);
else
    effOffset = 0;
    effVarInd = [];
end

Td = [p.xd p.yd p.thd].';
if p.positionErrorObjectiveWeighting
for i = 1:nP
    

% calculate the cost from premade file
f_i = fFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
f = f + p.positionErrorObjectiveWeighting*f_i;

dfdth_i = dfdthFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
dfdl_i = dfdlFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
df_i = zeros(1,nVars);
df_i( (i*nJ - (nJ-1)):(i*nJ) ) = dfdth_i;
df_i( nJ*nP+(1:nJ)) = dfdl_i;
df = df + p.positionErrorObjectiveWeighting*df_i;

ddfddth_i = ddfddthFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
ddfddl_i = ddfddlFunc(th(:,i), lengths, rb, effOffset, Td(:,i));

ddf_i = zeros(nVars,nVars);
%% To do: check this, since I am not sure I got it right
ddf_i( (i*nJ - (nJ-1)):(i*nJ), ...
      (i*nJ - (nJ-1)):(i*nJ) ) = ddfddth_i;
ddf_i( nJ*nP+(1:nJ), ...
      nJ*nP+(1:nJ)) = ddfddl_i;
ddf = ddf + p.positionErrorObjectiveWeighting*ddf_i;

if p.variableBase
    inds = nJ*nP+nJ+(1:2);
    dfdrb = dfdrbFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
    df(inds)= df(inds)+p.positionErrorObjectiveWeighting*dfdrb(1:2); % only use x,y for 2D
    
    ddfddrb = ddfddrbFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
    ddf(inds,inds)= ddf(inds,inds)+ p.positionErrorObjectiveWeighting*ddfddrb(1:2,1:2); 
    % only use x,y for 2D
end
if p.variableEnd
       df(effVarInd ) = df(effVarInd) + ...
           p.positionErrorObjectiveWeighting*dfdeffOffsetFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
ddf(effVarInd,effVarInd) = ddf(effVarInd,effVarInd) + ...
    p.positionErrorObjectiveWeighting*ddfddeffOffsetFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
end

end


  


%% evaluate extra objective 1: torques from gravity
if p.useTorqueObjective
    f_torque_i = f_torqueFunc(th(:,i), lengths, rb);
    df_torquedth_i = df_torquedthFunc(th(:,i), lengths, rb);
    df_torquedl_i = df_torquedlFunc(th(:,i), lengths, rb);
    df_torque_i = zeros(1,nP*nJ+nJ);
    df_torque_i((i-1)*nJ+1:(i*nJ)) = df_torquedth_i;
    df_torque_i( nJ*nP+1:end) = df_torquedl_i;
    
    if p.variableBase
        df_torque_i = [df_torque_i,0,0];
    end
    if p.variableEnd
        df_torque_i = [df_torque_i, 0];
    end
    f = f+p.useTorqueObjective*f_torque_i;
    df = df + p.useTorqueObjective*df_torque_i;
end

if p.lengthObjectiveWeighting
    % Linearly penalizing component length
    
    df_fromLength = zeros(size(df));
    df_fromLength(nP*nJ+(1:nJ)) = p.lengthObjectiveWeighting;
    
    f = f + p.lengthObjectiveWeighting*sum(x(nP*nJ+(1:nJ)));
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
    dfJoints = zeros(1, nP*nJ);
    dfJoints(1:nJ*(nP-1)) = sin(diff_th);
    dfJoints((nJ+1):nJ*nP) = dfJoints((nJ+1):nJ*nP)...
         - sin(diff_th);
    
    % add gradient component
    df(1:nJ*nP) = df(1:nJ*nP) + dfJoints*p.jointSmoothingWeighting;
    
    % add hessian component
    ddfJoints =  diag([cos(diff_th) zeros(1,nJ)]) +diag([ zeros(1,nJ) cos(diff_th)]) ...
        - diag(cos(diff_th),nJ) - diag(cos(diff_th),nJ).';
    ddf(1:nJ*nP, 1:nJ*nP) = ddf(1:nJ*nP, 1:nJ*nP) ...
        + ddfJoints*p.jointSmoothingWeighting;
    
end


end



plotResults(x, p)

end






