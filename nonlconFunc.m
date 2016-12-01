function  [c,ceq,gradc,gradceq] = nonlconFunc(x,p)
% function  [c,ceq,gradc,gradceq] = nonlconFunc(x,p)
% constriant function of input x [n, 1] which returns
% the c<=0, its gradient, ceq==0, and its gradient wrt x.
% Given Parameters structure p
nJ = p.nJoints;
nP = p.nPoses;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



if p.useTorqueConstraint
    th = reshape(x(1:nP*nJ), [nJ, nP]);
    lengths = x(nP*nJ+1 : end);
    rb = [0;0;0];
    % To do: add these if needed
    tau = zeros(1,nP*nJ);
    dtau = zeros(nJ*(nP+1),nJ*nP);
    if p.variableBase
        dtau = [dtau; zeros(2,nJ*nP)];
    end
    if p.variableEnd
        dtau = [dtau; zeros(1,nJ*nP)];
    end
    for i = 1:nP
        tau(1+nJ*(i-1):nJ*i) = g_torqueFunc(th(:,i), lengths, rb);
        g_temp = gd_torqueFunc(th(:,i), lengths, rb)';
        dtau(nJ*(i-1)+1:nJ*i,1+nJ*(i-1):nJ*i) = g_temp(1:nJ,1:nJ);
        dtau(nJ*nP+1:nJ*(nP+1),1+nJ*(i-1):nJ*i) = g_temp(nJ+1:2*nJ,1:nJ);
    end
    c=tau-p.jointMaxTorque*ones(1,nP*nJ);
    c=[c -tau-p.jointMaxTorque*ones(1,nP*nJ)];
    gradc = dtau;
    gradc = [gradc -dtau];
    if p.IneqConstr
        for i = 1:nP
            f_i = fFunc(th(:,i), lengths, rb, 0, Td(:,i)) - p.IneqConstrVal;
            c=[c f_i];
            
            dfdth_i = dfdthFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
            dfdl_i = dfdlFunc(th(:,i), lengths, rb, effOffset, Td(:,i));
            df_i = zeros(1,nVars);
            df_i( (i*nJ - (nJ-1)):(i*nJ) ) = dfdth_i;
            df_i( nJ*nP+(1:nJ)) = dfdl_i;
            
%             sc=size(c)
%             sgc=size(gradc)
%             sdf=size(df_i)
%             %keyboard;
            
            gradc=[gradc df_i'];
        end
    end
else
    c = [];
    gradc =[];
end
ceq=[];
gradceq =[];

end