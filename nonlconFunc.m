function  [c,ceq,gradc,gradceq] = nonlconFunc(x,p)
% function  [c,ceq,gradc,gradceq] = nonlconFunc(x,p)
% constriant function of input x [n, 1] which returns
% the c<=0, its gradient, ceq==0, and its gradient wrt x.
% Given Parameters structure p
nJ = p.nJoints;
nP = p.nPoses;

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
else
    c = [];
    gradc =[];
end
ceq=[];
gradceq =[];

end