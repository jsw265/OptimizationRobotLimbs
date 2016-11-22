function  [c,ceq,gradc,gradceq] = nonlconFunc(x,p)
% function  [c,ceq,gradc,gradceq] = nonlconFunc(x,p)
% constriant function of input x [n, 1] which returns
% the c<=0, its gradient, ceq==0, and its gradient wrt x.
% Given Parameters structure p
nJ = p.nJoints;
if p.useTorqueConstraint
    th = reshape(x(1:p.nPoses*nJ), [nJ, p.nPoses]);
    lengths = x(p.nPoses*nJ+1 : end);
    rb = [0;0;0];
    % To do: add these if needed
    tau = zeros(p.nPoses*nJ,1);
    for i = 1:p.nPoses
        g_temp = g_torqueFunc(th(:,i), lengths, rb);
        tau(1+nJ*(i-1):nJ*i) = g_temp(1:nJ);
        g_temp = gd_torqueFunc(th(:,i), lengths, rb);
        dtau(1+nJ*(i-1):nJ*i) = g_temp(1:nJ);
    end
    c=tau-p.jointMaxTorque*ones(p.nPoses*nJ,1);
    gradc =dtau;
else
    c = [];
    gradc =[];
end
ceq=[];
gradceq =[];


end