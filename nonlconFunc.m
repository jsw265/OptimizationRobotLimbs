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
    tau = zeros(1,p.nPoses*nJ);
    for i = 1:p.nPoses
        g_temp = g_torqueFunc(th(:,i), lengths, rb);
        tau(1+nJ*(i-1):nJ*i) = g_temp;
        dtau(1:nJ,1+2*nJ*(i-1):2*nJ*i) = gd_torqueFunc(th(:,i), lengths, rb);
    end
    c=tau-p.jointMaxTorque*ones(1,p.nPoses*nJ);
    gradc =dtau;
else
    c = [];
    gradc =[];
end
ceq=[];
gradceq =[];

end