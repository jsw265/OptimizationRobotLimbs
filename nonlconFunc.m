function  [c,ceq,gradc,gradceq] = nonlconFunc(x,p)
% function  [c,ceq,gradc,gradceq] = nonlconFunc(x,p)
% constriant function of input x [n, 1] which returns
% the c<=0, its gradient, ceq==0, and its gradient wrt x.
% Given Parameters structure p

th = reshape(x(1:p.nPoses*p.nJoints), [p.nJoints, p.nPoses]);
lengths = x(p.nPoses*p.nJoints+1 : end);
rb = [0;0;0];
% To do: add these if needed
tau = zeros(p.nPoses*p.nJoints,1);
for i = 1:p.nPoses
    g_temp = g_torqueFunc(th(:,i), lengths, rb);
    tau(1+3*(i-1):3*i) = g_temp(1:3);
    g_temp = g_torqueFunc(th(:,i), lengths, rb);
    dtau(1+3*(i-1):3*i) = g_temp(1:3);
end
c=tau-p.jointMaxTorque*ones(p.nPoses*p.nJoints,1);
ceq=[];
gradc =;
gradceq =[];





end