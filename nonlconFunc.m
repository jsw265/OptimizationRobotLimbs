function  [c,ceq,gradc,gradceq] = nonlconFunc(x,p)
% function  [c,ceq,gradc,gradceq] = nonlconFunc(x,p)
% constriant function of input x [n, 1] which returns
% the c<=0, its gradient, ceq==0, and its gradient wrt x.
% Given Parameters structure p

c = [];
gradc = [];
ceq=[];
gradceq =[];

nJ = p.nJoints;
nP = p.nPoses;
th = reshape(x(1:nP*nJ), [nJ, nP]);

% %% the gradc here is the wrong size
% if 0 %p.useTorqueConstraint
%     lengths = x(nP*nJ+1 : end);
%     rb = [0;0;0];
%     % To do: add these if needed
%     tau = zeros(nP*nJ,1);
%     for i = 1:nP
%         g_temp = g_torqueFunc(th(:,i), lengths, rb);
%         tau(1+nJ*(i-1):nJ*i) = g_temp(1:nJ);
%         g_temp = gd_torqueFunc(th(:,i), lengths, rb);
%         dtau(1+nJ*(i-1):nJ*i) = g_temp(1:nJ);
%     end
%     c = tau-p.jointMaxTorque*ones(nP*nJ,1);
%     gradc = dtau;
%     
% end

if p.slackUseJointWeighting

    jointSlack = x(end);
% diff_th = -[th(:,1)-th(:,2)  th(:,2)-th(:,3)  th(:,3)-th(:,4)  th(:,4)-th(:,5)];
   
    diff_th = diff(th, 1, 2);

    cSlack = sum(cos(diff_th),2) - jointSlack - (nP-1);
    c = [c; cSlack];
    
    gradcSlack = (zeros(nJ, length(x)));
    for i = 1:nJ
        gradcSlack(i,i:nJ:nJ*(nP-1)) = sin(diff_th(i,:));
        gradcSlack(i,(i+nJ):nJ:nJ*nP) = gradcSlack(i,(i+nJ):nJ:nJ*nP) - sin(diff_th(i,:));
    end
    gradcSlack(:,end) = -1;
    
    gradc = [gradc; gradcSlack];
end

end