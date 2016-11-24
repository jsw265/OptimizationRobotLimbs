function  [c,ceq,gradc,gradceq] = nonlconFunc(x,p)
% function  [c,ceq,gradc,gradceq] = nonlconFunc(x,p)
% constriant function of input x [n, 1] which returns
% the c<=0, its gradient, ceq==0, and its gradient wrt x.
% Given Parameters structure p

nJ = p.nJoints;
c = [];
gradc = [];
    th = reshape(x(1:p.nPoses*nJ), [nJ, p.nPoses]);

%% the gradc here is the wrong size
if 0 %p.useTorqueConstraint
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
    c = tau-p.jointMaxTorque*ones(p.nPoses*nJ,1);
    gradc = dtau;
    
end

if p.slackUseJointWeighting
    % example for what this constraint looks like, for 5 poses 3 joints
    %  4*cos(th1_1 - th1_2) - s - 4 <= 0
    %  4*cos(th2_1 - th2_2) - s - 4 <= 0
    %  4*cos(th3_1 - th3_2) - s - 4 <= 0
    %  4*cos(th1_2 - th1_3) - s - 4 <= 0
    %  4*cos(th2_2 - th2_3) - s - 4 <= 0
    %  4*cos(th3_2 - th3_3) - s - 4 <= 0
    %  4*cos(th1_3 - th1_4) - s - 4 <= 0
    %  4*cos(th2_3 - th2_4) - s - 4 <= 0
    %  4*cos(th3_3 - th3_4) - s - 4 <= 0
    %  4*cos(th1_4 - th1_5) - s - 4 <= 0
    %  4*cos(th2_4 - th2_5) - s - 4 <= 0
    %  4*cos(th3_4 - th3_5) - s - 4 <= 0
    jointSlack = x(end);
    diff_th = diff(th, 1, 2);
    diff_th = diff_th(:);
    cSlack = cos(diff_th) - jointSlack - 1; %  rescaled by 4
    c = [c; cSlack];
    
    gradcSlack = -[diag(sin(diff_th)) zeros(p.nJoints*(p.nPoses-1), p.nJoints)] + ...
                  [ zeros(p.nJoints*(p.nPoses-1), p.nJoints) diag(sin(diff_th))];
   gradcSlack = [gradcSlack zeros(p.nJoints*(p.nPoses-1), p.nJoints)...
                                  -ones(p.nJoints*(p.nPoses-1),1)];
              
    gradc = [gradc; gradcSlack];
end

ceq=[];
gradceq =[];


end