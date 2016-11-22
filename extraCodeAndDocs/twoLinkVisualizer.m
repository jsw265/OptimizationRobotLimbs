% twoLinkVisualizer:
% Julian Whitman 10/29/2016
% Plot out the landscape of optimal-IK-cost, for 2 link arm.
% x axis L1, y axis L2, z axis cost value for best IK for all the poses.

close all;
% Create workspace poses, obstacles, other parameters
p = []; % parameters structure: includes all non-decision variables
p.nJoints = 2; % this is fixed for now: The number of actuated joints
p.nPoses = 5; % number of poses that we are trying to fit
[xd, yd, thd] = makeArmPoses(p.nPoses);
p.xd = xd; p.yd = yd; p.thd = thd;
p.nVars = p.nPoses*p.nJoints+p.nJoints;
Td = [p.xd p.yd p.thd].';
rb = [0;0;0];
% make an initial guess: This will be important since its nonconvex.
x0 = makeInitGuess(p);
th0 = reshape(x0(1:p.nPoses*p.nJoints), [p.nJoints, p.nPoses]);

% make a meshgrid of points to test
lMax = 3;
lMin = 0;
nSamples = 15;
l1list = linspace(lMin, lMax, nSamples);
l2list = linspace(lMin, lMax, nSamples);
[L1, L2] = meshgrid(l1list, l2list);

% initial empty matrices
F = zeros(nSamples, nSamples);
dF = zeros(nSamples,nSamples,p.nVars);
th = zeros(p.nJoints, p.nPoses);
startPlot;
options = optimoptions('fmincon', 'Display', 'off');

% go through the grid and solve for cost of best IK
for i = 1:nSamples
    for j = 1:nSamples
                    lengths = [L1(i,j); L2(i,j)];

        for k = 1:p.nPoses;
            % solve for best angles for each desired pose
%              fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options)
            [th_k, fval] = fmincon( @(t)(fFunc(t, lengths, rb, Td(:,k))),...
                                      th0(:,k), ...
                                      [], [], [], [], [], [], [], options);
            th(:,k) = th_k;
        end
        % reshape into 'x' decision vector
        x = [th(:); lengths];
      [f, df, ddf] = objFunc(x,p);
       F(i,j) = f;
       dF(i,j,:) = df;
    end
end

figure;
surf(L1, L2, F); 
xlabel('Length 1'); ylabel('Length 2');
zlabel('Cost with optimal IK');



