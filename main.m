% Main function for optimization

% Create workspace poses, obstacles, other parameters
p = []; % parameters structure: includes all non-decision variables
p.nPoses = 5; % number of poses that we are trying to fit
[xd, yd, thd] = makeArmPoses(p.nPoses);
p.xd = xd; p.yd = yd; p.thd = thd;
p.nJoints = 3; % this is fixed for now: The number of actuated joints

% make an initial guess: This will be important since its nonconvex.
x0 = makeInitGuess(p);

% other bounds
% Max and min
angleMax = pi;
angleMin = -pi;
lengthMin = 0;
lengthMax = 10;
ub = [ones(p.nJoints*p.nPoses,1)*angleMax;...
    ones(p.nJoints,1)*lengthMax]; 
lb = [ones(p.nJoints*p.nPoses,1)*angleMin;...
    ones(p.nJoints,1)*lengthMin];
% linear equality and inequality
A = []; b= [];
Aeq = []; beq = [];


% set up problem 
options = optimoptions('fmincon','Display','iter');
problem.options = options;
problem.solver = 'fmincon';
problem.objective = @(x)(objFunc(x,p));
problem.x0 = x0;
problem.ub = ub;
problem.lb = lb;
problem.A = A;
problem.b = b;
problem.Aeq = A;
problem.beq = b;
problem.nonlcon = @(x)(nonlconFunc(x,p));

% run optimization
tic;
[xFinal,fval,exitflag,output,lambda,grad,hessian] = fmincon(problem);
timeToOpt = toc;

% visualize results
plotResults(x, p);

