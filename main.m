% Main function for optimization
close all;

% Create workspace poses, obstacles, other parameters
p = []; % parameters structure: includes all non-decision variables
p.nPoses = 5; % number of poses that we are trying to fit
[xd, yd, thd] = makeArmPoses(p.nPoses);
p.xd = xd; p.yd = yd; p.thd = thd;
p.nJoints = 2; % this is fixed for now: The number of actuated joints

% physical parameters: will be used in extra objectives and constraints
p.jointMass = .36; % kg, X-9 module mass (heaviest of the series)
p.jointMaxTorque = 9; % N-m, the Continuous torque output of X-9 module (strongest of the series)
p.gravity = [0;-9.81;0]; % gravitational acceleration vector in -y direction

% make an initial guess: This will be important since its nonconvex.
x0 = makeInitGuess(p);


% other bounds
% Max and min
angleMax = Inf;
angleMin = -Inf;
lengthMin = 0;
lengthMax = 10;
ub = [ones(p.nJoints*p.nPoses,1)*angleMax;...
    ones(p.nJoints,1)*lengthMax]; 
lb = [ones(p.nJoints*p.nPoses,1)*angleMin;...
    ones(p.nJoints,1)*lengthMin];
% linear equality and inequality
A = []; b= [];
Aeq = []; beq = [];

% initial conditions plot
startPlot;
plotResults(x0, p);
% disp('Press any key to start');
% pause;
disp('Optimizing...');

% set up problem 
options = optimoptions('fmincon');
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
[x,fval,exitflag,output,lambda,grad,hessian] = fmincon(problem);
timeToOpt = toc;

% visualize results
plotResults(x, p);

disp('Lengths:')
disp(num2str(x(p.nJoints*p.nPoses+1:end)));
