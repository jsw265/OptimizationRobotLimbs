% Main function for optimization

% To do: make function that measures joint torques as a function of link
%        length, number of joints, joint angles, and gravity direction
% To do: add weights to each pose part. we may only care about end effector
%        angle or location exactly at waypoints like the start and end points.
% To do: make a neat/organized/thoughtful way to change the number of objectives and variables

close all;
addpath(genpath(pwd)); % adds all subfolders in this directory to path
global p

% Create workspace poses, obstacles, other parameters
p = []; % parameters structure: includes all non-decision variables
p.nPoses = 5; % number of poses that we are trying to fit
p.nJoints = 2; % fixed for now: The number of actuated joints
p.nJoints = min(p.nJoints, 6); % 5 is the max for now.
p.nRestarts = 3; % Number of random initial conditions to try, conflicts with useLastInitialGuess

% other options? Which functions to use, etc
p.writeVideo = false; % A flag to say whether to make a video
p.positionErrorObjectiveWeighting = 1.01; % May want to split into rotational and positional error
p.lengthObjectiveWeighting = 0.0275;
p.useTorqueObjective = 0;
p.useTorqueConstraint = 0;
p.jointSmoothingWeighting = 0;%0.0001;
p.variableBase = true; % allows base location xb,yb to vary
p.variableEnd = true; % allows the end effector fixed angle wrt the last link to vary
p.useGradient = true; % use the gradient in fmincon
p.randomStart = true;    % we want to pick a random initial guess
p.useLastTargets = true; % load and reuse the lastSoln.mat, use same xd yd thd
p.useLastInitialGuess = false; % make a new initial guess

% physical parameters: will be used in extra objectives and constraints
p.jointMass = .36; % kg, X-9 module mass (heaviest of the series)
p.jointMaxTorque = 9; % N-m, the Continuous torque output of X-9 module (strongest of the series)
p.gravity = [0;-9.81;0]; % gravitational acceleration vector in -y direction
p.linkMassPerLength = .425; % kg/m for thicker pipe. 0.226 kg/m if thinner pipe.

% other bounds
% Max and min
angleMax = Inf;
angleMin = -Inf;
lengthMin = 0;
lengthMax = 5;
ub = [ones(p.nJoints*p.nPoses,1)*angleMax;...
    ones(p.nJoints,1)*lengthMax];
lb = [ones(p.nJoints*p.nPoses,1)*angleMin;...
    ones(p.nJoints,1)*lengthMin];
if p.variableBase
    lb = [lb; -lengthMax*p.nJoints;-lengthMax*p.nJoints];
    ub = [ub; lengthMax*p.nJoints;lengthMax*p.nJoints];
end
if p.variableEnd
    lb = [lb; -Inf];
    ub = [ub; Inf];
end

%% logic chain to load all or parts of the last solution
if (p.useLastTargets||p.useLastInitialGuess)&&exist('lastSoln.mat')
    S = load('lastSoln.mat');
    if p.useLastTargets
        xd = S.xd;  yd = S.yd;  thd = S.thd;
    else
        [xd, yd, thd] = makeArmPoses(p.nPoses);
    end
    if p.useLastInitialGuess
        x0 = S.x0;
    else
        x0 = makeInitGuess(p, lb, ub);
    end
else
    [xd, yd, thd] = makeArmPoses(p.nPoses);
    % make an initial guess: This will be important since its nonconvex.
    x0 = makeInitGuess(p, lb, ub);
end
p.xd = xd; p.yd = yd; p.thd = thd;


% linear equality and inequality
A = []; b= [];
Aeq = []; beq = [];

% initial conditions plot
startPlot;
plotResults(x0(:,1), p);

% disp('Press any key to start');
% pause;
disp('Optimizing...');

% set up problem
options = optimoptions('fmincon',...
    'Algorithm','interior-point',...
    'SpecifyObjectiveGradient',p.useGradient);
problem.options = options;
problem.solver = 'fmincon';
problem.objective = @(x)(objFunc(x,p));
problem.x0 = x0(:,1);
problem.ub = ub;
problem.lb = lb;
problem.A = A;
problem.b = b;
problem.Aeq = A;
problem.beq = b;
problem.nonlcon = @(x)(nonlconFunc(x,p));

% run optimization
tic;
bestValue = inf;
for iRestart = 1:size(x0,2)
    problem.x0 = x0(:,iRestart);
    [tempx,tempfval,tempexitflag,tempoutput,templambda,tempgrad,temphessian] = fmincon(problem);
    disp(tempfval)
    if tempfval < bestValue
        x = tempx;
        fval = tempfval;
        exitflag = tempexitflag;
        foutput = tempoutput;
        lambda = templambda;
        grad = tempgrad;
        hessian = temphessian;
    end
end
timeToOpt = toc;

% visualize results
plotResults(x, p);

disp('Lengths:')
disp(num2str(x(p.nJoints*p.nPoses+(1:p.nJoints))));

th = reshape(x(1:p.nPoses*p.nJoints), [p.nJoints, p.nPoses]);
disp('Angles:')
disp(num2str(mod(th, 2*pi)));

if p.variableBase
    disp('Base:')
    disp(x(p.nJoints*p.nPoses+p.nJoints+(1:2)).');
end

if p.variableEnd
    disp('End Eff Offset:')
    if p.variableBase
        disp(x(p.nJoints*p.nPoses+p.nJoints+3));
    else
        disp(x(p.nJoints*p.nPoses+p.nJoints+1));
    end
end
disp(['Function Value: ' num2str(fval)])

if p.writeVideo
    close(p.vid);
end
save('lastSoln.mat');