% Main function for optimization

% To TEST: add xbase,ybase as design decision variables
% To TEST: add static end effector angle offset as design decision variable
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
[xd, yd, thd] = makeArmPoses(p.nPoses);
p.xd = xd; p.yd = yd; p.thd = thd;
p.nJoints = 1; % fixed for now: The number of actuated joints

p.nJoints = min(p.nJoints, 5); % 5 is the max for now.

% other options? Which functions to use, etc
p.writeVideo = false; % A flag to say whether to make a video
p.positionErrorObjectiveWeighting = 1.01; % May want to split into rotational and positional error
p.lengthObjectiveWeighting = 0.0275;
p.useTorqueObjective = 0;
p.useTorqueConstraint = 0;
p.jointSmoothingWeighting = 0;%0.0001;
p.variableBase = false; % allows base location xb,yb to vary
p.variableEnd = true; % allows the end effector fixed angle wrt the last link to vary
p.useGradient = true; % use the gradient in fmincon

% physical parameters: will be used in extra objectives and constraints
p.jointMass = .36; % kg, X-9 module mass (heaviest of the series)
p.jointMaxTorque = 9; % N-m, the Continuous torque output of X-9 module (strongest of the series)
p.gravity = [0;-9.81;0]; % gravitational acceleration vector in -y direction
p.linkMassPerLength = .425; % kg/m for thicker pipe. 0.226 kg/m if thinner pipe.


% make an initial guess: This will be important since its nonconvex.
x0 = makeInitGuess(p);

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
options = optimoptions('fmincon',...
    'Algorithm','interior-point',...
    'SpecifyObjectiveGradient',p.useGradient);
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

if p.writeVideo
    close(p.vid);
end
