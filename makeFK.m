% Makes the forward kinematics of a rotary joint arm:
% This means that the output is the symbolic variable g_st
% which is the SE3 transformation of the end effector with respect to the
% inertial frame.
% Julian Whitman, 10/26/2016

nJ = 3; % number of joints
lengths = sym('L', [nJ,1]);
assume(lengths>=0);
th = sym('th', [nJ,1]); 
syms xb yb zb real % the base location variables

%% exponential products formula
% assumes joint angles build off of eachother
w = repmat([0 0 1].', [1,nJ]); % the joint axis: all are in plane for now
q = [[0 cumsum(lengths(1:nJ-1)).']; zeros(2,nJ)]; % each col is a position of a joint at th = 0
xiv = [-cross(w,q); w];

g_st0 = sym(zeros(4,4,nJ+1)); % the position at zero joint angles
for i = 1:nJ+1
g_st0(:,:,i) = [eye(3) [sum(lengths(1:(i-1))); 0; 0]; [0 0 0 1]]; % transform to end effector at zero joint angles
end

% % forward kinematics with exponential formula:
g_st = sym(zeros(4,4,nJ+1));
g_st(:,:,1) = eye(4);
for i = 1:nJ
g_st(:,:,i+1) = g_st(:,:,i) * twistExp(xiv(:,i), th(i));
end
for i = 1:nJ+1
g_st(:,:,i) = g_st(:,:,i) * g_st0(:,:,i);
end
g_st = simplify(g_st);
g_st = g_st + repmat([zeros(3,3), [xb;yb;zb;]; 0 0 0 0], [1,1,nJ+1]);

% Here we now have:
% g_st(:,:,1) = base frame wrt inertial
% g_st(:,:,1+j) =  frame of output of link j wrt inertial
% g_st(:,:,end) = frame of end effector wrt inertial




