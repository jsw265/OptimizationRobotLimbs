% Makes the forward kinematics of a rotary joint arm:
% This means that the output is the symbolic variable g_st
% which is the SE3 transformation of the end effector with respect to the
% inertial frame.
% Julian Whitman, 10/26/2016


for nJ = 1:5

disp(['calculating symbolics for nJ = '  num2str(nJ)]);

lengths = sym('L', [nJ,1]);
assume(lengths>=0);
th = sym('th', [nJ,1]); 
syms xb yb zb effOffset real % the base location variables and end effector offset

%% exponential products formula
% assumes joint angles build off of eachother
w = repmat([0 0 1].', [1,nJ]); % the joint axis: all are in plane for now
if nJ>1
q = [[0 cumsum(lengths(1:nJ-1)).']; zeros(2,nJ)]; % each col is a position of a joint at th = 0
else
q = [0 ; 0;0];
end
xiv = [-cross(w,q); w];

g_st0 = sym(zeros(4,4,nJ+1)); % the position at zero joint angles
for i = 1:nJ+1
g_st0(:,:,i) = [eye(3) [sum(lengths(1:(i-1))); 0; 0]; [0 0 0 1]]; % transform to end effector at zero joint angles
end
g_st0(1:3,1:3, end) =  R_z(effOffset)*g_st0(1:3,1:3, end);


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

%% Make gradients with respect to angles and lengths
v = [th; lengths];
xEff = g_st(1,4,end);
yEff = g_st(2,4,end);
REff = g_st(1:3,1:3,end);
syms xd yd thd
Td = [xd; yd; thd]; % the desired "pose" of the end effector
rb = [xb; yb; zb]; % the base location
Rd = rotmat([0;0;1], thd);
M = eye(3) - REff*Rd.';
f = (xd - xEff)^2 + (yd - yEff)^2 + sum(sum(M.^2));
dfdv = jacobian(f, v); % gradient
ddfddv = jacobian(dfdv,v); % hessian

% gradients wrt th
dfdth = jacobian(f, th); % gradient
ddfddth = jacobian(dfdth, th); % hessian

% gradients wrt lengths
dfdl = jacobian(f, lengths); % gradient
ddfddl = jacobian(dfdl, lengths); % hessian

% gradients wrt base location
dfdrb = jacobian(f, rb); % gradient
ddfddrb = jacobian(dfdrb, rb); % hessian

% gradients wrt end eff offset
dfdeffOffset = jacobian(f, effOffset); % gradient
ddfddeffOffset = jacobian(dfdeffOffset, effOffset); % hessian

%% here's how we can make this into an optimized matlab function:
% fFunc = matlabFunction(f, 'vars', {th, lengths, rb, Td});
% dfdvFunc = matlabFunction(dfdv, 'vars', {th, lengths, rb, Td});
% ddfddvFunc = matlabFunction(ddfddv, 'vars', {th, lengths, rb, Td});

disp(['writing files for nJ = '  num2str(nJ)]);

% note that these could also be written to a file
fkFunc = matlabFunction(g_st, 'File', ['fkFunc' num2str(nJ)], 'vars', {th, lengths, rb, effOffset});
fFunc = matlabFunction(f, 'File', ['fFunc' num2str(nJ)], 'vars', {th, lengths, rb, effOffset Td});
dfdthFunc = matlabFunction(dfdth, 'File', ['dfdthFunc' num2str(nJ)], 'vars', {th, lengths, rb, effOffset Td});
dfdlFunc = matlabFunction(dfdl, 'File', ['dfdlFunc' num2str(nJ)], 'vars', {th, lengths, rb, effOffset Td});
dfdrbFunc = matlabFunction(dfdrb, 'File', ['dfdrbFunc' num2str(nJ)], 'vars', {th, lengths, rb, effOffset Td});
dfdeffOffsetFunc = matlabFunction(dfdeffOffset, 'File', ['dfdeffOffsetFunc' num2str(nJ)], 'vars', {th, lengths, rb, effOffset Td});
ddfddthFunc = matlabFunction(ddfddth, 'File', ['ddfddthFunc' num2str(nJ)], 'vars', {th, lengths, rb, effOffset Td});
ddfddlFunc = matlabFunction(ddfddl, 'File', ['ddfddlFunc' num2str(nJ)], 'vars', {th, lengths, rb, effOffset Td});
ddfddrbFunc = matlabFunction(ddfddrb, 'File', ['ddfddrbFunc' num2str(nJ)], 'vars', {th, lengths, rb, effOffset Td});
ddfddeffOffsetFunc = matlabFunction(ddfddeffOffset, 'File', ['ddfddeffOffsetFunc' num2str(nJ)], 'vars', {th, lengths, rb, effOffset Td});

end
disp('Done');