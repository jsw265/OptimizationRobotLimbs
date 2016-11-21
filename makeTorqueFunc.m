% Ky Woodard, 11/19/2016

if ~exist('nJ')
nJ = 3; % number of joints
end

gravity = 9.81;
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
    % Transformation to the second to last joint
    %%% Not the same FK transform as in the makeFK funciton%%%
    g_st0(:,:,i) = [eye(3) [sum(lengths(1:(i-2))); 0; 0]; [0 0 0 1]]; 
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

%% Euler-Lagrangian Dynamics
v = [th; lengths];
rb = [xb; yb; zb]; % the base location

g_com = sym(eye(4));
g_trans = sym(eye(4));
g_st_com = sym(zeros(4,4,nJ));
for i = 1:nJ-1
    g_com(1:3,4) = [(p.linkMassPerLength*lengths(i)^2/2+p.jointMass*...
        lengths(i))/(p.linkMassPerLength*lengths(i)+p.jointMass);0;0];
    g_st_com(:,:,i) = g_st(:,:,i+1)*g_com;
end
g_com(1:3,4) = [lengths(nJ)/2;0;0];
g_st_com(:,:,nJ) = g_st(:,:,nJ+1)*g_com;

PE = 0;
for i=1:nJ-1
    PE = PE+gravity*(p.linkMassPerLength*lengths(i)+p.jointMass)...
        *g_st_com(2,4,i);
end
PE = PE+gravity*(p.linkMassPerLength*lengths(nJ))...
        *g_st_com(2,4,nJ);
Lagrangian = PE;
tau = jacobian(Lagrangian,v)';
dtau = jacobian(tau,v);
f_tau = sum(tau.^2);

p.tau = tau;
p.dtau = dtau;

% Gradient and Hessian with respect to torque (tau)
df_torquedth = jacobian(f_tau, th); % Gradient
ddf_torqueddth = jacobian(df_torquedth, th); % Hessian
df_torquedl = jacobian(f_tau, lengths); % Gradient
ddf_torqueddl = jacobian(df_torquedl, lengths); % Hessian

g_torqueFunc = matlabFunction(tau, 'File', 'g_torqueFunc', 'vars', {th, lengths, rb});
f_torqueFunc = matlabFunction(f_tau, 'File', 'f_torqueFunc', 'vars', {th, lengths, rb});
df_torquedthFunc = matlabFunction(df_torquedth, 'File', 'df_torquedthFunc', 'vars', {th, lengths, rb});
ddf_torqueddthFunc = matlabFunction(ddf_torqueddth, 'File', 'ddf_torqueddthFunc', 'vars', {th, lengths, rb});
df_torquedlFunc = matlabFunction(df_torquedl, 'File', 'df_torquedlFunc', 'vars', {th, lengths, rb});
ddf_torqueddlFunc = matlabFunction(ddf_torqueddl, 'File', 'ddf_torqueddlFunc', 'vars', {th, lengths, rb});