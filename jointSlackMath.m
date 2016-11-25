% jointSlackMath

% slack variable to minimize one of the joints

nJoints = 4;
nPoses = 5;
th = sym('th', [nJoints, nPoses]);

diff_th = -[th(:,1)-th(:,2)  th(:,2)-th(:,3)  th(:,3)-th(:,4)  th(:,4)-th(:,5)];

err= sym(zeros(nJoints, nPoses-1));
for r = 1:nJoints
    for c = 1:nPoses-1

M = (eye(3) - R_z(th(r,c)).'*R_z(th(r,c+1)));

err(r,c) = sum(sum((M.^2)));

    end
end

err = simplify(err);
term = sum(err,2)/4;

syms s
c= -s-term(:);
syms l1 l2 l3
x = [th(:);l1; l2; l3; s];
Jd = jacobian(-s-term(:), x)

% OLD:
% 0 = jacobian(-s-term(:), th(:)) + [diag(sin(diff_th)), zeros(nJ*(nP-1), nJ)] - [ zeros(nJ*(nP-1), nJ) diag(sin(diff_th))]
% jacobian(-s-term(:), th(:)) = -[diag(sin(diff_th)),
% zeros(nJ*(nP-1), nJ)] + [ zeros(nJ*(nP-1), nJ) diag(sin(diff_th))]

nJ = 4;
nP = 5;
% NEW:
% to do: make this without a for loop?
J = sym(zeros(nJ, length(x)));
for i =1:nJ
J(i,i:nJ:nJ*(nP-1)) = sin(diff_th(i,:));
% J(2,2:nJ:nJ*(nP-1)) = -sin(diff_th(2,:));
% J(3,3:nJ:nJ*(nP-1)) = -sin(diff_th(3,:));
J(i,(i+nJ):nJ:nJ*nP) = J(i,(i+nJ):nJ:nJ*nP) - sin(diff_th(i,:));
% J(2,(2+nJ):nJ:nJ*nP) = J(2,(2+nJ):nJ:nJ*nP) + sin(diff_th(2,:));
% J(3,(3+nJ):nJ:nJ*nP) = J(3,(3+nJ):nJ:nJ*nP) + sin(diff_th(3,:));
end
J(:,end) = -1;

Jd - J

%% It looks like this is minimizing the minimum of the difference of joint angles,
% Like cos(diff(th)) second row is all the same.
thTest = sym(zeros(size(th)));
thTest(:,2:2:end) = pi;

subs(err, th, thTest)







