% jointSlackMath

% slack variable to minimize one of the joints

nJoints = 3;
nPoses = 5;
th = sym('th', [nJoints, nPoses]);

diff_th = [th(:,1)-th(:,2); th(:,2)-th(:,3); th(:,3)-th(:,4); th(:,4)-th(:,5)];

err= sym(zeros(nJoints, nPoses-1));
for r = 1:nJoints
    for c = 1:nPoses-1

M = (eye(3) - R_z(th(r,c)).'*R_z(th(r,c+1)));

err(r,c) = sum(sum((M.^2)));

    end
end

err = simplify(err);
term = err/4 - 1;

syms s
-s-term(:) <= 0;
syms l1 l2 l3 jointSlack
jacobian(-s-term(:), [th(:);l1; l2; l3; s] )
% 0 = jacobian(-s-term(:), th(:)) + [diag(sin(diff_th)), zeros(p.nJoints*(p.nPoses-1), p.nJoints)] - [ zeros(p.nJoints*(p.nPoses-1), p.nJoints) diag(sin(diff_th))]
% jacobian(-s-term(:), th(:)) = -[diag(sin(diff_th)),
% zeros(p.nJoints*(p.nPoses-1), p.nJoints)] + [ zeros(p.nJoints*(p.nPoses-1), p.nJoints) diag(sin(diff_th))]