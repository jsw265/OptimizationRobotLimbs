
nJoints = 3;
nPoses = 5;
th = sym('th', [nJoints, nPoses]);
err= sym(zeros(nJoints, nPoses-1));
for r = 1:nJoints
    for c = 1:nPoses-1

M = (eye(3) - R_z(th(r,c)).'*R_z(th(r,c+1)));

err(r,c) = sum(sum((M.^2)));

    end
end

err = simplify(err);

f = sum(sum(err))/4;

df = jacobian(f, th(:));
df.'

ddf = jacobian(df, th(:))

% 0 = ddf - diag([cos(diff_th); 0; 0; 0]) - diag([0; 0; 0; cos(diff_th)]) + diag(cos(diff_th),3) + diag(cos(diff_th),3).'
% ddf = diag([cos(diff_th); 0; 0; 0]) +diag([0; 0; 0; cos(diff_th)]) - diag(cos(diff_th),3) - diag(cos(diff_th),3).'