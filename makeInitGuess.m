function x0 = makeInitGuess(p)
% x0 = makeInitGuess(p);
% make the initial guess: this matters for nonconvex problems.

nP = p.nPoses;
nJ = p.nJoints;
% use first angles that are just pointing at the points
% th01 = atan2(p.yd, p.xd);
% th0 = [th01, zeros(nP, nJ-1)].';

th0 = zeros(nJ, nP);

l1_0 = mean(sqrt(p.xd.^2 + p.yd.^2));
l0 = [l1_0; zeros(nJ-1, 1)];

% x = [thjoint1_pose1, th2_1, th3_1, th2_1, ...  lengths];
x0 = [th0(:); l0];

if p.variableBase
    x0 = [x0; 0;0];
end
if p.variableEnd
    x0 = [x0; 0];
end

    

end