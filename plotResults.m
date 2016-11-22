function plotResults(x, p)
% plot the results


th = reshape(x(1:p.nPoses*p.nJoints),  [p.nJoints, p.nPoses]);
lengths = x(p.nPoses*p.nJoints + (1:p.nJoints));

xJ = zeros(p.nJoints+1,p.nPoses); 
yJ = zeros(p.nJoints+1,p.nPoses); 
thEff = sum(th,1);


rb = [0;0;0];
for m = 1:p.nPoses
 g_st = fkFunc(th(:,m), lengths, rb); % This depends on the existance of an fkFunc function
xJ(:,m) = squeeze(g_st(1,4,:));
yJ(:,m) = squeeze(g_st(2,4,:));
set(p.outArm(m) ,'xdata',xJ(:,m), 'ydata',  yJ(:,m), 'color', p.colors{m});
end

set(p.outScatter, 'xdata', xJ(end,:) , 'ydata', yJ(end,:));
set(p.outAngles, 'xdata', xJ(end,:), 'ydata', yJ(end,:), ...
    'udata', p.arrowLen*cos(thEff), 'vdata', p.arrowLen*sin(thEff));
% axlims = axis;

drawnow;

if p.writeVideo
writeVideo(p.vid, getframe);
end