function plotResults(x, p)
% plot the results


th = reshape(x(1:p.nJoints*p.nPoses),  [p.nJoints, p.nPoses]);
lengths = x((1:p.nJoints*p.nPoses)+p.nJoints);

xJ = zeros(p.nJoints+1,p.nPoses); 
yJ = zeros(p.nJoints+1,p.nPoses); 

if p.variableBase&&p.variableEnd
    effOffset = x(p.nJoints*p.nPoses+p.nJoints +3);
        thEff = sum(th,1) + effOffset;

elseif p.variableBase&&(~p.variableEnd)
    effOffset = x(p.nJoints*p.nPoses+p.nJoints +1);
    thEff = sum(th,1) + effOffset;

else
    thEff = sum(th,1);
effOffset = 0;
end


if p.variableBase
rb = [x(p.nJoints*p.nPoses+p.nJoints+(1:2)); 0];
else
rb = [0;0;0];
end


for m = 1:p.nPoses
 g_st = fkFunc(th(:,m), lengths, rb, effOffset); % This depends on the existance of an fkFunc function
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