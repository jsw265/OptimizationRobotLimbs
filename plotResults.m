
% plot the results
arrowLen = 1;
axLims = [-1 1 -1 1]*(max(abs([p.xd; p.yd])) + arrowLen);

th = reshape(xFinal(1:end-nLinks),  [p.nJoints, p.nPoses]).';
lenghts = xEff(end-nLinks+1:end);
xEff = zeros(nPts,nLinks); 
yEff = zeros(nPts,nLinks); 
thEff = th(:,end);
 xEff(:,1) = lenghts(1)*cos(th(:,1));
 yEff(:,1) = lenghts(1)*sin(th(:,1));
for j = 2:nLinks
   xEff(:,j) = xEff(:,j-1) + lenghts(j)*cos(th(:,j));
   yEff(:,j) = yEff(:,j-1) + lenghts(j)*sin(th(:,j));
end

outScatter = scatter(0,0,'g', 'marker', 'x');
outAngles = quiver(xOut(:,end), yOut(:,end), ...
             arrowLen*cos(theOut),arrowLen*sin(theOut), 0, 'g');
outArm = zeros(1,nPts);
for m=1:nPts
   outArm(m) = plot( [0 xOut(m,:)], [0 yOut(m,:)], 'g');
end


for m=1:p.nPoses
   set(outArm(m) ,'xdata', [0 xEff(m,:)], 'ydata', [0  yEff(m,:)]);
end
set(outScatter, 'xdata', xEff(:,end) , 'ydata', yEff(:,end));
set(outAngles, 'xdata', xEff(:,end), 'ydata', yEff(:,end), ...
    'udata', arrowLen*cos(thEff), 'vdata', arrowLen*sin(thEff));
axis(axLims);