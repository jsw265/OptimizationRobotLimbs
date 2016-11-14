figure('name', 'Optimization Visualization', 'position', [50 50 800 800]);


arrowLen = .5;
axLims = [-1 1 -1 1]*(max(abs([p.xd; p.yd]))*1.5 + arrowLen);
colors  = {'r', [1 .5 0], 'y', 'g', 'b',  [0.5 0 0.5]};
while (length(colors)<p.nPoses)
   colors = {colors{:} colors{:}};
end
p.colors = colors(1:p.nPoses);


outScatter = scatter(0,0,'k', 'marker', 'x'); hold on;
targetScatter = zeros(1,p.nPoses);
for m = 1:p.nPoses
targetScatter(m) = scatter(p.xd(m), p.yd(m), 'k');
set(targetScatter(m), 'MarkerFaceColor', p.colors{m})
end

targetQuiver = quiver(p.xd, p.yd, arrowLen*cos(p.thd),arrowLen*sin(p.thd), 0, 'k');
outAngles = quiver(0, 0, ...
            0,0, 0, 'k', ...
            'LineStyle', ':');
outArm = zeros(1,p.nPoses);
for m=1:p.nPoses
   outArm(m) = plot( 0, 0, 'lineWidth', 2);
end

p.arrowLen = arrowLen;
p.outAngles = outAngles;
p.outScatter = outScatter;
p.outArm = outArm;

axis(axLims);


if p.writeVideo
p.vid = VideoWriter('outputVideo.mp4', 'MPEG-4');
open(p.vid)
end
