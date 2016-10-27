function [x, y, th] = makeArmPoses(n)
% [x, y, th] = makeArmPoses(nPoses);
% makes vectors of an arm pose in SE2
% x [n x 1] = x position of the end effector
% y [n x 1] = y position of the end effector
% th [n x 1] = angle of the end effector wrt horizontal axis

 
% for now, use random points
xMax = 2;
yMax = 2;

x = (rand(n,1)-.5)*2 * xMax;
y = (rand(n,1)-.5)*2 * yMax;
th = rand(n,1)*2*pi;


end