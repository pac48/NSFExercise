function x = getX(robot)
xElbowT = robot.getTransform(5);
xWristT = robot.getTransform(7);
x = [xElbowT(1:3,end);xWristT(1:3,end)];
x([1 4]) = x([1 4])-.1; 
end