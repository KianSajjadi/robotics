clf
robot = HansCute;
q = zeros(1, 7);
robot.model.animate(q);
breadTr = transl(0.1, 0.1, 0);
bread = Bread(breadTr);
f2bread = transl(0, 0, 0.05) * trotx(pi);
testAnimate(robot, q, bread, breadTr, f2bread);



%% Main Function
%Used to run demonstration
function testAnimate(robot, startQ, bread, breadTr, f2bread)
    qVelocities = zeros(1,7);
    q = startQ;
   %first location above toast
    numSteps = 120;
    endEffectorLocationTr = transl(0.1, 0.1, 0.2) * trotx(pi);
	goalJoints = robot.model.ikine(endEffectorLocationTr, q, [1 1 1 1 1 1]);
    qMatrix = robot.getPoseQMatrix(q, goalJoints, numSteps);
	isHolding = false;
	robot.animateRobotMovement(qMatrix, robot, isHolding, bread, breadTr);
	q = goalJoints;

	goalTr = breadTr * trotx(pi);
	
	qMatrix = robot.getCartesianQMatrix(q, goalTr);
	robot.animateRobotMovement(qMatrix, robot, isHolding, bread, breadTr);
	%now robot is holding the bread
	isHolding = true;
	q = qMatrix(end, :);
	endEffectorLocationTr = asd  
	goalJoints = robot.model.ikine(endEffectorLocationTr, q, [1 1 1 1 1 1]);
	qMatrix = robot.getPoseQMatrix(q, goalJoints, numSteps);
	robot.animateRobotMovement(qMatrix, robot, isHolding, bread, transl(0, 0, 0.05) * trotx(pi));
end
	

	%% HomInvert
function result = HomInvert(transform)
    result = eye(4);
	rot = t2r(transform)';
	result(1:3,4) = -rot * transform(1:3, 4);
	result(1:3, 1:3) = rot;
end


