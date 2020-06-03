clf
robot = HansCute;
q = zeros(1, 7);
robot.model.animate(q);
breadTr = transl(0.1, 0.1, 0);
bread = Bread(breadTr);
testAnimate(robot, q, bread, breadTr)


%% Main Function
%Used to run demonstration
function testAnimate(robot, startQ, bread, breadTr)
    qVelocities = zeros(1,7);
    q = startQ;
	endEffectorVelocities = transpose([0 0 -0.01 0 0 0]);
   %first location above toast
    numSteps = 120;
    endEffectorLocationTr = transl(0.1, 0.1, 0.2) * trotx(pi);
	goalJoints = robot.model.ikine(endEffectorLocationTr, q, [1 1 1 1 1 1]);
    qMatrix = robot.getPoseQMatrix(q, goalJoints, numSteps);
	isHolding = false;
	robot.animateRobotMovement(qMatrix, robot, isHolding, bread, breadTr);
	q = goalJoints;

	goalTr = breadTr * trotx(pi);
	currentTransform = robot.model.fkine(q);
	qMatrix = robot.getCartesianQMatrix(q, goalTr);
	robot.animateRobotMovement(qMatrix, robot, isHolding, bread, breadTr);
end
	

	%% HomInvert
function result = HomInvert(transform)
    result = eye(4);
	rot = t2r(transform)';
	result(1:3,4) = -rot * transform(1:3, 4);
	result(1:3, 1:3) = rot;
end


