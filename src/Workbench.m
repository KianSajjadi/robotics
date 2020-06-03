qMax = [2.5 1.8 2.5 1.8 1.8 1.8 2.5];
qMin = -qMax;
maxAllowedVelocity = 0.6/30; %taken from hanscute recommended maxspeed in the sourcecode
clf
robot = HansCute;
q = zeros(1, 7);
robot.model.animate(q);
breadTr = transl(0.1, 0.1, 0);
bread = Bread(breadTr);
testAnimate(robot, q, qMax, qMin, maxAllowedVelocity, bread, breadTr)


%% Main Function
%Used to run demonstration
function testAnimate(robot, startQ, qMax, qMin, maxAllowedVelocity, bread, breadTr)
    qVelocities=zeros(1,7);
    q=startQ;
	endEffectorVelocities = transpose([0 0 -0.01 0 0 0]);
   %first location above toast
    endEffectorLocationTr = transl(0.1, 0.1, 0.2) * trotx(pi);
	goalJoints = robot.model.ikine(endEffectorLocationTr, q, [1 1 1 1 1 1]);
	numSteps = 120;
    qMatrix = robot.GetListOfPoses(q, goalJoints, numSteps);
	isHolding = false;
	AnimateRobotMovement(qMatrix, robot, numSteps, isHolding, 0, 0, 0);
	%now toast has been grabbed
	isHolding = true;
	q = goalJoints;
	while true
		w = JointsTools.getWeightedMatrix(q, qMax, qMin, qVelocities, ones(1,7));
		j = robot.model.jacob0(q);
		qVelocities = JointsTools.getJointVelocities(j, endEffectorVelocities, w);
		clampedQVelocities = clampQVelocities(qVelocities, maxAllowedVelocity);
		q = q + transpose(clampedQVelocities);
		robot.model.animate(q);
		drawnow();
		
		rTr = robot.model.fkine(q);
		[xCoord, yCoord, zCoord] = transl(rTr);
		[xGoal, yGoal, zGoal] = transl(breadTr);
		zGoal = zGoal + 0.052 %optimal clamping locale
		
		%dynamic position correction
		endEffectorVelocities(1) = xGoal - xCoord;
		endEffectorVelocities(2) = yGoal - yCoord;
		if zCoord < zGoal
			break
		end
    end
	
end
	
	%% clampedQVelocity
	%prevent joins from exceeding maxAllowedVelocity
function clampedQVelocities = clampQVelocities(qVelocities, maxAllowedVelocity)
	maxVelocity = max(qVelocities);
	x = qVelocities / maxVelocity;
	clampedQVelocities = x * maxAllowedVelocity;
end
	
function result = HomInvert(transform)
    result = eye(4);
	rot = t2r(transform)';
	result(1:3,4) = -rot * transform(1:3, 4);
	result(1:3, 1:3) = rot;
end


