clf
robot = HansCute;
q = zeros(1, 7);

breadTr = transl(0.1, 0.1, 0);
bread = Bread(breadTr);
bread2EffTr = transl(0, 0, 0.05) * trotx(pi);

<<<<<<< HEAD
toasterTr = transl(0.15,0,0.02);
=======
toasterTr = transl(0.15,0,0);
>>>>>>> 7f206e2e6623ef76764afd8ffc162a88d1723abe
toaster2Slot1Tr = transl(0,0.0126,0.006103);

toasterSliderStartTr = toasterTr * transl(-0.02, 0, 0);
toasterSliderEndTr = toasterSliderStartTr * transl(0, 0, -0.05);
toasterSlider2EffTr = transl(0, 0, 0) * trotx(pi);

testAnimate(robot, q, bread, breadTr, bread2EffTr, toasterTr, toaster2Slot1Tr, toasterSliderStartTr, toasterSliderEndTr, toasterSlider2EffTr);


%% Main Function
%Used to run demonstration
function testAnimate(robot, startQ, bread, breadTr, bread2EffTr, toasterTr, toaster2Slot1Tr, toasterSliderStartTr, toasterSliderEndTr, toasterSlider2EffTr)
    qVelocities = zeros(1,7);
    q = startQ;
    eff2BreadTr = HomInvert(bread2EffTr);
    numSteps = 10;
    isHolding = false;
    
    hold on
    toaster_h=createProp("toaster.ply",toasterTr,[1 1 1])
    hold off
    
    %go to first location above toast
    goalTr = breadTr * transl(0,0,0.1) * bread2EffTr; % location 100mm above bread grabbing point
	q = moveRobotJoints(robot, goalTr, isHolding, bread, eff2BreadTr, q, numSteps);

    %descend in a straight line onto bread
	goalTr = breadTr * bread2EffTr;
	q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
    
    %grab the bread
	isHolding = true;

    %lift bread up 100mm
	goalTr = breadTr * transl(0,0,0.08) * bread2EffTr;
	q = moveRobotJoints(robot, goalTr, isHolding, bread, eff2BreadTr, q, numSteps);
    
	%lift bread to above the toaster
    % when end effector is at the following location, bread will be 15cm
    % above slot 1
    goalTr = toasterTr * toaster2Slot1Tr * transl(0,0,0.2) * bread2EffTr ;
	q = moveRobotJoints(robot, goalTr, isHolding, bread, eff2BreadTr, q, numSteps);
    
    %slide the bread into the toaster, slot 1
    goalTr = toasterTr * toaster2Slot1Tr * bread2EffTr;
	q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
	
	%move end effector away from toaster while being toasted
	isHolding = false;
	goalTr = toasterTr * toaster2Slot1Tr  * transl(0, 0, 0.15) * bread2EffTr;
	q = moveRobotJoints(robot, goalTr, isHolding, bread, eff2BreadTr, q, numSteps);
	
	%move end effector to depress slider and initilialise toasting
	isHolding = false;
	goalTr = toasterSliderStartTr * toasterSliderEndTr * toasterSlider2EffTr;
	q = moveRobotJoints(robot, goalTr, isHolding, bread, toasterSlider2EffTr, q, numSteps);
end

function prop_h = createProp(propName, locationTr, colour)
    [faces, points, data] = plyread(propName, "tri");
    hold on
    prop_h=trisurf(faces,points(:,1),points(:,2),points(:,3),"LineStyle","none","Facecolor",colour);
    hold off
    numPoints = size(points);
    self.numPoints = numPoints(1);
    for j=1:self.numPoints
        prop_h.Vertices(j,:)=transl(locationTr*transl(points(j,:)))';
    end
end

function endJoints = moveRobotCartesian(robot, goalTr, isHolding, prop, eff2PropTr, startJoints)
    qMatrix = robot.getCartesianQMatrix(startJoints, goalTr);
    endJoints = qMatrix(end, :);
    robot.animateRobotMovement(qMatrix, robot, isHolding, prop, eff2PropTr);
end

function endJoints = moveRobotJoints(robot, goalTr, isHolding, prop, eff2PropTr, startJoints, numSteps)
	goalJoints = robot.model.ikcon(goalTr, startJoints);
    qMatrix = robot.getPoseQMatrix(startJoints, goalJoints, numSteps);
    endJoints = goalJoints;
	robot.animateRobotMovement(qMatrix, robot, isHolding, prop, eff2PropTr);
end


