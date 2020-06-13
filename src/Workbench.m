pq = q
q = robot.model.ikcon(transl(x,y,z) * rpy2tr(r,p,y))
robot.model.animate(q, pq);
drawnow();


clf
robot = HansCute;
q = zeros(1, 7);
qVelocities = zeros(1, 7);
numSteps = 120;
% bread vectors
breadTr = transl(0.1, 0.1, 0); % absolute
bread2EffTr = transl(0, 0, 0.05) * trotx(pi); % relative
eff2BreadTr = HomInvert(bread2EffTr); % relative
% toaster vectors
toasterTr = transl(0.2, -0.04, 0); % absolute
toaster2Slot1Tr = transl(0, 0.0126, 0.04); % relative
% slider vectors
sliderTr = toasterTr * transl(-0.063369,0,0.070012); % absolute
sliderEndTr = sliderTr * transl(0, 0, -0.05); % absolute
slider2EffTr = transl(0, 0, 0) * trotx(pi); % relative
% dial vectors
dialTr = toasterTr * transl(-0.020589,0.027688,0.067801) * trotx(-pi/2); % absolute
dial2EffTr = transl(0,0,0.006) * trotx(pi); % relative
eff2DialTr = HomInvert(dial2EffTr); % relative
% other vectors
tableTr = transl(0,0,0); % absolute
plateTr = transl(0.12,-0.12,0); % absolute
plate2BreadTr = trotz(pi/2) * transl(0, 0.022265, 0.010481) * trotx(-72.8 * pi/180); % relative
% draw props
hold on
    bread = Bread(breadTr);
    toaster_h = createProp("toaster.ply", toasterTr, [1 1 1]);
    table_h = createProp("table.ply", tableTr, [231 217 198]/255);
    plate_h = createProp("plate.ply", plateTr, [1 1 1]);
    dial_h = createProp("dial.ply", dialTr, [0.5 0.5 0.5]);
    slider_h = createProp("slider.ply", sliderTr, [0.8 0.8 0.8]);
hold off
% move to 10cm above bread
isHolding = false;
goalTr = breadTr * transl(0,0,0.1) * bread2EffTr;
q = moveRobotJoints(robot, goalTr, isHolding, bread, eff2BreadTr, q, numSteps);
% move to bread
goalTr = breadTr * bread2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
% grab bread
isHolding = true;
% lift bread
goalTr = breadTr * transl(0,0,0.1) * bread2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
% move bread to above toaster
goalTr = toasterTr * toaster2Slot1Tr * transl(0,0,0.15) * bread2EffTr;
q = moveRobotJoints(robot, goalTr, isHolding, bread, eff2BreadTr, q, numSteps);
% lower bread into toaster
goalTr = toasterTr * toaster2Slot1Tr * bread2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
%let go of bread
isHolding = false;
breadTr = robot.model.fkine(q) * eff2BreadTr;
% move arm up away from toaster
goalTr = toasterTr * toaster2Slot1Tr * transl(0,0.1,0.15) * bread2EffTr;
q = moveRobotJoints(robot, goalTr, isHolding, bread, eff2BreadTr, q, numSteps);
% move arm to above dial
goalTr = dialTr * transl(0,0,0.05) * dial2EffTr;
q = moveRobotJoints(robot, goalTr, isHolding, bread, eff2BreadTr, q, numSteps);
% lower arm to dial
goalTr = dialTr * dial2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
pause(0.5);
% twist dial
goalTr = dialTr * trotz(-pi/4) * dial2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
pause(0.5);
% move arm to above dial
goalTr = dialTr * transl(0,0,0.02) * trotz(-pi/4) * dial2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
% move arm to above slider
goalTr = sliderTr * transl(0,0,0.1) * slider2EffTr;
q = moveRobotJoints(robot, goalTr, isHolding, bread, eff2BreadTr, q, numSteps);
% move arm to slider
goalTr = sliderTr * slider2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
pause(0.5);
% push slider down
isHolding = true;
eff2BreadTr = HomInvert(robot.model.fkine(q)) * breadTr;
goalTr = sliderEndTr * slider2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
breadTr = robot.model.fkine(q) * eff2BreadTr;
pause(0.5);
% move arm to above slider
isHolding = false;
goalTr = sliderTr * transl(0,0,0.1) * slider2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
% wait for toast
pause(2);
% move arm to slider
goalTr = sliderEndTr * slider2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
pause(0.2);
% pull slider up
isHolding = true;
eff2BreadTr = HomInvert(robot.model.fkine(q)) * breadTr;
goalTr = sliderTr * slider2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
breadTr = robot.model.fkine(q) * eff2BreadTr;
pause(0.2);
% move arm to above slider
isHolding = false;
goalTr = sliderTr * transl(0,0,0.1) * slider2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
% move arm to bread
bread2EffTr = transl(0, 0, 0.05) * trotx(pi); % relative
eff2BreadTr = HomInvert(bread2EffTr); % relative
goalTr = breadTr * bread2EffTr;
q = moveRobotJoints(robot, goalTr, isHolding, bread, eff2BreadTr, q, numSteps);
% pull bread out of toaster
isHolding = true;
goalTr = breadTr * transl(0,0,0.05) * bread2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
goalTr = breadTr * transl(-0.1,0,0.1) * bread2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
% move bread above plate
goalTr = plateTr * transl(0,0,0.1) * plate2BreadTr * bread2EffTr;
q = moveRobotJoints(robot, goalTr, isHolding, bread, eff2BreadTr, q, numSteps);
% put bread on plate
goalTr = plateTr * plate2BreadTr * bread2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
% move away from bread
isHolding = false;
goalTr = plateTr * plate2BreadTr * transl(0,0,0.05) * bread2EffTr;
q = moveRobotCartesian(robot, goalTr, isHolding, bread, eff2BreadTr, q);
% return to start
goalJoints = [0 0 0 0 0 0 0];
qMatrix = robot.getPoseQMatrix(q, goalJoints, numSteps);
robot.animateRobotMovement(qMatrix, robot, isHolding, bread, eff2BreadTr);


function prop_h = createProp(propName, locationTr, colour)
    [faces, points, data] = plyread(propName, "tri");
    hold on
    prop_h=trisurf(faces,points(:,1),points(:,2),points(:,3),"LineStyle","none","Facecolor",colour);
    hold off
    numPoints = size(points);
    numPoints = numPoints(1);
    for j=1:numPoints
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


