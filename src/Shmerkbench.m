classdef Shmerkbench < handle
	properties
		
		robot;
		q;
		
		breadTr;
		bread;
		bread2EffTr;
		
		toasterTr;
		toaster2Slot1Tr;
		
		sliderTr;
		sliderEndTr;
		slider2EffTr;
		
		dialTr;
		dial2EffTr;
		
		tableTr;
		
		plateTr;
		plate2BreadTr;
		
		toaster_h;
		table_h;
		plate_h;
		
		stopState;
	end
	
	
	methods
		
		function self = Shmerkbench()
			clf
			self.robot = HansCute();
			
			self.breadTr = transl(0.1, 0.1, 0); % absolute
			self.bread2EffTr = transl(0, 0, 0.05) * trotx(pi); % relative
			
			self.toasterTr = transl(0.2, 0, 0); % absolute
			self.toaster2Slot1Tr = transl(0, 0.0126, 0.008103); % relative
			
			self.sliderTr = self.toasterTr * transl(-0.07, 0.0135, 0.1); % absolute
			self.sliderEndTr = self.sliderTr * transl(0, 0, -0.08); % absolute
			self.slider2EffTr = transl(0, 0, 0) * trotx(pi); % relative

			self.dialTr = self.toasterTr * transl(-0.020589,0.027688,0.067801) * trotx(-pi/2); % absolute
			self.dial2EffTr = transl(0,0,0.006) * trotx(pi); % relative
			
			self.tableTr = transl(0,0,0); % absolute
			
			self.plateTr = transl(0.12,-0.12,0); % absolute
			self.plate2BreadTr = trotz(pi/2) * transl(0, 0.022265, 0.010481) * trotx(-72.8 * pi/180); % relative
			
			self.bread = Bread(self.breadTr);
			self.toaster_h = createProp("toaster.ply", self.toasterTr, [1 1 1]);
			self.table_h = createProp("table.ply", self.tableTr, [231 217 198]/255);
			self.plate_h = createProp("plate.ply", self.plateTr, [1 1 1]);
			
			self.stopState = 0;
			
			ooeyGui(self);
		end
		%% Main Function
		%Used to run demonstration
		function testAnimate(self, toastDarknessValue)
			qVelocities = zeros(1, 7);
			q = zeros(1, 7);
			eff2BreadTr = HomInvert(self.bread2EffTr);
			numSteps = 120;
			isHolding = false;
			self.breadTr = self.plateTr * self.plate2BreadTr;
			toast_val = -1.56 + 0.52 * toastDarknessValue;
			eff2DialTr = HomInvert(self.dial2EffTr);
			

			% move to 10cm above bread
			isHolding = false;
			goalTr = self.breadTr * transl(0,0,0.1) * self.bread2EffTr;
			self.robot.model.ikcon(goalTr, q)
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			% move to bread
			goalTr = self.breadTr * self.bread2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			% grab bread
			isHolding = true;
			% lift bread
			goalTr = self.breadTr * transl(0,0,0.1) * self.bread2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			% move bread to above toaster
			goalTr = self.toasterTr * self.toaster2Slot1Tr * transl(0,0,0.15) * self.bread2EffTr;
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			% lower bread into toaster
			goalTr = self.toasterTr * self.toaster2Slot1Tr * self.bread2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			%let go of bread
			isHolding = false;
			self.breadTr = self.robot.model.fkine(q) * eff2BreadTr;
			% move arm up away from toaster
			goalTr = self.toasterTr * self.toaster2Slot1Tr * transl(0,0.1,0.15) * self.bread2EffTr;
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			% move arm to above dial
			goalTr = self.dialTr * transl(0,0,0.05) * self.dial2EffTr;
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			% lower arm to dial
			goalTr = self.dialTr * self.dial2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			pause(0.5);
			% twist dial
			goalTr = self.dialTr * trotz(-pi/4) * self.dial2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			pause(0.5);
			% move arm to above dial
			goalTr = self.dialTr * transl(0,0,0.02) * trotz(-pi/4) * self.dial2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			% move arm to above slider
			goalTr = self.sliderTr * transl(0,0,0.1) * self.slider2EffTr;
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			% move arm to slider
			goalTr = self.sliderTr * self.slider2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			pause(0.5);
			% push slider down
			isHolding = true;
			eff2BreadTr = HomInvert(self.robot.model.fkine(q)) * self.breadTr;
			goalTr = self.sliderEndTr * self.slider2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			self.breadTr = self.robot.model.fkine(q) * eff2BreadTr;
			pause(0.5);
			% move arm to above slider
			isHolding = false;
			goalTr = self.sliderTr * transl(0,0,0.1) * self.slider2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			% wait for toast
			pause(2);
			% move arm to slider
			goalTr = self.sliderEndTr * self.slider2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			pause(0.2);
			% pull slider up
			isHolding = true;
			eff2BreadTr = HomInvert(self.robot.model.fkine(q)) * self.breadTr;
			goalTr = self.sliderTr * self.slider2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			self.breadTr = self.robot.model.fkine(q) * eff2BreadTr;
			pause(0.2);
			% move arm to above slider
			isHolding = false;
			goalTr = self.sliderTr * transl(0,0,0.1) * self.slider2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			% move arm to bread
			self.bread2EffTr = transl(0, 0, 0.05) * trotx(pi); % relative
			eff2BreadTr = HomInvert(self.bread2EffTr); % relative
			goalTr = self.breadTr * self.bread2EffTr;
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			% pull bread out of toaster
			isHolding = true;
			goalTr = self.breadTr * transl(0,0,0.05) * self.bread2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			goalTr = self.breadTr * transl(-0.1,0,0.1) * self.bread2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			% move bread above plate
			goalTr = self.plateTr * transl(0,0,0.1) * self.plate2BreadTr * self.bread2EffTr;
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			% put bread on plate
			goalTr = self.plateTr * self.plate2BreadTr * self.bread2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			% move away from bread
			isHolding = false;
			goalTr = self.plateTr * self.plate2BreadTr * transl(0,0,0.05) * self.bread2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			% return to start
			goalJoints = [0 0 0 0 0 0 0];
			qMatrix = self.robot.getPoseQMatrix(q, goalJoints, numSteps);
			self.robot.animateRobotMovement(qMatrix, self.robot, isHolding, self.bread, eff2BreadTr);
		end
	end
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

function result = HomInvert(transform)
    result = eye(4);
	rot = t2r(transform)';
	result(1:3,4) = -rot * transform(1:3, 4);
	result(1:3, 1:3) = rot;
end
