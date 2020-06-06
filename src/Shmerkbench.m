classdef Shmerkbench < handle
	properties
		
		robot;
		q;
		
		breadTr;
		bread;
		bread2EffTr;
		
		toasterTr;
		toaster2Slot1Tr;
		
		toasterSliderStartTr;
		toasterSliderEndTr;
		toasterSlider2EffTr;
		
		toasterDialTr;
		toasterDial2EffTr;
	end
	
	
	methods(Static)
		
		function self = Shmerkbench()
			self.robot = HansCute;
			self.q = zeros(1, 7);
			
			self.breadTr = transl(0.1, 0.1, 0);
			self.bread = Bread(self.breadTr);
			self.bread2EffTr = transl(0, 0, 0.05) * trotx(pi);
			
			self.toasterTr = transl(0.2, 0, 0);
			self.toaster2Slot1Tr = transl(0, 0.0126, 0.006103);
			
			self.toasterSliderStartTr = self.toasterTr * transl(-0.07, 0.0135, 0.1);
			self.toasterSliderEndTr = self.toasterSliderStartTr * transl(0, 0, -0.08);
			self.toasterSlider2EffTr = transl(0, 0, 0) * trotx(pi);
			
			self.toasterDialTr = self.toasterTr * transl(-0.07, -0.0135, 0.1);
			self.toasterDial2EffTr = transl(0, 0, 0) * trotx(pi);
		end
		%% Main Function
		%Used to run demonstration
		function testAnimate(toastDarknessInput)
			clf
			self = Shmerkbench();
			qVelocities = zeros(1, 7);
			q = zeros(1, 7);
			eff2BreadTr = HomInvert(self.bread2EffTr);
			numSteps = 120;
			isHolding = false;
			
			hold on
			toaster_h = createProp("toaster.ply", self.toasterTr, [1 1 1]);
			hold off
			
			%go to first location above toast
			goalTr = self.breadTr * transl(0, 0, 0.1) * self.bread2EffTr; % location 100mm above bread grabbing point
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			
			%descend in a straight line onto bread
			goalTr = self.breadTr * self.bread2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			
			%grab the bread
			isHolding = true;
			
			%lift bread up 100mm
			goalTr = self.breadTr * transl(0, 0, 0.08) * self.bread2EffTr;
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			
			%lift bread to above the toaster
			% when end effector is at the following location, bread will be 15cm
			% above slot 1
			goalTr = self.toasterTr * self.toaster2Slot1Tr * transl(0, 0, 0.2) * self.bread2EffTr ;
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			
			%slide the bread into the toaster, slot 1
			goalTr = self.toasterTr * self.toaster2Slot1Tr * self.bread2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			
			%move end effector away from toaster while being toasted
			isHolding = false;
			goalTr = self.toasterTr * self.toaster2Slot1Tr  * transl(0, 0, 0.15) * self.bread2EffTr;
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			
			%move end effector to the dial to adjust darkness of toast
			goalTr = self.toasterDialTr * self.toasterDial2EffTr;
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, self.toasterDial2EffTr, q, numSteps);
			
			%adjust dial to set darkness of toast from user input
			goalTr = self.toasterDialTr * self.toasterDial2EffTr * trotx(toastDarknessInput);
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, self.toasterDial2EffTr, q, numSteps);
			
			%move effector back as to not collide with the dial
			goalTr = goalTr * transl(-0.01, 0, 0);
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, self.toasterDial2EffTr, q, numSteps);
			
			%move end effector to just above the slider
			isHolding = false;
			goalTr = self.toasterSliderStartTr * self.toasterSlider2EffTr;
			q = moveRobotJoints(self.robot, goalTr, isHolding, self.bread, self.toasterSlider2EffTr, q, numSteps);
			
			%move end effector straight down to initialise toasting
			isHolding = false;
			goalTr = self.toasterSliderEndTr * self.toasterSlider2EffTr;
			q = moveRobotCartesian(self.robot, goalTr, isHolding, self.bread, self.toasterSlider2EffTr, q);
			
			%move end effector to above where toast will be
			
			
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
