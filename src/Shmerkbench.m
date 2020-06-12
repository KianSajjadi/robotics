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
        
        tableTr;
        
        plateTr;
        plate2BreadTr;
        
		GUI;
	end
	
	
	methods
		
		function self = Shmerkbench()
			self.robot = HansCute();
			self.q = zeros(1, 7);
			
			self.breadTr = transl(0.1, 0.1, 0); % absolute
			self.bread2EffTr = transl(0, 0, 0.05) * trotx(pi); % relative
			
			self.toasterTr = transl(0.2, 0, 0); % absolute
			self.toaster2Slot1Tr = transl(0, 0.0126, 0.008103); % relative
			
			self.toasterSliderStartTr = self.toasterTr * transl(-0.07, 0.0135, 0.1); % absolute
			self.toasterSliderEndTr = self.toasterSliderStartTr * transl(0, 0, -0.08); % absolute
			self.toasterSlider2EffTr = transl(0, 0, 0) * trotx(pi); % relative
			
			self.toasterDialTr = self.toasterTr * transl(-0.07, -0.0135, 0.1); % absolute
			self.toasterDial2EffTr = transl(0, 0, 0) * trotx(pi); % relative
            
            self.tableTr = transl(0,0,0); % absolute
            
            self.plateTr = transl(0.12,-0.12,0); % absolute
            self.plate2BreadTr = trotz(pi/2) * transl(0, 0.022265, 0.010481) * trotx(-72.8 * pi/180); % relative
% 			self.GUI = GUI;
        end
        
 		%% Main Function
		%Used to run demonstration
		function testAnimate(self)
			clf
% 			self = Shmerkbench();
			qVelocities = zeros(1, 7);
			q = zeros(1, 7);
			eff2BreadTr = HomInvert(self.bread2EffTr);
			numSteps = 120;
			isHolding = false;
            self.breadTr = self.plateTr * self.plate2BreadTr;
			hold on
                bread = Bread(self.breadTr);
                toaster_h = createProp("toaster.ply", self.toasterTr, [1 1 1]);
                table_h = createProp("table.ply", self.tableTr, [231 217 198]/255);
                plate_h = createProp("plate.ply", self.plateTr, [1 1 1]);
			hold off
			toast_val = -1.56 + 0.52 * str2double(self.GUI.ToastDarknessKnob.Value);
            
			%go to first location above bread
			goalTr = self.breadTr * transl(0, 0, 0.1) * self.bread2EffTr; % location 100mm above bread grabbing point
			q = moveRobotJoints(self, self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			
			%descend in a straight line onto bread
			goalTr = self.breadTr * self.bread2EffTr;
			q = moveRobotCartesian(self, self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			
			%grab the bread
			isHolding = true;
			
			%lift bread up 100mm
			goalTr = self.breadTr * transl(0, 0, 0.08) * self.bread2EffTr;
			q = moveRobotJoints(self, self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			
			%lift bread to above the toaster
			% when end effector is at the following location, bread will be 15cm
			% above slot 1
			goalTr = self.toasterTr * self.toaster2Slot1Tr * transl(0, 0, 0.2) * self.bread2EffTr ;
			q = moveRobotJoints(self, self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			
			%slide the bread into the toaster, slot 1
			goalTr = self.toasterTr * self.toaster2Slot1Tr * self.bread2EffTr;
			q = moveRobotCartesian(self, self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q);
			
			%move end effector away from toaster while being toasted
			isHolding = false;
			goalTr = self.toasterTr * self.toaster2Slot1Tr  * transl(0, 0, 0.15) * self.bread2EffTr;
			q = moveRobotJoints(self, self.robot, goalTr, isHolding, self.bread, eff2BreadTr, q, numSteps);
			
			%move end effector to the dial to adjust darkness of toast
			goalTr = self.toasterDialTr * self.toasterDial2EffTr;
			q = moveRobotJoints(self, self.robot, goalTr, isHolding, self.bread, self.toasterDial2EffTr, q, numSteps);
			
			%adjust dial to set darkness of toast from user input
			goalTr = self.toasterDialTr * self.toasterDial2EffTr * trotx(toast_val);
			q = moveRobotJoints(self, self.robot, goalTr, isHolding, self.bread, self.toasterDial2EffTr, q, numSteps);
			
			%move effector back as to not collide with the dial
			goalTr = goalTr * transl(-0.01, 0, 0);
			q = moveRobotJoints(self, self.robot, goalTr, isHolding, self.bread, self.toasterDial2EffTr, q, numSteps);
			
			%move end effector to just above the slider
			isHolding = false;
			goalTr = self.toasterSliderStartTr * self.toasterSlider2EffTr;
			q = moveRobotJoints(self, self.robot, goalTr, isHolding, self.bread, self.toasterSlider2EffTr, q, numSteps);
			
			%move end effector straight down to initialise toasting
			isHolding = false;
			goalTr = self.toasterSliderEndTr * self.toasterSlider2EffTr;
			q = moveRobotCartesian(self, self.robot, goalTr, isHolding, self.bread, self.toasterSlider2EffTr, q);
			
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
		
		function endJoints = moveRobotCartesian(self, robot, goalTr, isHolding, prop, eff2PropTr, startJoints)
			qMatrix = robot.getCartesianQMatrix(startJoints, goalTr);
			endJoints = qMatrix(end, :);
			robot.animateRobotMovement(qMatrix, robot, isHolding, prop, eff2PropTr, self.GUI);
		end
		
		function endJoints = moveRobotJoints(self, robot, goalTr, isHolding, prop, eff2PropTr, startJoints, numSteps)
			goalJoints = robot.model.ikcon(goalTr, startJoints);
			qMatrix = robot.getPoseQMatrix(startJoints, goalJoints, numSteps);
			endJoints = goalJoints;
			robot.animateRobotMovement(qMatrix, robot, isHolding, prop, eff2PropTr, self.GUI);
		end
		
		function result = HomInvert(transform)
		  result = eye(4);
		  rot = t2r(transform)';
	      result(1:3,4) = -rot * transform(1:3, 4);
	      result(1:3, 1:3) = rot;
		end
