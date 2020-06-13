classdef HansCute < handle
	properties
		%> Robot model
		model;
		%>
		workspace = [-0.8 0.8 -0.8 0.8 0 1];
		
		%> Flag to indicate if gripper is used
		useGripper = false;
		
		maxAllowedVelocity = 0.6 / 30; %taken from hanscute recommended maxspeed in the sourcecode
	end
	
	methods%% Class for HansCute robot simulation
		function self = HansCute()
			self.GetHansCuteRobot();
			% robot =
			self.PlotAndColourRobot();%robot,workspace);
		end
		
		%% GetHansCuteRobot
		% Given a name (optional), create and return a HansCute robot model
		function GetHansCuteRobot(self)
			%     if nargin < 1
			% Create a unique name (ms timestamp after 1ms pause)
			pause(0.001);
			name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];
			%     end
			L1 = Link('d',0.15,'a',0,'alpha',pi/2,'offset',0,'qlim',[-2.5, 2.5]);
			L2 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0,'qlim',[-1.8, 1.8]);
			L3 = Link('d',0.1258,'a',0,'alpha',pi/2,'offset',0,'qlim',[-2.5, 2.5]);
			L4 = Link('d',0,'a',0.0667,'alpha',-pi/2,'offset',pi/2,'qlim',[-1.8, 1.8]);
			L5 = Link('d',0,'a',0.0667,'alpha',pi/2,'offset',0,'qlim',[-1.8, 1.8]);
			L6 = Link('d',0,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[-1.8 1.8]);
			L7 = Link('d',0.1517,'a',0,'alpha',0,'offset',0,'qlim',[-2.5 2.5]);
			
			self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
		end
		%% PlotAndColourRobot
		% Given a robot index, add the glyphs (vertices and faces) and
		% colour them in if data is available
		function PlotAndColourRobot(self)%robot,workspace)
			for linkIndex = 0:self.model.n
				if self.useGripper && linkIndex == self.model.n
					[ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['HCLink',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
				else
					[ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['HCLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
				end
				self.model.faces{linkIndex+1} = faceData;
				self.model.points{linkIndex+1} = vertexData;
			end
			
			% Display robot
			self.model.plot3d(zeros(1,self.model.n), 'noarrow', 'workspace', self.workspace);
			if isempty(findobj(get(gca, 'Children'), 'Type', 'Light'))
				camlight
			end
			self.model.delay = 0;
			
			% Try to correctly colour the arm (if colours are in ply file data)
			for linkIndex = 0:self.model.n
				handles = findobj('Tag', self.model.name);
				h = get(handles,'UserData');
				try
					h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
						, plyData{linkIndex+1}.vertex.green ...
						, plyData{linkIndex+1}.vertex.blue]/255;
					h.link(linkIndex+1).Children.FaceColor = 'interp';
				catch ME_1
					disp(ME_1);
					continue;
				end
			end
		end
		
		%% animateRobotMovement
		function animateRobotMovement(self, qMatrix, robot, isHolding, prop, effToPropTr, gui)
			numSteps = size(qMatrix);
			numSteps = numSteps(1);
			for i = 1:numSteps
				drawnow()
% 				stop_state = str2double(gui.EMERGENCYSTOPSwitch.Value);
% 				if stop_state == 1
% 					break;
% 					stopAnimating(q, robot, isHolding, prop, effToPropTr);
% 					return;
% 				end 
				animate(robot.model, qMatrix(i, :));
				if isHolding == true
					prop.updatePos(robot.model.fkine(qMatrix(i, :)) * effToPropTr);
				end
			end
		end
		%% stopAnimating
		function stopAnimating(self, qMatrix, robot, isHolding, prop, effToPropTr)
			robot.model.animate(q);
			drawnow();
			if isHolding == true
				prop.updatePos(robot.model.fkine(q) * effToPropTr);
			end
		end
		
		%% getPoseQMatrix
		function qMatrix = getPoseQMatrix(self, startJoints, goalJoints, numSteps)
			trapezoid = lspb(0, 1, numSteps);
			qMatrix = zeros(numSteps, 7);
			for  i = 1:numSteps
				qMatrix(i,:) = startJoints + trapezoid(i) * (goalJoints - startJoints);
			end
		end
		
		%% getCartesianQMatrix
		function qMatrix = getCartesianQMatrix(self, startJoints, goalTransform)
			qMin = self.model.qlim(:, 1)';
			qMax = self.model.qlim(:, 2)';
			qVelocities = zeros(1, 7); %init velocity is 0
			q = startJoints;
			i = 1;
			while true
				currentTransform = self.model.fkine(q);
				diffTransform = HomInvert(currentTransform) * goalTransform;
				coords = transl(goalTransform)-transl(currentTransform);
				coords = transpose(coords);
				re = t2r(goalTransform)*t2r(currentTransform)';
				rpy = tr2rpy(r2t(re));
				endEffectorVelocities = [coords rpy];
				endEffectorVelocities = transpose(endEffectorVelocities);
				w = JointsTools.getWeightedMatrix(q, qMax, qMin, qVelocities, ones(1,7));
				j = self.model.jacob0(q);
				qVelocities = JointsTools.getJointVelocities(j, endEffectorVelocities, w);
				maxVelocity = max(qVelocities);
				x = qVelocities / maxVelocity;
				qVelocities = x * self.maxAllowedVelocity;
				q = q + transpose(qVelocities);
				qMatrix(i, :) = q;
				i = i + 1;
				%stop loop when end effector is in acceptable distance of goal
				d = distance(currentTransform, goalTransform);
				if  d(4,4) < 0.005
					break
				end
				if i > 500
					break
				end
				%                 self.model.animate(q);
				%                 drawnow();
			end
		end
		
		%% clampedQVelocity
		%prevent joins from exceeding maxAllowedVelocity
		function clampedQVelocities = clampQVelocities(qVelocities, maxAllowedVelocity)
			maxVelocity = max(abs(qVelocities));
			x = qVelocities / maxVelocity;
			clampedQVelocities = x * maxAllowedVelocity;
		end
		
		%% getEndEffectorVelocities
		function endEffectorVelocities = getEndEffectorVelocities(currentTransform, goalTransform)
			coords = transl(goalTransform) - transl(currentTransform);
			coords = transpose(coords);
			rpy = tr2rpy(goalTransform) - tr2rpy(currentTransform);
			endEffectorVelocities = [coords rpy];
		end
	end
end