classdef HansCute < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-0.8 0.8 -0.8 0.8 0 1];
        
        %> Flag to indicate if gripper is used
        useGripper = false;
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

    L1 = Link('d',0.15,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L2 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L3 = Link('d',0.1258,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L4 = Link('d',0,'a',0.0667,'alpha',-pi/2,'offset',pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
    L5 = Link('d',0,'a',0.0667,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L6 = Link('d',0,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
    L7 = Link('d',0.1517,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

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
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
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
		%% GetListOfPoses
function qMatrix = GetListOfPoses(self, startJoints, goalJoints, numSteps)
	trapezoid = lspb(0,1,numSteps);
	qMatrix = zeros(numSteps, 7);
	for  i = 1:numSteps
		qMatrix(i,:) = startJoints + trapezoid(i) * (goalJoints - startJoints);
	end
end
		%% AnimateRobotMovement
function AnimateRobotMovement(qMatrix, robot, numSteps, isHolding, points, prop_h, transform, numPoints)
	for i=1:numSteps
		animate(robot.model, qMatrix(i,:));
		if isHolding == true
			for j = 1:numPoints
				prop_h.Vertices(j,:) = transl(robot.model.fkine(qMatrix(i,:)) * transform * transl(points(j,:)))';
			end
		end
		drawnow()
	end
end
    end
end