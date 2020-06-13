function advTeach(robot, varargin)
    %params for teach panel
	bgcol = [135 206 250]/255;
	height = 0.06;
	
	%handle options
	opt.deg = true;
	opt.mode = {'xyz', 'joints'};
    opt.orientation = {'rpy', 'eul', 'approach'};
    opt.callback = [];    
    [opt,args] = tb_optparse(opt, varargin);
	
	handles.orientation = opt.orientation;
    handles.callback = opt.callback;
    handles.opt = opt;
	handles.mode = opt.mode;
	
	qlim = robot.model.qlim;
	
	if any(isinf(qlim))
		error('RTB:teach:badarg', 'Must define joint coordinate limits for prismatic axes, set qlim properties for prismatic Links');
	end
	
	if isempty(args)
		q = [];
	else
		q = args{1};
	end
	
	% set up scale factor, from actual limits in radians/metres to display units
	qscale = ones(robot.model.n,1);
	for j = 1:robot.model.n
		L = robot.model.links(j);
		if opt.deg && L.isrevolute
			qscale(j) = 180/pi;
		end
	end
	
	handles.robot = robot;
	
	%find figure to put advTeach panel in
	c = findobj(gca, 'Tag', robot.model.name);
	if isempty(c)
		c = findobj(0, 'Tag', robot.model.name);
		if isempty(c)
			robot.model.plot(zeros(1, robot.model.n));
			ax = gca;
		else
			ax = get(c(1), 'Parent');
		end
	else
		ax = gca;
	end
	
		handles.fig = get(ax, 'Parent');
		
		set(ax, 'Outerposition', [0.25 0 0.70 1]);
		
		handles.curax = ax;
		
		%create panel
		panel = uipanel(handles.fig, ...
			'Title', 'advTeach', ...
			'BackGroundColor', bgcol, ...
			'Position', [0 0 0.25 1]);
		set(panel, 'Units', 'pixels');
		handles.panel = panel;
		set(handles.fig, 'Units', 'pixels');
		set(handles.fig, 'ResizeFcn', @(src,event) resize_callback(robot.model, handles));
		
		
		%get current robot state
		if isempty(q)
			rhandles = findobj('Tag', robot.model.name);
			
			if isempty(rhandles)
				error('RTB:teach:badarg', 'No graphical robot of this name found');
			end
			info = get(rhandles(1), 'UserData');
			
			if ~isempty(info.q)
				q = info.q;
			end
		else
			robot.model.plot(q);
		end
		q = q
		handles.q = q;
		T6 = robot.model.fkine(q);
		
		%% XYZ sliders and edit boxes
		xyz = transl(T6);
		xyzNames = ['x', 'y', 'z'];
		rpyNames = ['r', 'p', 'y'];
		n = 3;
		
		for j = 1:n
			% slider label
			uicontrol(panel, 'Style', 'text', ...
				'Units', 'normalized', ...
				'BackgroundColor', bgcol, ...
				'Position', [0 height*(n - j + 2) 0.15 height], ...
				'FontUnits', 'normalized', ...
				'FontSize', 0.5, ...
				'String', sprintf('%c', xyzNames(1, j)));
			
			% slider itself
			q(j) = max( qlim(j,1), min( qlim(j,2), q(j) ) ); % clip to range
			handles.slider(j) = uicontrol(panel, 'Style', 'slider', ...
				'Units', 'normalized', ...
				'Position', [0.15 height*(n - j + 2) 0.65 height], ...
				'Min', qlim(j,1), ...
				'Max', qlim(j,2), ...
				'Value', xyz(j, 1), ...
				'Tag', sprintf('Slider%c', xyzNames(1, j)));
			
			% text box showing slider value, also editable
			handles.edit(j) = uicontrol(panel, 'Style', 'edit', ...
				'Units', 'normalized', ...
				'Position', [0.80 height*(n - j + 2)+.01 0.20 0.9*height], ...
				'BackgroundColor', bgcol, ...
				'String', num2str(xyz(j, 1), 3), ...
				'HorizontalAlignment', 'left', ...
				'FontUnits', 'normalized', ...
				'FontSize', 0.4, ...
				'Tag', sprintf('Edit%c', xyzNames(1, j)));
		end
		
	%% Joint limit sliders and edit boxes
	
		nR = robot.model.n;
		for j = 1:nR
			% slider label
			uicontrol(panel, 'Style', 'text', ...
				'Units', 'normalized', ...
				'BackgroundColor', bgcol, ...
				'Position', [0 height*(nR - j + 5) 0.15 height], ...
				'FontUnits', 'normalized', ...
				'FontSize', 0.5, ...
				'String', sprintf('q%d', j));
			
			% slider itself
			q(j) = max( qlim(j,1), min( qlim(j,2), q(j) ) ); % clip to range
			handles.slider(j + 3) = uicontrol(panel, 'Style', 'slider', ...
				'Units', 'normalized', ...
				'Position', [0.15 height*(nR - j + 5) 0.65 height], ...
				'Min', qlim(j,1), ...
				'Max', qlim(j,2), ...
				'Value', q(j), ...
				'Tag', sprintf('Slider%d', j));
			
			% text box showing slider value, also editable
			handles.edit(j + 3) = uicontrol(panel, 'Style', 'edit', ...
				'Units', 'normalized', ...
				'Position', [0.80 height*(nR - j + 5)+.01 0.20 0.9*height], ...
				'BackgroundColor', bgcol, ...
				'String', num2str(qscale(j)*q(j), 5), ...
				'HorizontalAlignment', 'left', ...
				'FontUnits', 'normalized', ...
				'FontSize', 0.4, ...
				'Tag', sprintf('Edit%d', j));
		end
		
	%% Display Values
    
    % X
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.05 1-height 0.2 0.3*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', 'x:');
    
    handles.t6.t(1) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-height 0.6 0.3*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', T6(1, 4)), ...
        'Tag', 'T6');
    
    % Y
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.05 1-1.4*height 0.2 0.3*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', 'y:');
    
    handles.t6.t(2) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-1.4*height 0.6 0.3*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', T6(2, 4)));
    
    % Z
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.05 1-1.8*height 0.2 0.3*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', 'z:');
    
    handles.t6.t(3) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-1.8* height 0.6 0.3*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', T6(3, 4)));
    
    % Orientation
    switch opt.orientation
        case 'approach'
            labels = {'ax:', 'ay:', 'az:'};
        case 'eul'
            labels = {[char(hex2dec('3c6')) ':'], [char(hex2dec('3b8')) ':'], [char(hex2dec('3c8')) ':']}; % phi theta psi
        case'rpy'
            labels = {'R:', 'P:', 'Y:'};
    end
    
    %---- set up the orientation display box

    % AX
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.05 1-2.2*height 0.2 0.3*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', labels(1));
    
    handles.t6.r(1) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-2.2*height 0.6 0.3*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', T6(1, 3)));
    
    % AY
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.05 1-2.6*height 0.2 0.3*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', labels(2));
    
    handles.t6.r(2) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-2.6*height 0.6 0.3*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', T6(2, 3)));
    
    % AZ
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.05 1-3*height 0.2 0.3*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', labels(3));
    
    handles.t6.r(3) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-3*height 0.6 0.3*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', T6(3, 3)));   

	%% Callbacks and functionality
    %---- add buttons
	%Exit button
    uicontrol(panel, 'Style', 'pushbutton', ...
        'Units', 'normalized', ...
        'Position', [0.80 height * (0) + 0.01 0.15 height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.7, ...
        'CallBack', @(src,event) quit_callback(robot.model, handles), ...
        'BackgroundColor', 'white', ...
        'ForegroundColor', 'red', ...
        'String', 'X');
    
    % the record button
    handles.record = [];
    if ~isempty(opt.callback)
    uicontrol(panel, 'Style', 'pushbutton', ...
        'Units', 'normalized', ...
        'Position', [0.1 height * (0) + 0.01 0.30 height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.6, ...
        'CallBack', @(src,event) record_callback(robot.model, handles), ...
        'BackgroundColor', 'red', ...
        'ForegroundColor', 'white', ...
        'String', 'REC');
	end
    
    %---- now assign the callbacks
	n = robot.model.n + 3;
    for j = 1 : n
        % text edit box
        set(handles.edit(j), ...
            'Interruptible', 'off', ...
            'Callback', @(src,event)advTeach_callback(src, robot.model.name, j, handles));
        
        % slider
        set(handles.slider(j), ...
            'Interruptible', 'off', ...
            'BusyAction', 'queue', ...
            'Callback', @(src,event)advTeach_callback(src, robot.model.name, j, handles));
	end
end
%% advTeach Callback
function advTeach_callback(src, name, j, handles)
    switch get(src, 'Style')
		case 'slider'
			newval = get(src, 'Value');
			set(handles.edit(j), 'String', num2str(newval, 3));
		case 'edit'
			newval = str2double(get(src, 'String'));
			set(handles.slider(j), 'Value', newval);
	end
	
	%find all graphical objects tagged with robot name
	h = findobj('Tag', name);
	
	%find graphical element with name h
	if isempty(h)
		error('RTB:teach:badarg', 'No graphical robot of this name found');
	end
	
	%Find current coordinates
    currentXYZ = [str2double(get(handles.t6.t(1), 'String')) str2double(get(handles.t6.t(2), 'String')) str2double(get(handles.t6.t(3), 'String'))]';
	info = get(h(1), 'UserData');
	%Find current Joint angles
	currentJoints = info.q;
	%update the stored joint coordinates
	set(h(1), 'UserData', info);
	
	%set the goalXYZ to the current coordinates with the new coordinate changes
	if j < 4
		goalXYZ = currentXYZ;
		goalXYZ(j, 1) = newval;
		goalTr = transl(goalXYZ);
		goalJoints = handles.robot.model.ikcon(goalTr, currentJoints);
		qMatrix = handles.robot.getPoseQMatrix(currentJoints, goalJoints, 5);
		handles.robot.animateRobotMovement(qMatrix, handles.robot,  0, 0, 0, 0);
		info.q = goalJoints;
		set(h(1), 'UserData', info);
	end
	
	if j > 4
		info.q(j - 3) = newval;
		set(h(1), 'UserData', info);
		animate(handles.robot.model, info.q);
	end
	
	T6 = handles.robot.model.fkine(info.q);
	% convert orientation to desired format
	switch handles.orientation
		case 'approach'
			orient = T6(:,3);    % approach vector
		case 'eul'
			orient = tr2eul(T6, 'setopt', handles.opt);
		case'rpy'
			orient = tr2rpy(T6, 'setopt', handles.opt);
	end
	
	% update the display in the teach window
    for i = 1:3
        set(handles.t6.t(i), 'String', sprintf('%.3f', T6(i, 4)));
        set(handles.t6.r(i), 'String', sprintf('%.3f', orient(i)));
    end
    
    if ~isempty(handles.callback)
        handles.callback(handles.robot, info.q);
    end
	
end
%% Quit Callback
function quit_callback(robot, handles)
    set(handles.fig, 'ResizeFcn', '');
    delete(handles.panel);
    set(handles.curax, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1])
end

%% Resize Callback
function resize_callback(robot, handles)

    % come here on figure resize events
    fig = gcbo;   % this figure (whose callback is executing)
    fs = get(fig, 'Position');  % get size of figure
    ps = get(handles.panel, 'Position');  % get position of the panel
    % update dimensions of the axis area
    set(handles.curax, 'Units', 'pixels', ...
        'OuterPosition', [ps(3) 0 fs(3)-ps(3) fs(4)]);
    % keep the panel anchored to the top left corner
    set(handles.panel, 'Position', [1 fs(4)-ps(4) ps(3:4)]);
end
