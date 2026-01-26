function [fastRestart, waypointsStruct] = enableFastRestartIfPossible(modelName, lastRun, config)
% reload waypoints
waypointsStruct     = loadWaypoints(config.trajectory, config.homeConfig);
assignin("base", 'waypointsStruct', waypointsStruct);

    % define function handle to set the "FastRestart" parameter
    function setStopTime(m, t)
        printlog('Setting "StopTime"=%ds...', max(t));
        set_param(m, 'StopTime', num2str(max(t)));
    end

    function setPolynomial(m, s, p)
        printlog('Setting TrajectoryGenerator "PolynomialMethod"="%s"...', p);
        set_param(sprintf('%s/TrajectoryPlanning/%s/TrajectoryGenerator', m, char(s)), 'Method', p);
    end

    function setFastRestart(m, u)
        if strcmpi(u, 'off')
            com = 'enabled';
        else
            com = 'skipped';
        end
        printlog('Setting "FastRestart"="%s"... -> Compilation %s:', u, com);
        set_param(m, 'FastRestart', u);
    end

setStopTime(modelName, waypointsStruct.times);

% check if the this is the first simulation. If yes -> disable "FastRestart"
if isempty(lastRun.trajectory) || ...
        isempty(lastRun.space) || ...
        isempty(lastRun.waypoints) || ...
        isempty(lastRun.polynomial)
    setFastRestart(modelName, 'off');
    fprintf('* No previous simulation can be retrieved.\n');
    setPolynomial(modelName, config.space, config.polynomial);
    fastRestart = false;
    return;
end

% build condition for "FastRestart" option
isPolynomialEqual   = isequal(lastRun.polynomial, config.polynomial);
isSpaceEqual        = isequal(lastRun.space, config.space);
isTrajectoryEqual   = isequal(lastRun.trajectory, config.trajectory);
isWaypointSetEqual  = isequal(waypointsStruct, lastRun.waypoints);
fastRestart         = isTrajectoryEqual && isSpaceEqual && isWaypointSetEqual && isPolynomialEqual;

% set "FastRestart=on" if all the conditions are verified (nothing changed
% from the previous run).
if fastRestart
    setFastRestart(modelName, 'on');
    return;
end

% set FastRestart=off and gather messages
setFastRestart(modelName, 'off');
fields = {}; messages = {};
if ~isTrajectoryEqual
    fields{end+1} = 'trajectory';
    messages{end+1} = sprintf(' ["%d"~="%d"]', config.trajectory, lastRun.trajectory);
end
if ~isSpaceEqual
    fields{end+1} = 'space';
    messages{end+1} = sprintf(' ["%s"~="%s"]', char(config.space), char(lastRun.space));
end
if ~isPolynomialEqual
    fields{end+1} = 'polynomial';
    messages{end+1} = sprintf(' ["%s"~="%s"]', char(config.polynomial), char(lastRun.polynomial));
    setPolynomial(modelName, config.space, config.polynomial);
end
if ~isWaypointSetEqual
    fields{end+1} = 'waypoints';
    messages{end+1} = '';
end
for i=1:numel(fields)
    fprintf('* Current %s differs from previous simulation%s.\n', fields{i}, messages{i});
end
end
