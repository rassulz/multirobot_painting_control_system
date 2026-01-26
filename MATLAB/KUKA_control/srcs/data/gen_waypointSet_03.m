function waypointsStruct = gen_waypointSet_03(homeConfig)
%      _____+y
%     /|
%    / |
% z+/  |
%     +x
% ---------- %
% DEFINITION %
% ---------- %
% create waypoints
xH = homeConfig(1); yH = homeConfig(2); zH = homeConfig(3);
waypoints = [
    xH+100, yH+500, zH+400;
	xH+100, yH-500, zH+400;
	xH-300, yH-600, zH-300;
	xH-300, yH+600, zH-100;
	xH+100, yH+500, zH+400;
    ]';
numWaypoints = size(waypoints, 2);

% create orientations
orientations = [
    0, pi/2, pi/8;
    0, 0, 0;
    0, 0, 0;
    pi/2, 0, 0;
    0, 0, 0]';
%orientations = zeros(size(waypoints));
% create waypoint times (reached waypoint instant)
step = 5;
waypointTimes = 0:step:(numWaypoints-1)*step;
% define velocity boundaries
% waypointVels = zeros(size(waypoints));
waypointVels = 0.25 * ...
    [0  -0.5  0; 
    -1  -1  0; 
     0  -1  1; 
     1   1  0;  
     0  0  0]';
% define accelerations boundaries
waypointAccels = zeros(size(waypointVels));

% ---------- %
% CHECK-SIZE %
% ---------- %
% check waypoint times dimension
if numWaypoints ~= numel(waypointTimes)
    warning('WaypointTimes length [%d] differs from number of Waypoints [%d]', ...
        numel(waypointTimes), numWaypoints);
end
% check waypoint velocities dimension
sizeWay = size(waypoints);
sizeVel = size(waypointVels);
if any(sizeWay ~= sizeVel)
    sizeVelC = num2cell(sizeVel);
    sizeWayC = num2cell(sizeWay);
    error('WaypointVels size [%d %d] differs from Waypoints matrix dimension [%d %d]', ...
        sizeVelC{:}, sizeWayC{:});
end
% check waypoint acceleration dimension
sizeAcc = size(waypointAccels);
if any(size(waypointVels) ~= size(waypoints))
    sizeAccC = num2cell(size(sizeAcc));
    sizeWayC = num2cell(size(waypoints));
    error('WaypointAccels size [%d %d] differs from Waypoints matrix dimension [%d %d]', ...
        sizeAccC{:}, sizeWayC{:});
end

% ---------- %
%   RETURN   %
% ---------- %
waypointsStruct = struct( ...
    'waypoints', waypoints, ...
    'velocities', waypointVels, ...
    'accelerations', waypointAccels, ...
    'times', waypointTimes, ...
    'orientations', orientations);
end
% 
% 
% % create waypoints
% numWaypoints = 5;
% xH = homeConfig(1); yH = homeConfig(2); zH = homeConfig(3);
% x_waypoints = [xH-50,   xH+80,  xH-60,  xH+70,  xH-50];
% y_waypoints = [yH-200,  yH+100, yH-100, yH+100, yH-200];
% z_waypoints = [zH-50, 	zH-80,  zH+60,  zH+70,  zH-50];
% waypoints = zeros(3, numWaypoints);
% for i = 1:numWaypoints
%     waypoints(:, i) = [x_waypoints(i); y_waypoints(i); z_waypoints(i)];
% end
% % create orientations
% orientations = [-pi/2, 0, 0;
%     pi/4, 0, 0;
%     0, -pi/2, 0;
%     0, 0, pi/4;
%     0, 0, pi/2]';
% % create waypoint times (reached waypoint instant)
% waypointTimes = 0:numWaypoints:(numWaypoints-1)*numWaypoints;
% % define velocity boundaries
% waypointVels = 0.5 * ...
%     [0 0 1; -1 0 0; 0 0 -1; 0 1 0; 0 1 0]';
% % define accelerations boundaries
% waypointAccels = zeros(size(waypointVels));
% 
% waypointsStruct = struct( ...
%     'waypoints', waypoints, ...
%     'velocities', waypointVels, ...
%     'accelerations', waypointAccels, ...
%     'times', waypointTimes, ...
%     'orientations', orientations);
% end