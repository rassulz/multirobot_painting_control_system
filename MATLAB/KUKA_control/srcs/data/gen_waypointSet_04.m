function waypointsStruct = gen_waypointSet_04(homeConfig)
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
    xH+500, yH-250, zH+300;
    xH+500, yH-250, zH+100;
    xH-100, yH-100, zH-100;
    xH-100, yH+100, zH-100;
    xH+500, yH+250, zH+100;
    xH+500, yH+250, zH+300;
    ]';
numWaypoints = size(waypoints, 2);
% create orientations
orientations = [
    pi/2, -pi/8, 0;
    pi/2, -pi/8, 0;
    0, 0, 0;
    0, 0, 0;
    pi/2, 0, pi/2;
    pi/2, 0, pi/2]';
% create waypoint times (reached waypoint instant)
step = 5;
waypointTimes = 0:step:(numWaypoints-1)*step;
% define velocity boundaries
% waypointVels = zeros(size(waypoints));
waypointVels = 0.25 * ...
    [0 0 0; 0 0 -1; -1 -1 -1; 0 1 0; 1 1 1; 0 0 1]';
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
    warning('WaypointVels size [%d %d] differs from Waypoints matrix dimension [%d %d]', ...
        sizeVelC{:}, sizeWayC{:});
end
% check waypoint acceleration dimension
sizeAcc = size(waypointAccels);
if any(size(waypointVels) ~= size(waypoints))
    sizeAccC = num2cell(size(sizeAcc));
    sizeWayC = num2cell(size(waypoints));
    warning('WaypointAccels size [%d %d] differs from Waypoints matrix dimension [%d %d]', ...
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