function waypointsStruct = gen_waypointSet_00(homeConfig)
% create waypoints
numWaypoints = 10;
delta = 50;
x_waypoints = linspace(homeConfig(1)-delta, homeConfig(1)+delta, numWaypoints);
y_waypoints = linspace(homeConfig(2)-delta, homeConfig(2)+delta, numWaypoints);
z_waypoints = linspace(homeConfig(3)-delta, homeConfig(3)+delta, numWaypoints);
waypoints = zeros(3, numWaypoints);
for i = 1:numWaypoints
    waypoints(:, i) = [x_waypoints(i); y_waypoints(i); z_waypoints(i)];
end
% create orientations
orientations = [];
% create waypoint times (reached waypoint instant)
waypointTimes = 0:numWaypoints:(numWaypoints-1)*numWaypoints;
% define velocity boundaries
waypointVels = 0.1 * ...
    [0 0 1; -1 0 0; 0 0 -1; 0 1 0; 0 0 1; 0 0 1; -1 0 0; 0 0 -1; 1 0 0; 0 0 1]';
% define accelerations boundaries
waypointAccels = zeros(size(waypointVels));

waypointsStruct = struct( ...
    'waypoints', waypoints, ...
    'velocities', waypointVels, ...
    'accelerations', waypointAccels, ...
    'times', waypointTimes, ...
    'orientations', orientations);
end