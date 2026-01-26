function [waypointsStruct] = loadWaypoints(messID, homeConfig)
genWaypointsFunName = sprintf('gen_waypointSet_%02d', messID);
%fprintf('Running %s...\n', genWaypointsFunName);
genWaypointsFun = str2func(genWaypointsFunName);
waypointsStruct = genWaypointsFun(homeConfig);
end

