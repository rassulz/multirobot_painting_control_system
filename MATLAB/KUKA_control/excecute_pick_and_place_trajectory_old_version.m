%% Execute Pick-and-Place Trajectory on KUKA Robot with Live Visualization & Telemetry
% ENHANCED: Real-time plotting of robot motion, joint angles, torques, and velocities
% Shows complete robot state for comprehensive troubleshooting

clc; clear; close all;

%% ===== CONFIGURATION =====
ROBOT_IP  = '172.31.17.101';  % Your robot IP
PORT      = 7000;
WAIT_TIME = 2.0;              % Seconds between waypoints (adjust as needed)

% Visualization options
ENABLE_LIVE_PLOT = true;      % Set to false to disable live plotting
PLOT_UPDATE_RATE = 5;         % Update plot every N waypoints (higher = faster execution)

% Telemetry options
ENABLE_TELEMETRY = true;      % Set to false to disable torque/velocity reading
TELEMETRY_RATE = 2;           % Read telemetry every N waypoints (higher = faster)

%% ===== LOAD TRAJECTORY =====
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║   KUKA Pick-and-Place Execution with Telemetry ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

fprintf('=== Loading Trajectory ===\n');

traj_files = dir('trajectory_pick_place_*.mat');
if isempty(traj_files)
    error('No pick-and-place trajectory files found! Run create_pick_place_trajectory.m first.');
end

fprintf('Available pick-and-place trajectories:\n');
fprintf('  # | Filename                                     | Date\n');
fprintf('----+----------------------------------------------+-------------------\n');
for i = 1:length(traj_files)
    fprintf(' %2d | %-44s | %s\n', i, traj_files(i).name, traj_files(i).date);
end

if length(traj_files) == 1
    file_idx = 1;
    fprintf('\nAuto-selecting: %s\n', traj_files(1).name);
else
    file_idx = input(sprintf('\nSelect trajectory (1-%d): ', length(traj_files)));
    if isempty(file_idx) || file_idx < 1 || file_idx > length(traj_files)
        error('Invalid selection.');
    end
end

selected_file = traj_files(file_idx).name;
fprintf('Loading: %s\n', selected_file);

data = load(selected_file);

%% ===== EXTRACT WAYPOINTS =====
if isfield(data, 'waypoints')
    waypoints = data.waypoints;
    pattern_name = data.PATTERN;
elseif isfield(data, 'trajectory_data')
    waypoints = data.trajectory_data.waypoints;
    if isfield(data.trajectory_data, 'pattern_type')
        pattern_name = data.trajectory_data.pattern_type;
    else
        pattern_name = 'unknown';
    end
else
    fields = fieldnames(data);
    waypoints = [];
    for f = 1:length(fields)
        var = data.(fields{f});
        if isnumeric(var) && size(var, 2) == 6
            waypoints = var;
            fprintf('  Found waypoints in field: %s\n', fields{f});
            break;
        end
    end
    if isempty(waypoints)
        error('Cannot find waypoints in file! Expected Nx6 matrix.');
    end
    pattern_name = 'unknown';
end

NUM_WAYPOINTS = size(waypoints, 1);

fprintf('✓ Loaded %d waypoints\n', NUM_WAYPOINTS);
fprintf('  Pattern: %s\n', pattern_name);

if size(waypoints, 2) ~= 6
    error('Invalid waypoints format! Expected Nx6 matrix [X Y Z A B C], got %dx%d', ...
          size(waypoints, 1), size(waypoints, 2));
end

%% ===== ORIENTATION CONVENTION CHECK =====
% KUKA uses two equivalent Euler representations for ~tool-down:
%   (A=0, B~180, C=0)   and   (A=-180, B~0, C=-180)
% The controller internally uses the small-|B| convention.
% If the trajectory was generated with B>90, convert it now.
traj_B_mean = mean(waypoints(:,5));
if abs(traj_B_mean) > 90
    fprintf('\n=== Orientation Convention Conversion ===\n');
    fprintf('  Trajectory uses B=%.1f\xb0 convention\n', traj_B_mean);
    fprintf('  Robot uses small-B convention (B\x2248 2\xb0)\n');
    fprintf('  Converting to robot-native representation...\n');
    
    for k = 1:size(waypoints,1)
        A_old = waypoints(k,4);
        B_old = waypoints(k,5);
        C_old = waypoints(k,6);
        
        if B_old > 0
            waypoints(k,4) = A_old - 180;
            waypoints(k,5) = 180 - B_old;
            waypoints(k,6) = C_old - 180;
        else
            waypoints(k,4) = A_old + 180;
            waypoints(k,5) = -180 - B_old;
            waypoints(k,6) = C_old + 180;
        end
    end
    
    % Ensure angle continuity (no jumps > 180 between consecutive waypoints)
    for col = [4, 6]  % A and C columns
        for k = 2:size(waypoints,1)
            while (waypoints(k,col) - waypoints(k-1,col)) > 180
                waypoints(k,col) = waypoints(k,col) - 360;
            end
            while (waypoints(k,col) - waypoints(k-1,col)) < -180
                waypoints(k,col) = waypoints(k,col) + 360;
            end
        end
    end
    
    fprintf('  Converted orientation ranges:\n');
    fprintf('    A: %.1f to %.1f\xb0\n', min(waypoints(:,4)), max(waypoints(:,4)));
    fprintf('    B: %.1f to %.1f\xb0\n', min(waypoints(:,5)), max(waypoints(:,5)));
    fprintf('    C: %.1f to %.1f\xb0\n', min(waypoints(:,6)), max(waypoints(:,6)));
    fprintf('  \x2713 Orientations converted to robot-native convention\n');
end

%% ===== DISPLAY TRAJECTORY INFO =====
fprintf('\n=== Trajectory Information ===\n');
fprintf('Position range:\n');
fprintf('  X: %.1f to %.1f mm\n', min(waypoints(:,1)), max(waypoints(:,1)));
fprintf('  Y: %.1f to %.1f mm\n', min(waypoints(:,2)), max(waypoints(:,2)));
fprintf('  Z: %.1f to %.1f mm\n', min(waypoints(:,3)), max(waypoints(:,3)));
fprintf('Orientation range:\n');
fprintf('  A: %.1f to %.1f degrees\n', min(waypoints(:,4)), max(waypoints(:,4)));
fprintf('  B: %.1f to %.1f degrees\n', min(waypoints(:,5)), max(waypoints(:,5)));
fprintf('  C: %.1f to %.1f degrees\n', min(waypoints(:,6)), max(waypoints(:,6)));
fprintf('Estimated time: %.1f seconds (%.1f minutes)\n', NUM_WAYPOINTS*WAIT_TIME, NUM_WAYPOINTS*WAIT_TIME/60);

%% ===== SAFETY CHECKS =====
fprintf('\n');
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║        REAL ROBOT EXECUTION - PICK & PLACE     ║\n');
fprintf('║               SAFETY CHECKLIST                 ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

safety_items = {
    'Robot powered ON and initialized'
    'MatlabControl.src program loaded on SmartPAD'
    'MatlabControl.src running (NOT paused)'
    'Robot in T1 mode'
    'Deadman switch ready to be held'
    'Emergency stop button accessible'
    'Workspace clear of obstacles'
    'Object placed at pick location'
    'Place location is clear'
    'KUKAVARPROXY service running on controller'
};

for i = 1:length(safety_items)
    fprintf('  ☐ %s\n', safety_items{i});
end

fprintf('\n⚠  WARNING: This will move a real robot!\n');
fprintf('   Hold the deadman switch and be ready to release if needed.\n\n');

response = input('All checks complete? Type YES (all caps) to continue: ', 's');
if ~strcmpi(strtrim(response), 'YES')
    fprintf('\n✗ Execution cancelled by user.\n');
    return;
end

%% ===== CONNECT TO ROBOT =====
fprintf('\n=== Connecting to Robot ===\n');
fprintf('IP: %s, Port: %d\n', ROBOT_IP, PORT);

try
    tcp = tcpclient(ROBOT_IP, PORT, 'Timeout', 10);
    fprintf('✓ Connected to KUKA via KUKAVARPROXY\n');
catch ME
    fprintf('\n✗ Connection failed: %s\n', ME.message);
    fprintf('\nTroubleshooting:\n');
    fprintf('  1. Verify robot IP: %s\n', ROBOT_IP);
    fprintf('  2. Check network connection\n');
    fprintf('  3. Ensure KUKAVARPROXY is running\n');
    error('Cannot connect to robot.');
end

% Ensure cleanup
cleanupObj = onCleanup(@() cleanupConnection(tcp));

%% ===== READ CURRENT ROBOT CONFIGURATION (S/T values) =====
fprintf('\n=== Reading Current Robot Configuration ===\n');
fprintf('  (S=Status and T=Turn define the IK configuration)\n');

% Read current position to get valid S (Status) and T (Turn) values
% Using wrong S/T is the #1 cause of "software limit switch" errors
ROBOT_S = 2;   % fallback default
ROBOT_T = 35;  % fallback default
try
    pos_act_str = readVar(tcp, '$POS_ACT');
    if ischar(pos_act_str) || isstring(pos_act_str)
        pos_str = char(pos_act_str);
        
        % Parse S value
        s_match = regexp(pos_str, '[,\s]S\s+(\d+)', 'tokens');
        if ~isempty(s_match)
            ROBOT_S = str2double(s_match{1}{1});
        else
            fprintf('  \x26a0 Could not parse S from $POS_ACT, using default: %d\n', ROBOT_S);
        end
        
        % Parse T value
        t_match = regexp(pos_str, '[,\s]T\s+(-?\d+)', 'tokens');
        if ~isempty(t_match)
            ROBOT_T = str2double(t_match{1}{1});
        else
            fprintf('  \x26a0 Could not parse T from $POS_ACT, using default: %d\n', ROBOT_T);
        end
        
        fprintf('  \x2713 Current S (Status): %d\n', ROBOT_S);
        fprintf('  \x2713 Current T (Turn):   %d\n', ROBOT_T);
        fprintf('  Raw $POS_ACT: %s\n', pos_str);
    else
        fprintf('  \x26a0 $POS_ACT returned non-string, using defaults S=%d, T=%d\n', ROBOT_S, ROBOT_T);
    end
catch ME
    fprintf('  \x26a0 Error reading $POS_ACT: %s\n', ME.message);
    fprintf('    Using defaults S=%d, T=%d\n', ROBOT_S, ROBOT_T);
end

% Read current joint angles for reference
try
    axis_act_str = readVar(tcp, '$AXIS_ACT');
    if ischar(axis_act_str) || isstring(axis_act_str)
        fprintf('  Current joints: %s\n', char(axis_act_str));
    end
catch
    fprintf('  \x26a0 Could not read $AXIS_ACT\n');
end

fprintf('\n  NOTE: S=%d, T=%d will be used for all waypoints.\n', ROBOT_S, ROBOT_T);
fprintf('  These values match the robot''s current IK configuration.\n\n');

%% ===== INITIALIZE LIVE PLOTTING WITH TELEMETRY =====
if ENABLE_LIVE_PLOT
    fprintf('\n=== Initializing Live Visualization with Telemetry ===\n');
    
    % Create figure with expanded layout for telemetry
    fig = figure('Name', 'KUKA Robot Execution Monitor with Telemetry', ...
                 'Position', [20, 20, 1800, 1000], ...
                 'Color', 'w');
    
    % ===== SUBPLOT 1: 3D Trajectory =====
    ax1 = subplot(3, 4, [1, 2, 5, 6]);
    hold(ax1, 'on');
    grid(ax1, 'on');
    xlabel(ax1, 'X (mm)');
    ylabel(ax1, 'Y (mm)');
    zlabel(ax1, 'Z (mm)');
    title(ax1, '3D Trajectory - Robot Motion');
    view(ax1, 45, 30);
    axis(ax1, 'equal');
    
    plot3(ax1, waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
          'Color', [0.7 0.7 0.7], 'LineWidth', 1, 'LineStyle', '--');
    plot3(ax1, waypoints(1,1), waypoints(1,2), waypoints(1,3), ...
          'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot3(ax1, waypoints(end,1), waypoints(end,2), waypoints(end,3), ...
          'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    h_exec_path = plot3(ax1, NaN, NaN, NaN, 'b-', 'LineWidth', 2.5);
    h_current = plot3(ax1, NaN, NaN, NaN, 'mo', 'MarkerSize', 15, 'MarkerFaceColor', 'm');
    legend(ax1, 'Location', 'best');
    
    % ===== SUBPLOT 2: Joint Angles =====
    ax2 = subplot(3, 4, 3);
    hold(ax2, 'on');
    grid(ax2, 'on');
    xlabel(ax2, 'Waypoint');
    ylabel(ax2, 'Angle (deg)');
    title(ax2, 'Wrist Joints (4,5,6)');
    plot(ax2, 1:NUM_WAYPOINTS, waypoints(:,4), 'r--', 'LineWidth', 1);
    plot(ax2, 1:NUM_WAYPOINTS, waypoints(:,5), 'g--', 'LineWidth', 1);
    plot(ax2, 1:NUM_WAYPOINTS, waypoints(:,6), 'b--', 'LineWidth', 1);
    h_angle_A = plot(ax2, NaN, NaN, 'r-', 'LineWidth', 2);
    h_angle_B = plot(ax2, NaN, NaN, 'g-', 'LineWidth', 2);
    h_angle_C = plot(ax2, NaN, NaN, 'b-', 'LineWidth', 2);
    legend(ax2, {'A', 'B', 'C'}, 'Location', 'best');
    
    % ===== SUBPLOT 3: Cartesian Position =====
    ax3 = subplot(3, 4, 4);
    hold(ax3, 'on');
    grid(ax3, 'on');
    xlabel(ax3, 'Waypoint');
    ylabel(ax3, 'Position (mm)');
    title(ax3, 'Cartesian Position');
    plot(ax3, 1:NUM_WAYPOINTS, waypoints(:,1), 'r--', 'LineWidth', 1);
    plot(ax3, 1:NUM_WAYPOINTS, waypoints(:,2), 'g--', 'LineWidth', 1);
    plot(ax3, 1:NUM_WAYPOINTS, waypoints(:,3), 'b--', 'LineWidth', 1);
    h_pos_X = plot(ax3, NaN, NaN, 'r-', 'LineWidth', 2);
    h_pos_Y = plot(ax3, NaN, NaN, 'g-', 'LineWidth', 2);
    h_pos_Z = plot(ax3, NaN, NaN, 'b-', 'LineWidth', 2);
    legend(ax3, {'X', 'Y', 'Z'}, 'Location', 'best');
    
    % ===== SUBPLOT 4: Joint Torques (Actual) =====
    ax4 = subplot(3, 4, 7);
    hold(ax4, 'on');
    grid(ax4, 'on');
    xlabel(ax4, 'Waypoint');
    ylabel(ax4, 'Torque (Nm)');
    title(ax4, 'Joint Torques - Actual');
    colors = lines(6);
    h_torque_act = gobjects(6, 1);
    for j = 1:6
        h_torque_act(j) = plot(ax4, NaN, NaN, '-', 'Color', colors(j,:), 'LineWidth', 1.5);
    end
    legend(ax4, {'J1','J2','J3','J4','J5','J6'}, 'Location', 'best');
    
    % ===== SUBPLOT 5: Torque Tracking (Wrist) =====
    ax5 = subplot(3, 4, 8);
    hold(ax5, 'on');
    grid(ax5, 'on');
    xlabel(ax5, 'Waypoint');
    ylabel(ax5, 'Torque (Nm)');
    title(ax5, 'Wrist Torque Tracking');
    h_torque_j4_target = plot(ax5, NaN, NaN, 'r--', 'LineWidth', 1);
    h_torque_j4_actual = plot(ax5, NaN, NaN, 'r-', 'LineWidth', 2);
    h_torque_j5_target = plot(ax5, NaN, NaN, 'g--', 'LineWidth', 1);
    h_torque_j5_actual = plot(ax5, NaN, NaN, 'g-', 'LineWidth', 2);
    h_torque_j6_target = plot(ax5, NaN, NaN, 'b--', 'LineWidth', 1);
    h_torque_j6_actual = plot(ax5, NaN, NaN, 'b-', 'LineWidth', 2);
    legend(ax5, {'J4 Tgt','J4 Act','J5 Tgt','J5 Act','J6 Tgt','J6 Act'}, 'Location', 'best');
    
    % ===== SUBPLOT 6: Joint Velocities =====
    ax6 = subplot(3, 4, 11);
    hold(ax6, 'on');
    grid(ax6, 'on');
    xlabel(ax6, 'Waypoint');
    ylabel(ax6, 'Velocity (deg/s)');
    title(ax6, 'Joint Velocities');
    h_velocity = gobjects(6, 1);
    for j = 1:6
        h_velocity(j) = plot(ax6, NaN, NaN, '-', 'Color', colors(j,:), 'LineWidth', 1.5);
    end
    legend(ax6, {'J1','J2','J3','J4','J5','J6'}, 'Location', 'best');
    
    % ===== SUBPLOT 7: Motor Currents =====
    ax7 = subplot(3, 4, 12);
    hold(ax7, 'on');
    grid(ax7, 'on');
    xlabel(ax7, 'Waypoint');
    ylabel(ax7, 'Current (A)');
    title(ax7, 'Motor Currents');
    h_current_plot = gobjects(6, 1);
    for j = 1:6
        h_current_plot(j) = plot(ax7, NaN, NaN, '-', 'Color', colors(j,:), 'LineWidth', 1.5);
    end
    legend(ax7, {'J1','J2','J3','J4','J5','J6'}, 'Location', 'best');
    
    % ===== SUBPLOT 8: Progress Bar =====
    ax8 = subplot(3, 4, 9);
    h_progress = barh(ax8, 1, 0, 'FaceColor', [0.2 0.6 1]);
    xlim(ax8, [0, 100]);
    ylim(ax8, [0.5, 1.5]);
    xlabel(ax8, 'Progress (%)');
    title(ax8, 'Execution Progress');
    set(ax8, 'YTick', []);
    grid(ax8, 'on');
    h_progress_text = text(ax8, 50, 1, '0.0%', 'HorizontalAlignment', 'center', ...
                           'FontSize', 14, 'FontWeight', 'bold');
    
    % ===== SUBPLOT 9: Status Text =====
    ax9 = subplot(3, 4, 10);
    axis(ax9, 'off');
    h_status = text(ax9, 0.1, 0.9, 'Status: Starting...', 'FontSize', 10, ...
                    'VerticalAlignment', 'top', 'FontName', 'Courier');
    
    drawnow;
    fprintf('✓ Live visualization with telemetry initialized\n');
    
    % Initialize telemetry storage
    telemetry_log = struct();
    telemetry_log.waypoint_idx = [];
    telemetry_log.torque_actual = [];
    telemetry_log.torque_target = [];
    telemetry_log.velocity = [];
    telemetry_log.current = [];
    telemetry_log.timestamps = [];
end

%% ===== MOVE TO START POSITION =====
fprintf('\n=== Moving to Start Position ===\n');

start_pos = waypoints(1, :);
fprintf('Target: X=%.1f, Y=%.1f, Z=%.1f, A=%.1f, B=%.1f, C=%.1f\n', ...
       start_pos(1), start_pos(2), start_pos(3), ...
       start_pos(4), start_pos(5), start_pos(6));

% E6POS format with S/T read from robot (avoids software limit errors)
posStr = sprintf('{X %.3f,Y %.3f,Z %.3f,A %.3f,B %.3f,C %.3f,S %d,T %d}', ...
               start_pos(1), start_pos(2), start_pos(3), ...
               start_pos(4), start_pos(5), start_pos(6), ROBOT_S, ROBOT_T);

fprintf('  E6POS: %s\n', posStr);
writeVar(tcp, 'target_pos', posStr);
pause(0.2);
writeVar(tcp, 'move_trigger', 'TRUE');

fprintf('\xe2\x8f\xb3 Moving to start position...\n');
pause(WAIT_TIME + 2);

% *** CRITICAL: Re-read S/T after start move ***
% The robot's IK solver may choose a DIFFERENT S/T configuration
% than the one read before the move. Using stale S/T values is the
% primary cause of "software limit switch" errors.
try
    pos_after = readVar(tcp, '$POS_ACT');
    if ischar(pos_after) || isstring(pos_after)
        pos_after_str = char(pos_after);
        fprintf('  Robot position after start move: %s\n', pos_after_str);
        
        % Update S value from actual position
        s_match2 = regexp(pos_after_str, '[,\s]S\s+(\d+)', 'tokens');
        if ~isempty(s_match2)
            new_S = str2double(s_match2{1}{1});
            if new_S ~= ROBOT_S
                fprintf('  *** S changed: %d -> %d (updating)\n', ROBOT_S, new_S);
                ROBOT_S = new_S;
            end
        end
        
        % Update T value from actual position
        t_match2 = regexp(pos_after_str, '[,\s]T\s+(-?\d+)', 'tokens');
        if ~isempty(t_match2)
            new_T = str2double(t_match2{1}{1});
            if new_T ~= ROBOT_T
                fprintf('  *** T changed: %d -> %d (updating)\n', ROBOT_T, new_T);
                ROBOT_T = new_T;
            end
        end
        
        fprintf('  Using S=%d, T=%d for trajectory execution\n', ROBOT_S, ROBOT_T);
    end
catch
    fprintf('  Could not re-read $POS_ACT after start move\n');
end
fprintf('✓ At start position\n');

input('\nPress ENTER to begin pick-and-place trajectory execution...', 's');

%% ===== EXECUTE TRAJECTORY WITH TELEMETRY =====
fprintf('\n');
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║    EXECUTING PICK-AND-PLACE WITH TELEMETRY     ║\n');
fprintf('╚══════════════���═════════════════════════════════╝\n\n');

start_time = tic;
success_count = 0;
failed_waypoints = [];

% Storage for executed path
executed_waypoints = zeros(NUM_WAYPOINTS, 6);

for i = 1:NUM_WAYPOINTS
    progress = (i / NUM_WAYPOINTS) * 100;
    
    % Console output
    fprintf('[%3d/%3d | %5.1f%%] ', i, NUM_WAYPOINTS, progress);
    fprintf('X=%6.1f Y=%6.1f Z=%6.1f ', waypoints(i,1), waypoints(i,2), waypoints(i,3));
    fprintf('A=%5.1f B=%5.1f C=%5.1f ', waypoints(i,4), waypoints(i,5), waypoints(i,6));

    % E6POS format with S/T read from robot at startup
    posStr = sprintf('{X %.3f,Y %.3f,Z %.3f,A %.3f,B %.3f,C %.3f,S %d,T %d}', ...
                   waypoints(i,1), waypoints(i,2), waypoints(i,3), ...
                   waypoints(i,4), waypoints(i,5), waypoints(i,6), ROBOT_S, ROBOT_T);

    success1 = writeVar(tcp, 'target_pos', posStr);
    pause(0.1);
    success2 = writeVar(tcp, 'move_trigger', 'TRUE');

    if success1 && success2
        fprintf('\x2713');
        success_count = success_count + 1;
        executed_waypoints(i, :) = waypoints(i, :);
    else
        fprintf('\x2717 FAILED');
        failed_waypoints = [failed_waypoints; i];
        executed_waypoints(i, :) = waypoints(i, :) * NaN;
    end
    
    % ===== CHECK FOR ROBOT ERRORS (joint limits, active commands) =====
    if mod(i, 5) == 0 || i == 1
        try
            axis_act = readVar(tcp, '$AXIS_ACT');
            if ischar(axis_act) || isstring(axis_act)
                axis_str = char(axis_act);
                % Parse A4 value to detect approaching limits
                a4_match = regexp(axis_str, 'A4\s+([-\d.]+)', 'tokens');
                if ~isempty(a4_match)
                    a4_val = str2double(a4_match{1}{1});
                    if abs(a4_val) > 170
                        fprintf('\n  \x26a0 WARNING: A4=%.1f\xb0 APPROACHING \xb1185\xb0 LIMIT!\n', a4_val);
                    end
                end
                % Parse A5 value
                a5_match = regexp(axis_str, 'A5\s+([-\d.]+)', 'tokens');
                if ~isempty(a5_match)
                    a5_val = str2double(a5_match{1}{1});
                    if abs(a5_val) > 110
                        fprintf('\n  \x26a0 WARNING: A5=%.1f\xb0 APPROACHING \xb1120\xb0 LIMIT!\n', a5_val);
                    end
                end
            end
        catch
            % Non-critical: continue even if joint read fails
        end
        
        % Check if robot has a stop message (error/fault)
        % Using $STOPMESS (boolean) instead of $PRO_STATE1 which may not be readable
        try
            stopmess = readVar(tcp, '$STOPMESS');
            if ischar(stopmess) || isstring(stopmess)
                stopmess_str = upper(strtrim(char(stopmess)));
                if strcmp(stopmess_str, 'TRUE')
                    fprintf('\n  \x26a0 STOP MESSAGE active on robot!\n');
                    fprintf('    Check SmartPAD for the error message.\n');
                    fprintf('    This usually means "software limit switch" or similar.\n');
                    response_err = input('    Continue execution? (y/n) [n]: ', 's');
                    if ~strcmpi(strtrim(response_err), 'y')
                        fprintf('\n\x2717 Trajectory aborted by user at waypoint %d\n', i);
                        break;
                    end
                end
            end
            % If stopmess is NaN or unreadable, just skip - not an error
        catch
            % Variable may not be readable on this controller - skip silently
        end
        
        % Periodically re-read S/T to detect configuration changes
        if mod(i, 20) == 0
            try
                pos_mid = readVar(tcp, '$POS_ACT');
                if ischar(pos_mid) || isstring(pos_mid)
                    pos_mid_str = char(pos_mid);
                    t_mid = regexp(pos_mid_str, '[,\s]T\s+(-?\d+)', 'tokens');
                    s_mid = regexp(pos_mid_str, '[,\s]S\s+(\d+)', 'tokens');
                    if ~isempty(t_mid)
                        mid_T = str2double(t_mid{1}{1});
                        if mid_T ~= ROBOT_T
                            fprintf('\n  \x26a0 T changed mid-trajectory: %d -> %d (updating)\n', ROBOT_T, mid_T);
                            ROBOT_T = mid_T;
                        end
                    end
                    if ~isempty(s_mid)
                        mid_S = str2double(s_mid{1}{1});
                        if mid_S ~= ROBOT_S
                            fprintf('\n  \x26a0 S changed mid-trajectory: %d -> %d (updating)\n', ROBOT_S, mid_S);
                            ROBOT_S = mid_S;
                        end
                    end
                end
            catch
            end
        end
    end
    
    % ===== READ TELEMETRY =====
    if ENABLE_TELEMETRY && (mod(i, TELEMETRY_RATE) == 0 || i == 1 || i == NUM_WAYPOINTS)
        try
            telem = readTelemetry(tcp);
            
            % Store telemetry
            telemetry_log.waypoint_idx = [telemetry_log.waypoint_idx; i];
            telemetry_log.torque_actual = [telemetry_log.torque_actual; telem.torque_actual'];
            telemetry_log.torque_target = [telemetry_log.torque_target; telem.torque_target'];
            telemetry_log.velocity = [telemetry_log.velocity; telem.velocity'];
            telemetry_log.current = [telemetry_log.current; telem.current'];
            telemetry_log.timestamps = [telemetry_log.timestamps; telem.timestamp];
            
            % Print compact telemetry to console
            fprintf(' | T:[%4.1f %4.1f %4.1f %4.1f %4.1f %4.1f]', telem.torque_actual);
        catch telemetry_err
            fprintf(' | Telem ERR');
        end
    end
    
    fprintf('\n');

    % ===== UPDATE LIVE PLOT =====
    if ENABLE_LIVE_PLOT && (mod(i, PLOT_UPDATE_RATE) == 0 || i == NUM_WAYPOINTS)
        try
            % Update 3D path
            valid_idx = ~isnan(executed_waypoints(1:i, 1));
            set(h_exec_path, 'XData', executed_waypoints(valid_idx, 1), ...
                             'YData', executed_waypoints(valid_idx, 2), ...
                             'ZData', executed_waypoints(valid_idx, 3));
            set(h_current, 'XData', waypoints(i, 1), ...
                           'YData', waypoints(i, 2), ...
                           'ZData', waypoints(i, 3));
            
            % Update angles
            set(h_angle_A, 'XData', 1:i, 'YData', executed_waypoints(1:i, 4));
            set(h_angle_B, 'XData', 1:i, 'YData', executed_waypoints(1:i, 5));
            set(h_angle_C, 'XData', 1:i, 'YData', executed_waypoints(1:i, 6));
            
            % Update positions
            set(h_pos_X, 'XData', 1:i, 'YData', executed_waypoints(1:i, 1));
            set(h_pos_Y, 'XData', 1:i, 'YData', executed_waypoints(1:i, 2));
            set(h_pos_Z, 'XData', 1:i, 'YData', executed_waypoints(1:i, 3));
            
            % Update telemetry plots
            if ENABLE_TELEMETRY && ~isempty(telemetry_log.waypoint_idx)
                idx_list = telemetry_log.waypoint_idx;
                
                % Torques actual (all joints)
                for j = 1:6
                    set(h_torque_act(j), 'XData', idx_list, ...
                        'YData', telemetry_log.torque_actual(:, j));
                end
                
                % Torques target vs actual (wrist)
                set(h_torque_j4_target, 'XData', idx_list, 'YData', telemetry_log.torque_target(:, 4));
                set(h_torque_j4_actual, 'XData', idx_list, 'YData', telemetry_log.torque_actual(:, 4));
                set(h_torque_j5_target, 'XData', idx_list, 'YData', telemetry_log.torque_target(:, 5));
                set(h_torque_j5_actual, 'XData', idx_list, 'YData', telemetry_log.torque_actual(:, 5));
                set(h_torque_j6_target, 'XData', idx_list, 'YData', telemetry_log.torque_target(:, 6));
                set(h_torque_j6_actual, 'XData', idx_list, 'YData', telemetry_log.torque_actual(:, 6));
                
                % Velocities
                for j = 1:6
                    set(h_velocity(j), 'XData', idx_list, ...
                        'YData', telemetry_log.velocity(:, j));
                end
                
                % Currents
                for j = 1:6
                    set(h_current_plot(j), 'XData', idx_list, ...
                        'YData', telemetry_log.current(:, j));
                end
            end
            
            % Update progress
            set(h_progress, 'XData', progress);
            set(h_progress_text, 'String', sprintf('%.1f%%', progress));
            
            % Update status
            elapsed = toc(start_time);
            remaining = (elapsed / i) * (NUM_WAYPOINTS - i);
            status_str = sprintf(['Waypoint: %d / %d\n' ...
                                  'Progress: %.1f%%\n' ...
                                  'Success: %d / %d\n' ...
                                  'Elapsed: %.1f s\n' ...
                                  'Remaining: %.1f s'], ...
                                 i, NUM_WAYPOINTS, progress, ...
                                 success_count, i, elapsed, remaining);
            set(h_status, 'String', status_str);
            
            drawnow limitrate;
        catch plot_err
            warning(plot_err.identifier, '%s', plot_err.message);
        end
    end

    pause(WAIT_TIME);
end

execution_time = toc(start_time);

%% ===== RETURN HOME =====
fprintf('\n=== Returning to Home ===\n');

home_pos_vec = waypoints(1, :);
home_pos = sprintf('{X %.3f,Y %.3f,Z %.3f,A %.3f,B %.3f,C %.3f,S %d,T %d}', ...
                   home_pos_vec(1), home_pos_vec(2), home_pos_vec(3), ...
                   home_pos_vec(4), home_pos_vec(5), home_pos_vec(6), ROBOT_S, ROBOT_T);

fprintf('  E6POS: %s\n', home_pos);
writeVar(tcp, 'target_pos', home_pos);
pause(0.2);
writeVar(tcp, 'move_trigger', 'TRUE');
pause(WAIT_TIME + 2);
fprintf('✓ At home position\n');

%% ===== SAVE EXECUTION LOG WITH TELEMETRY =====
fprintf('\n=== Saving Execution Log with Telemetry ===\n');

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
log_filename = sprintf('execution_log_%s.mat', timestamp);

execution_log = struct();
execution_log.trajectory_file = selected_file;
execution_log.pattern_name = pattern_name;
execution_log.planned_waypoints = waypoints;
execution_log.executed_waypoints = executed_waypoints;
execution_log.num_waypoints = NUM_WAYPOINTS;
execution_log.success_count = success_count;
execution_log.failed_waypoints = failed_waypoints;
execution_log.execution_time = execution_time;
execution_log.timestamp = timestamp;
execution_log.robot_ip = ROBOT_IP;

% Add telemetry data
if ENABLE_TELEMETRY && exist('telemetry_log', 'var')
    execution_log.telemetry = telemetry_log;
    fprintf('✓ Telemetry data included in log\n');
end

save(log_filename, 'execution_log');
fprintf('✓ Execution log saved: %s\n', log_filename);

%% ===== FINAL PLOT UPDATE =====
if ENABLE_LIVE_PLOT
    sgtitle(fig, sprintf('Execution Complete: %d/%d waypoints (%.1f%%) | Telemetry: %d samples', ...
                         success_count, NUM_WAYPOINTS, (success_count/NUM_WAYPOINTS)*100, ...
                         length(telemetry_log.waypoint_idx)), ...
            'FontSize', 16, 'FontWeight', 'bold');
    
    fig_filename = sprintf('execution_plot_%s.png', timestamp);
    saveas(fig, fig_filename);
    fprintf('✓ Plot saved: %s\n', fig_filename);
end

%% ===== SUMMARY =====
fprintf('\n');
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║            EXECUTION COMPLETE                  ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

fprintf('=== Results ===\n');
fprintf('  Total waypoints:    %d\n', NUM_WAYPOINTS);
fprintf('  Successful:         %d (%.1f%%)\n', success_count, (success_count/NUM_WAYPOINTS)*100);
fprintf('  Failed:             %d\n', length(failed_waypoints));
fprintf('  Execution time:     %.1f seconds (%.1f minutes)\n', execution_time, execution_time/60);
fprintf('  Avg per waypoint:   %.2f seconds\n', execution_time/NUM_WAYPOINTS);

if ~isempty(failed_waypoints)
    fprintf('\n⚠  Failed waypoints: ');
    fprintf('%d ', failed_waypoints);
    fprintf('\n');
end

fprintf('\n=== Telemetry Summary ===\n');
if ENABLE_TELEMETRY && exist('telemetry_log', 'var') && ~isempty(telemetry_log.waypoint_idx)
    fprintf('  Samples collected:  %d\n', length(telemetry_log.waypoint_idx));
    if ~isempty(telemetry_log.torque_actual)
        fprintf('  Max torques (%%):    [%.1f %.1f %.1f %.1f %.1f %.1f]\n', ...
            max(abs(telemetry_log.torque_actual), [], 1));
    else
        fprintf('  Torques:            not available on this controller\n');
    end
    if ~isempty(telemetry_log.velocity)
        fprintf('  Max velocities (%%): [%.1f %.1f %.1f %.1f %.1f %.1f]\n', ...
            max(abs(telemetry_log.velocity), [], 1));
    else
        fprintf('  Velocities:         not available on this controller\n');
    end
    if ~isempty(telemetry_log.current)
        fprintf('  Max currents (A):   [%.1f %.1f %.1f %.1f %.1f %.1f]\n', ...
            max(abs(telemetry_log.current), [], 1));
    else
        fprintf('  Currents:           not available on this controller\n');
    end
else
    fprintf('  Telemetry: No samples collected\n');
    fprintf('  (Variables like $TORQUE_AXIS_ACT may not be available via KUKAVARPROXY)\n');
end

fprintf('\n=== Files Saved ===\n');
fprintf('  Execution log:  %s\n', log_filename);
if ENABLE_LIVE_PLOT
    fprintf('  Plot image:     %s\n', fig_filename);
end

fprintf('\n✓ Pick-and-place sequence complete!\n\n');

%% ===== ANALYZE JOINT MOTION =====
fprintf('=== Joint Motion Analysis ===\n');
fprintf('Position changes:\n');
fprintf('  X: %.1f mm (%.1f to %.1f)\n', ...
    range(executed_waypoints(:,1)), min(executed_waypoints(:,1)), max(executed_waypoints(:,1)));
fprintf('  Y: %.1f mm (%.1f to %.1f)\n', ...
    range(executed_waypoints(:,2)), min(executed_waypoints(:,2)), max(executed_waypoints(:,2)));
fprintf('  Z: %.1f mm (%.1f to %.1f)\n', ...
    range(executed_waypoints(:,3)), min(executed_waypoints(:,3)), max(executed_waypoints(:,3)));

fprintf('Orientation changes:\n');
fprintf('  A (Joint 4): %.1f° (%.1f to %.1f)\n', ...
    range(executed_waypoints(:,4)), min(executed_waypoints(:,4)), max(executed_waypoints(:,4)));
fprintf('  B (Joint 5): %.1f° (%.1f to %.1f)\n', ...
    range(executed_waypoints(:,5)), min(executed_waypoints(:,5)), max(executed_waypoints(:,5)));
fprintf('  C (Joint 6): %.1f° (%.1f to %.1f)\n', ...
    range(executed_waypoints(:,6)), min(executed_waypoints(:,6)), max(executed_waypoints(:,6)));

fprintf('\n');

%% ===== HELPER FUNCTIONS =====

function success = writeVar(tcp, varName, varValue)
    msgId = uint16(1);
    mode  = uint8(1);   % WRITE

    varNameBytes  = uint8(varName);
    varValueBytes = uint8(varValue);

    varNameLen  = uint16(length(varNameBytes));
    varValueLen = uint16(length(varValueBytes));

    contentLen = uint16( ...
        1 + ...
        2 + length(varNameBytes) + ...
        2 + length(varValueBytes) ...
    );

    msgIdBytes       = typecast(swapbytes(msgId),      'uint8');
    contentLenBytes  = typecast(swapbytes(contentLen), 'uint8');
    varNameLenBytes  = typecast(swapbytes(varNameLen), 'uint8');
    varValueLenBytes = typecast(swapbytes(varValueLen),'uint8');

    msg = [ ...
        msgIdBytes(:).' ...
        contentLenBytes(:).' ...
        mode ...
        varNameLenBytes(:).' ...
        varNameBytes ...
        varValueLenBytes(:).' ...
        varValueBytes ...
    ];

    write(tcp, msg);

    pause(0.05);
    success = true;
    if tcp.NumBytesAvailable > 0
        response = read(tcp, tcp.NumBytesAvailable);
        if length(response) >= 3
            tail = response(end-2:end);
            success = all(tail == [0 1 1]);
        end
    end
end

function value = readVar(tcp, varName)
    % Reads a variable from KUKA controller via KUKAVARPROXY
    msgId = uint16(1);
    mode  = uint8(0);   % READ mode
    
    varNameBytes = uint8(varName);
    varNameLen   = uint16(length(varNameBytes));
    contentLen = uint16(1 + 2 + length(varNameBytes));
    
    msgIdBytes      = typecast(swapbytes(msgId),      'uint8');
    contentLenBytes = typecast(swapbytes(contentLen), 'uint8');
    varNameLenBytes = typecast(swapbytes(varNameLen), 'uint8');
    
    msg = [ ...
        msgIdBytes(:).' ...
        contentLenBytes(:).' ...
        mode ...
        varNameLenBytes(:).' ...
        varNameBytes ...
    ];
    
    write(tcp, msg);
    pause(0.05);
    
    if tcp.NumBytesAvailable > 0
        response = read(tcp, tcp.NumBytesAvailable);
        if length(response) > 7
            valueLen = double(swapbytes(typecast(response(6:7), 'uint16')));
            if length(response) >= 7 + valueLen
                valueBytes = response(8:(7+valueLen));
                valueStr = char(valueBytes);
                value = str2double(valueStr);
                if isnan(value)
                    value = valueStr;
                end
                return;
            end
        end
    end
    value = NaN;
end

function telemetry = readTelemetry(tcp)
    % Reads telemetry from KUKA controller using real system variables.
    %
    % KUKA system variable names:
    %   Joint positions:  $AXIS_ACT          (E6AXIS struct)
    %   Joint torques:    $TORQUE_AXIS_ACT[] (REAL array, % of max torque)
    %   Joint velocities: $VEL_AXIS_ACT[]    (REAL array, % of max velocity)
    %   Motor currents:   $CURR_ACT[]        (REAL array, Amps)
    %
    % Some variables may not be available on all KRC versions.
    % The function tries multiple variable name patterns.
    
    telemetry = struct();
    
    % --- Joint angles from $AXIS_ACT (most reliable) ---
    telemetry.joint_angles = zeros(6, 1);
    try
        axis_str = readVar(tcp, '$AXIS_ACT');
        if ischar(axis_str) || isstring(axis_str)
            for j = 1:6
                tok = regexp(char(axis_str), sprintf('A%d\\s+([-\\d.]+)', j), 'tokens');
                if ~isempty(tok)
                    telemetry.joint_angles(j) = str2double(tok{1}{1});
                end
            end
        end
    catch
    end
    
    % --- Torques (try multiple variable name patterns) ---
    telemetry.torque_actual = zeros(6, 1);
    torque_read = false;
    % Pattern 1: $TORQUE_AXIS_ACT[i] (KRC4)
    try
        for j = 1:6
            val = readVar(tcp, sprintf('$TORQUE_AXIS_ACT[%d]', j));
            if isnumeric(val) && ~isnan(val)
                telemetry.torque_actual(j) = val;
                torque_read = true;
            end
        end
    catch
    end
    % Pattern 2: $TORMON_DAT[i].TORQUE_ACT (some KRC versions)
    if ~torque_read
        try
            for j = 1:6
                val = readVar(tcp, sprintf('$TORMON_DAT[%d].TORQUE_ACT', j));
                if isnumeric(val) && ~isnan(val)
                    telemetry.torque_actual(j) = val;
                    torque_read = true;
                end
            end
        catch
        end
    end
    
    % Torque target (may not be available)
    telemetry.torque_target = zeros(6, 1);
    try
        for j = 1:6
            val = readVar(tcp, sprintf('$TORQUE_AXIS_CMD[%d]', j));
            if isnumeric(val) && ~isnan(val)
                telemetry.torque_target(j) = val;
            end
        end
    catch
    end
    
    % --- Velocities ---
    telemetry.velocity = zeros(6, 1);
    try
        for j = 1:6
            val = readVar(tcp, sprintf('$VEL_AXIS_ACT[%d]', j));
            if isnumeric(val) && ~isnan(val)
                telemetry.velocity(j) = val;
            end
        end
    catch
    end
    
    % --- Motor currents ---
    telemetry.current = zeros(6, 1);
    try
        for j = 1:6
            val = readVar(tcp, sprintf('$CURR_ACT[%d]', j));
            if isnumeric(val) && ~isnan(val)
                telemetry.current(j) = val;
            end
        end
    catch
    end
    
    telemetry.timestamp = now;
end

function cleanupConnection(tcp)
    try
        if isvalid(tcp)
            clear tcp;
            fprintf('\n✓ TCP connection closed\n');
        end
    catch
        % Ignore cleanup errors
    end
end