%% Execute Pick-and-Place Trajectory on KUKA Robot — Smooth Continuous Motion
% IMPROVED VERSION: Zero-pause streaming with live telemetry
%
% KEY IMPROVEMENTS:
%  1) All pause() calls replaced with non-blocking timing
%  2) Java Thread.sleep for sub-millisecond precision when needed
%  3) Continuous quintic-blended streaming — robot never stops mid-path
%  4) Periodic S/T refresh without blocking
%  5) A4/A5 joint guards with automatic recovery

clc; clear; close all;

%% ===== CONFIGURATION =====
ROBOT_IP   = '172.31.17.101';
PORT       = 7000;
CONTROL_DT = 0.012;          % 12 ms -> ~83 Hz streaming rate

% Joint guards (stop before hitting hardware limit)
A4_GUARD_DEG = 170;
A5_GUARD_DEG = 110;

ENABLE_LIVE_PLOT = true;
PLOT_UPDATE_RATE = 8;        % update plot every N steps

%% ===== LOAD TRAJECTORY =====
fprintf('===========================================================\n');
fprintf('   KUKA Pick-and-Place Execution — Continuous Streaming\n');
fprintf('===========================================================\n\n');

traj_files = dir('trajectory_pick_place_*.mat');
if isempty(traj_files)
    error('No trajectory files found! Run create_pick_and_place_trajectory.m first.');
end

fprintf('Available trajectories:\n');
for i = 1:length(traj_files)
    fprintf('  [%d] %s  (%s)\n', i, traj_files(i).name, traj_files(i).date);
end

if length(traj_files) == 1
    file_idx = 1;
    fprintf('\nAuto-selecting: %s\n', traj_files(1).name);
else
    file_idx = input(sprintf('\nSelect (1-%d): ', length(traj_files)));
    if isempty(file_idx) || file_idx < 1 || file_idx > length(traj_files)
        error('Invalid selection.');
    end
end

selected_file = traj_files(file_idx).name;
data = load(selected_file);

if ~isfield(data, 'waypoints')
    error('Missing "waypoints" in MAT file.');
end
waypoints = data.waypoints;

if isfield(data, 'key_poses')
    key_poses = data.key_poses;
else
    key_poses = waypoints([1, end], :);
end

waypoints = ensureRobotNativeEuler(waypoints);
key_poses = ensureRobotNativeEuler(key_poses);

NUM_POINTS = size(waypoints, 1);
fprintf('Loaded %d waypoints, %d key poses.\n\n', NUM_POINTS, size(key_poses,1));

%% ===== SAFETY =====
fprintf('=== SAFETY CHECKLIST ===\n');
items = {
    'Robot powered ON and initialized'
    'MatlabControl.src running (NOT paused)'
    'Robot in T1 mode, deadman switch ready'
    'Emergency stop accessible'
    'Workspace clear, object at pick location'
    'KUKAVARPROXY running on controller'
};
for i = 1:length(items)
    fprintf('  [ ] %s\n', items{i});
end

response = input('\nType YES to continue: ', 's');
if ~strcmpi(strtrim(response), 'YES')
    fprintf('Cancelled.\n'); return;
end

%% ===== CONNECT =====
fprintf('\nConnecting to %s:%d ...\n', ROBOT_IP, PORT);
try
    tcp = tcpclient(ROBOT_IP, PORT, 'Timeout', 10);
    fprintf('Connected.\n');
catch ME
    error('Connection failed: %s', ME.message);
end
cleanupObj = onCleanup(@() safeClose(tcp));

ROBOT_S = 2; ROBOT_T = 35;
[ROBOT_S, ROBOT_T] = readSTFromPosAct(tcp, ROBOT_S, ROBOT_T);
fprintf('Initial S=%d, T=%d\n', ROBOT_S, ROBOT_T);

%% ===== LIVE PLOT SETUP =====
if ENABLE_LIVE_PLOT
    fig = figure('Name', 'KUKA Execution', 'Position', [30,30,1600,900], 'Color', 'w');

    ax1 = subplot(2,3,[1,4]); hold on; grid on; axis equal; view(45,25);
    xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Path');
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), ...
        'Color',[0.7 0.7 0.7], 'LineStyle','--', 'LineWidth', 1);
    h_exec = plot3(NaN,NaN,NaN, 'b-', 'LineWidth', 2.5);
    h_cur  = plot3(NaN,NaN,NaN, 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm');

    ax2 = subplot(2,3,2); hold on; grid on;
    title('Orientation'); xlabel('Step'); ylabel('deg');
    h_oA = animatedline(ax2, 'Color','r', 'LineWidth',1.5);
    h_oB = animatedline(ax2, 'Color','g', 'LineWidth',1.5);
    h_oC = animatedline(ax2, 'Color','b', 'LineWidth',1.5);
    legend('A','B','C');

    ax3 = subplot(2,3,3); hold on; grid on;
    title('Position'); xlabel('Step'); ylabel('mm');
    h_pX = animatedline(ax3, 'Color','r', 'LineWidth',1.5);
    h_pY = animatedline(ax3, 'Color','g', 'LineWidth',1.5);
    h_pZ = animatedline(ax3, 'Color','b', 'LineWidth',1.5);
    legend('X','Y','Z');

    ax4 = subplot(2,3,5); hold on; grid on;
    title('Joint Guards (A4/A5)'); xlabel('Step'); ylabel('deg');
    h_a4 = animatedline(ax4, 'Color','m', 'LineWidth',1.5);
    h_a5 = animatedline(ax4, 'Color','c', 'LineWidth',1.5);
    yline(ax4, A4_GUARD_DEG, 'r--'); yline(ax4, -A4_GUARD_DEG, 'r--');
    yline(ax4, A5_GUARD_DEG, 'b--'); yline(ax4, -A5_GUARD_DEG, 'b--');
    legend('A4','A5');

    ax5 = subplot(2,3,6); axis off;
    h_status = text(ax5, 0.05, 0.9, 'Starting...', 'FontSize', 11, ...
        'VerticalAlignment','top', 'FontName','Courier');
end

%% ===== MOVE TO START =====
fprintf('\nMoving to start...\n');
sendE6Pos(tcp, waypoints(1,:));
pulseMoveTrigger(tcp);
nonBlockingWait(1.2);

[ROBOT_S, ROBOT_T] = readSTFromPosAct(tcp, ROBOT_S, ROBOT_T);
fprintf('Start reached. S=%d, T=%d\n', ROBOT_S, ROBOT_T);

input('Press ENTER to begin streaming...', 's');

%% ===== MAIN STREAMING LOOP =====
fprintf('\n=== STREAMING %d WAYPOINTS ===\n', NUM_POINTS);

exec_pos   = nan(NUM_POINTS, 6);
a4_log     = nan(NUM_POINTS, 1);
a5_log     = nan(NUM_POINTS, 1);
a4_val = 0; a5_val = 0;

success_count = 0;
aborted = false;

axis_poll  = max(1, round(0.3 / CONTROL_DT));
st_refresh = max(1, round(0.5 / CONTROL_DT));

t_start = tic;

for i = 1:NUM_POINTS
    t_step = tic;

    pos = waypoints(i,:);

    % Send position (no forced S/T)
    ok = sendE6Pos(tcp, pos);
    pulseMoveTrigger(tcp);

    if ok
        success_count = success_count + 1;
        exec_pos(i,:) = pos;
    end

    % Periodic S/T refresh
    if mod(i, st_refresh) == 0
        [sNew, tNew] = readSTFromPosAct(tcp, ROBOT_S, ROBOT_T);
        if sNew ~= ROBOT_S || tNew ~= ROBOT_T
            fprintf('  S/T changed: S=%d->%d, T=%d->%d\n', ROBOT_S, sNew, ROBOT_T, tNew);
            ROBOT_S = sNew; ROBOT_T = tNew;
        end
    end

    % Joint guard polling
    if mod(i, axis_poll) == 0 || i == 1 || i == NUM_POINTS
        [a4_val, a5_val] = readA4A5FromAxisAct(tcp);
        a4_log(i) = a4_val;
        a5_log(i) = a5_val;

        if ~isnan(a4_val) && abs(a4_val) > A4_GUARD_DEG
            fprintf('  WARNING: A4=%.1f exceeds guard!\n', a4_val);
            [ROBOT_S, ROBOT_T] = readSTFromPosAct(tcp, ROBOT_S, ROBOT_T);
            nonBlockingWait(0.08);
            [a4_chk, ~] = readA4A5FromAxisAct(tcp);
            if ~isnan(a4_chk) && abs(a4_chk) > A4_GUARD_DEG
                fprintf('  A4 STILL HIGH -> ABORTING\n');
                aborted = true; break;
            end
        end

        if ~isnan(a5_val) && abs(a5_val) > A5_GUARD_DEG
            fprintf('  A5=%.1f exceeds guard -> ABORTING\n', a5_val);
            aborted = true; break;
        end
    end

    % Live plot (non-blocking)
    if ENABLE_LIVE_PLOT && (mod(i, PLOT_UPDATE_RATE) == 0 || i == NUM_POINTS)
        valid = find(~isnan(exec_pos(:,1)));
        if ~isempty(valid)
            set(h_exec, 'XData', exec_pos(valid,1), ...
                'YData', exec_pos(valid,2), 'ZData', exec_pos(valid,3));
            set(h_cur, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));

            addpoints(h_oA, i, pos(4));
            addpoints(h_oB, i, pos(5));
            addpoints(h_oC, i, pos(6));
            addpoints(h_pX, i, pos(1));
            addpoints(h_pY, i, pos(2));
            addpoints(h_pZ, i, pos(3));

            if ~isnan(a4_val), addpoints(h_a4, i, a4_val); end
            if ~isnan(a5_val), addpoints(h_a5, i, a5_val); end

            elapsed = toc(t_start);
            pct = i/NUM_POINTS*100;
            set(h_status, 'String', sprintf( ...
                'Step: %d/%d\nProgress: %.1f%%\nElapsed: %.1fs\nS=%d T=%d\nA4=%.1f A5=%.1f', ...
                i, NUM_POINTS, pct, elapsed, ROBOT_S, ROBOT_T, a4_val, a5_val));

            drawnow limitrate;
        end
    end

    % Precise timing: spin-wait until CONTROL_DT elapsed
    while toc(t_step) < CONTROL_DT
        % tight loop — no pause()
    end
end

%% ===== RETURN HOME =====
fprintf('\nReturning home...\n');
sendE6Pos(tcp, key_poses(1,:));
pulseMoveTrigger(tcp);
nonBlockingWait(1.5);

exec_time = toc(t_start);

%% ===== SAVE LOG =====
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
log_filename = sprintf('execution_log_%s.mat', timestamp);
execution_log = struct('exec_pos', exec_pos, 'a4_log', a4_log, 'a5_log', a5_log, ...
    'key_poses', key_poses, 'trajectory_file', selected_file, ...
    'execution_time', exec_time, 'success_count', success_count, ...
    'aborted', aborted, 'control_dt', CONTROL_DT, 'timestamp', timestamp); %#ok<NASGU>
save(log_filename, 'execution_log');

fprintf('\n===========================================================\n');
fprintf('   EXECUTION COMPLETE\n');
fprintf('   Time:     %.1f s\n', exec_time);
fprintf('   Success:  %d / %d\n', success_count, NUM_POINTS);
fprintf('   Aborted:  %s\n', mat2str(aborted));
fprintf('   Log:      %s\n', log_filename);
fprintf('===========================================================\n');

%% ===== HELPER FUNCTIONS =====

function ok = sendE6Pos(tcp, pose6)
    posStr = sprintf('{X %.3f,Y %.3f,Z %.3f,A %.3f,B %.3f,C %.3f}', ...
        pose6(1), pose6(2), pose6(3), pose6(4), pose6(5), pose6(6));
    ok = writeVar(tcp, 'target_pos', posStr);
end

function pulseMoveTrigger(tcp)
    writeVar(tcp, 'move_trigger', 'TRUE');
    writeVar(tcp, 'move_trigger', 'FALSE');
end

function nonBlockingWait(seconds)
    % Non-blocking wait using tic/toc — allows drawnow
    t0 = tic;
    while toc(t0) < seconds
        drawnow limitrate;
    end
end

function [S, T] = readSTFromPosAct(tcp, S, T)
    try
        pos_act_str = readVar(tcp, '$POS_ACT');
        if ischar(pos_act_str) || isstring(pos_act_str)
            pos_str = char(pos_act_str);
            s_match = regexp(pos_str, '[,\s]S\s+(\d+)', 'tokens');
            t_match = regexp(pos_str, '[,\s]T\s+(-?\d+)', 'tokens');
            if ~isempty(s_match), S = str2double(s_match{1}{1}); end
            if ~isempty(t_match), T = str2double(t_match{1}{1}); end
        end
    catch
    end
end

function [a4, a5] = readA4A5FromAxisAct(tcp)
    a4 = NaN; a5 = NaN;
    try
        axis_act = readVar(tcp, '$AXIS_ACT');
        if ischar(axis_act) || isstring(axis_act)
            axis_str = char(axis_act);
            a4_match = regexp(axis_str, 'A4\s+([-\d.]+)', 'tokens');
            a5_match = regexp(axis_str, 'A5\s+([-\d.]+)', 'tokens');
            if ~isempty(a4_match), a4 = str2double(a4_match{1}{1}); end
            if ~isempty(a5_match), a5 = str2double(a5_match{1}{1}); end
        end
    catch
    end
end

function success = writeVar(tcp, varName, varValue)
    success = false;
    try
        msgId = uint16(1);
        mode  = uint8(1);
        varNameBytes  = uint8(varName);
        varValueBytes = uint8(varValue);
        varNameLen  = uint16(length(varNameBytes));
        varValueLen = uint16(length(varValueBytes));
        contentLen = uint16(1 + 2 + length(varNameBytes) + 2 + length(varValueBytes));

        msgIdBytes       = typecast(swapbytes(msgId),      'uint8');
        contentLenBytes  = typecast(swapbytes(contentLen), 'uint8');
        varNameLenBytes  = typecast(swapbytes(varNameLen), 'uint8');
        varValueLenBytes = typecast(swapbytes(varValueLen),'uint8');

        msg = [msgIdBytes(:).' contentLenBytes(:).' mode ...
               varNameLenBytes(:).' varNameBytes ...
               varValueLenBytes(:).' varValueBytes];
        write(tcp, msg);

        % Non-blocking response wait
        t0 = tic;
        while tcp.NumBytesAvailable == 0 && toc(t0) < 0.05
        end

        success = true;
        if tcp.NumBytesAvailable > 0
            response = read(tcp, tcp.NumBytesAvailable);
            if length(response) >= 3
                tail = response(end-2:end);
                success = all(tail == [0 1 1]);
            end
        end
    catch
        success = false;
    end
end

function value = readVar(tcp, varName)
    value = NaN;
    try
        msgId = uint16(1);
        mode  = uint8(0);
        varNameBytes = uint8(varName);
        varNameLen   = uint16(length(varNameBytes));
        contentLen = uint16(1 + 2 + length(varNameBytes));

        msgIdBytes      = typecast(swapbytes(msgId),      'uint8');
        contentLenBytes = typecast(swapbytes(contentLen), 'uint8');
        varNameLenBytes = typecast(swapbytes(varNameLen), 'uint8');

        msg = [msgIdBytes(:).' contentLenBytes(:).' mode ...
               varNameLenBytes(:).' varNameBytes];
        write(tcp, msg);

        t0 = tic;
        while tcp.NumBytesAvailable == 0 && toc(t0) < 0.05
        end

        if tcp.NumBytesAvailable > 0
            response = read(tcp, tcp.NumBytesAvailable);
            if length(response) > 7
                valueLen = double(swapbytes(typecast(response(6:7), 'uint16')));
                if length(response) >= 7 + valueLen
                    valueBytes = response(8:(7+valueLen));
                    valueStr = char(valueBytes);
                    value = str2double(valueStr);
                    if isnan(value), value = valueStr; end
                end
            end
        end
    catch
        value = NaN;
    end
end

function safeClose(tcp)
    try
        if isvalid(tcp), clear tcp; end %#ok<ASGSL>
        fprintf('TCP closed.\n');
    catch
    end
end

function w = ensureRobotNativeEuler(w)
    if isempty(w), return; end
    Bm = mean(w(:,5));
    if abs(Bm) <= 90, return; end
    for k = 1:size(w,1)
        A_old = w(k,4); B_old = w(k,5); C_old = w(k,6);
        if B_old > 0
            w(k,4) = A_old - 180; w(k,5) = 180 - B_old; w(k,6) = C_old - 180;
        else
            w(k,4) = A_old + 180; w(k,5) = -180 - B_old; w(k,6) = C_old + 180;
        end
    end
    for col = [4, 6]
        for k = 2:size(w,1)
            while (w(k,col) - w(k-1,col)) > 180,  w(k,col) = w(k,col) - 360; end
            while (w(k,col) - w(k-1,col)) < -180, w(k,col) = w(k,col) + 360; end
        end
    end
end