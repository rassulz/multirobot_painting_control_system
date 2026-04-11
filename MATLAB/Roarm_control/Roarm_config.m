% ---------------------------------------------------------
% ROARM M2 - REAL-TIME PLOTTING & CONTROL
% ---------------------------------------------------------
clear all; clc; close all;

% --- CONFIGURATION ---
ip = '192.168.4.1';
run_duration = 50;    % Total runtime
move_time = 3.0;      % Duration allowed for each movement
poll_interval = 0.15; % Time between data queries (don't go too fast for HTTP)

% --- SETUP LIVE GRAPHS ---
figure('Name', 'RoArm M2 Telemetry', 'Color', 'w', 'Position', [100 100 1000 600]);

% Position Plot
subplot(3,1,1);
hPosX = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'X');
hPosY = animatedline('Color', 'g', 'LineWidth', 1.5, 'DisplayName', 'Y');
hPosZ = animatedline('Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'Z');
legend; title('Position (mm)'); grid on; ylabel('Distance');

% Velocity Plot
subplot(3,1,2);
hVel = animatedline('Color', 'k', 'LineWidth', 1.5);
title('Calculated Velocity (mm/s)'); grid on; ylabel('Speed');

% Torque/Load Plot
subplot(3,1,3);
hLoad = animatedline('Color', 'm', 'LineWidth', 1.5);
title('Servo Load / Torque (Approx)'); grid on; ylabel('Load (0-1000)'); xlabel('Time (s)');

% --- MOVEMENT WAYPOINTS ---
pick_pos  = struct('T', 104, 'x', 300, 'y', 0, 'z', 100, 'spd', 0.5);
place_pos = struct('T', 104, 'x', 180, 'y', 100, 'z', 180, 'spd', 0.5);

% --- INITIALIZATION ---
disp('Starting Real-Time Plotter...');
start_tic = tic;      % Global timer
last_pos = [0,0,0];   % For velocity calc
last_time = 0;        % For velocity calc
current_state = 0;    % 0=Going to Pick, 1=Going to Place

% --- MAIN LOOP ---
while toc(start_tic) < run_duration
    
    % 1. DECIDE WHERE TO GO
    if current_state == 0
        target_name = 'PICK';
        target_cmd = pick_pos;
    else
        target_name = 'PLACE';
        target_cmd = place_pos;
    end
    
    % 2. SEND MOVEMENT COMMAND
    fprintf('[%.1f s] Commanding %s...\n', toc(start_tic), target_name);
    send_json_cmd(ip, target_cmd);
    
    % 3. MONITORING LOOP (Replaces simple pause)
    % We stay in this loop for 'move_time' seconds, querying data continuously
    move_start = tic;
    while toc(move_start) < move_time
        
        % A. Query Status (Command T:105 asks for current info)
        status_json = get_robot_status(ip);
        
        % B. Update Graphs if data is valid
        if ~isempty(status_json)
            t_now = toc(start_tic);
            
            % Extract Position (Check if field exists, default to 0)
            x = safe_get(status_json, 'x');
            y = safe_get(status_json, 'y');
            z = safe_get(status_json, 'z');
            
            % Extract Load (Some firmware uses 'l', 'load', or 'torq')
            % We sum loads of base/shoulder joints for a rough aggregate
            load_val = safe_get(status_json, 'lb') + safe_get(status_json, 'le'); 
            
            % Calculate Velocity (Distance / Time)
            curr_pos = [x, y, z];
            dt = t_now - last_time;
            if dt > 0
                dist = norm(curr_pos - last_pos);
                vel = dist / dt;
            else
                vel = 0;
            end
            
            % Update Plots
            addpoints(hPosX, t_now, x);
            addpoints(hPosY, t_now, y);
            addpoints(hPosZ, t_now, z);
            addpoints(hVel, t_now, vel);
            addpoints(hLoad, t_now, load_val);
            drawnow limitrate;
            
            % Store history
            last_pos = curr_pos;
            last_time = t_now;
        end
        
        % Wait a tiny bit to not choke the HTTP connection
        pause(poll_interval);
    end
    
    % Toggle State
    current_state = ~current_state;
end

disp('Cycle Complete.');

% ---------------------------------------------------------
% HELPER: SEND COMMAND (Blind)
% ---------------------------------------------------------
function send_json_cmd(ip, cmdStruct)
    try
        url = ['http://', ip, '/js?json=', jsonencode(cmdStruct)];
        webread(url);
    catch
        % Ignore timeouts on send
    end
end

% ---------------------------------------------------------
% HELPER: GET STATUS (Read)
% ---------------------------------------------------------
function data = get_robot_status(ip)
    data = [];
    % T:105 usually returns current coords/angles
    cmd = struct('T', 105); 
    url = ['http://', ip, '/js?json=', jsonencode(cmd)];
    
    try
        options = weboptions('Timeout', 1); % Fast timeout
        resp = webread(url, options);
        
        % Clean response (sometimes firmware returns raw text before JSON)
        % We try to find the first '{' and last '}'
        start_idx = strfind(resp, '{');
        end_idx = strfind(resp, '}');
        
        if ~isempty(start_idx) && ~isempty(end_idx)
            json_str = resp(start_idx(1):end_idx(end));
            data = jsondecode(json_str);
        else
            % Attempt direct decode
            data = jsondecode(resp);
        end
    catch
        % disp('.'); % Dot indicates dropped packet (normal on UDP/HTTP loop)
    end
end

% ---------------------------------------------------------
% HELPER: SAFE FIELD GETTER
% ---------------------------------------------------------
function val = safe_get(structData, fieldName)
    if isfield(structData, fieldName)
        val = double(structData.(fieldName));
    else
        val = 0;
    end
end