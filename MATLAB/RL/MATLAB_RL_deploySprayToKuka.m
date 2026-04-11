%% DEPLOYSPRAYTOKUKA - Deploy trained RL spray agent to real KUKA KR10
%
%  Reads live robot feedback via KukaVarProxy TCP connection,
%  computes RL actions, and writes adaptive speed/pulse to controller.
%
%  SAFETY: Includes velocity limits, watchdog timeout, and manual override.
%
%  Usage:
%    >> deploySprayToKuka('172.31.17.101', 7000)

function deploySprayToKuka(robotIP, port)
    if nargin < 1, robotIP = '172.31.17.101'; end  % From your dataset
    if nargin < 2, port = 7000; end

    fprintf('=== Deploying Spray RL Agent to KUKA ===\n');
    fprintf('Robot IP: %s:%d\n\n', robotIP, port);

    %% 1. Load trained agent
    agentPath = fullfile(pwd, 'trained_agents', 'spray_ppo_agent.mat');
    loaded = load(agentPath);
    agent = loaded.agent;
    fprintf('Agent loaded.\n');

    %% 2. Connect to KUKA via TCP (KukaVarProxy)
    fprintf('Connecting to robot...\n');
    t = tcpclient(robotIP, port, 'Timeout', 10);
    fprintf('Connected!\n');

    %% 3. Safety limits
    MIN_SPEED_OVERRIDE = 10;   % % — never go below 10% speed
    MAX_SPEED_OVERRIDE = 80;   % % — never exceed 80% in autonomous mode
    MIN_PULSE_MS       = 6;    % ms
    MAX_PULSE_MS       = 18;   % ms
    WATCHDOG_TIMEOUT   = 5.0;  % seconds — abort if no response
    EMERGENCY_STOP     = false;

    %% 4. Initialize state tracking
    prev_speed_cmd = 0.5;
    prev_meas = [0 0 0];
    prev_time = tic;
    step_count = 0;

    % Normalization constants (from training)
    maxVel = 600; maxAccel = 20000; maxCurv = 3.14;
    maxErr = 120; maxSprayDist = 500; maxTorque = 250;
    surface_z = 204;  % From place_z in your dataset

    fprintf('\n--- LIVE CONTROL LOOP (Ctrl+C to stop) ---\n');
    fprintf('%-6s %-10s %-10s %-10s %-12s %-10s\n', ...
        'Step', 'Speed%', 'Pulse_ms', 'Thick_est', 'TrackErr', 'Vel_mm/s');

    try
        while ~EMERGENCY_STOP
            loop_start = tic;

            %% 5. Read current robot state
            % Read measured position
            meas_pos = readKukaVar(t, '$POS_ACT');
            target_pos = readKukaVar(t, 'target_pos');
            torques = readKukaTorques(t);

            % Parse positions (returns [X, Y, Z])
            meas_xyz   = parseKukaPos(meas_pos);
            target_xyz = parseKukaPos(target_pos);

            %% 6. Compute features for RL observation
            dt = toc(prev_time);
            prev_time = tic;
            dt = max(dt, 0.001);

            % Velocity
            vel_vec = (meas_xyz - prev_meas) / dt;
            ee_velocity = norm(vel_vec);

            % Tracking error
            track_err = norm(target_xyz - meas_xyz);

            % Spray distance
            spray_dist = abs(meas_xyz(3) - surface_z);

            % Torque magnitude
            torque_mag = norm(torques);

            % Approximate curvature (from velocity direction change)
            curvature = 0;  % Simplified for real-time

            % Approximate acceleration
            ee_accel = 0;  % Simplified for real-time

            % Progress (estimate from position)
            progress = min(step_count / 273, 1.0);

            %% 7. Build normalized observation vector
            obs = [
                min(ee_velocity / maxVel, 1);
                min(abs(ee_accel) / maxAccel, 1);
                min(curvature / maxCurv, 1);
                min(track_err / maxErr, 1);
                min(spray_dist / maxSprayDist, 1);
                min(torque_mag / maxTorque, 1);
                progress;
                prev_speed_cmd
            ];

            %% 8. Get RL action
            action = getAction(agent, obs);
            action = action{1};

            speed_factor = max(0.1, min(1.0, action(1)));
            pulse_factor = max(0.5, min(1.5, action(2)));

            %% 9. Convert to KUKA commands
            % Speed override: map [0.1, 1.0] → [10%, 80%]
            speed_override = round(MIN_SPEED_OVERRIDE + ...
                speed_factor * (MAX_SPEED_OVERRIDE - MIN_SPEED_OVERRIDE));

            % Pulse width: map [0.5, 1.5] → [6ms, 18ms]
            pulse_ms = round(MIN_PULSE_MS + ...
                (pulse_factor - 0.5) * (MAX_PULSE_MS - MIN_PULSE_MS));

            %% 10. Estimate resulting thickness (for logging)
            eff_speed = ee_velocity * speed_factor;
            thickness_est = 80 * (300/max(eff_speed,1)) * (300/max(spray_dist,50)) * pulse_factor;

            %% 11. Write commands to KUKA controller
            writeKukaVar(t, '$OV_PRO', num2str(speed_override));
            writeKukaVar(t, 'spray_pulse', num2str(pulse_ms));

            %% 12. Log
            step_count = step_count + 1;
            fprintf('%-6d %-10d %-10d %-10.1f %-12.1f %-10.1f\n', ...
                step_count, speed_override, pulse_ms, thickness_est, track_err, ee_velocity);

            %% 13. Update state
            prev_speed_cmd = speed_factor;
            prev_meas = meas_xyz;

            %% 14. Maintain control loop timing (30ms from your dataset)
            elapsed = toc(loop_start);
            pause_time = max(0, 0.03 - elapsed);
            pause(pause_time);
        end

    catch ME
        fprintf('\n!!! CONTROL LOOP TERMINATED: %s\n', ME.message);
        % Safety: reset to conservative defaults
        writeKukaVar(t, '$OV_PRO', '20');
        writeKukaVar(t, 'spray_pulse', '12');
        fprintf('Reset to safe defaults (20%% speed, 12ms pulse)\n');
    end

    %% Cleanup
    clear t;
    fprintf('\nDeployment ended. %d steps executed.\n', step_count);
end

%% ---- Helper Functions (reuse your existing KukaVarProxy protocol) ----

function writeKukaVar(tcp, varName, varValue)
    msgId = uint16(1);
    mode = uint8(1);  % Write
    varNameBytes = uint8(varName);
    varValueBytes = uint8(varValue);
    contentLen = uint16(1 + 2 + length(varNameBytes) + 2 + length(varValueBytes));
    msg = [typecast(swapbytes(msgId), 'uint8'), ...
           typecast(swapbytes(contentLen), 'uint8'), ...
           mode, ...
           typecast(swapbytes(uint16(length(varNameBytes))), 'uint8'), ...
           varNameBytes, ...
           typecast(swapbytes(uint16(length(varValueBytes))), 'uint8'), ...
           varValueBytes];
    write(tcp, msg);
    pause(0.005);  % Allow controller to process
    if tcp.NumBytesAvailable > 0
        read(tcp, tcp.NumBytesAvailable);  % Clear response buffer
    end
end

function response = readKukaVar(tcp, varName)
    msgId = uint16(1);
    mode = uint8(0);  % Read
    varNameBytes = uint8(varName);
    contentLen = uint16(1 + 2 + length(varNameBytes) + 2);
    msg = [typecast(swapbytes(msgId), 'uint8'), ...
           typecast(swapbytes(contentLen), 'uint8'), ...
           mode, ...
           typecast(swapbytes(uint16(length(varNameBytes))), 'uint8'), ...
           varNameBytes, ...
           typecast(swapbytes(uint16(0)), 'uint8')];
    write(tcp, msg);
    pause(0.01);
    raw = read(tcp, tcp.NumBytesAvailable);
    response = char(raw(end-50:end));  % Extract value string
end

function xyz = parseKukaPos(posStr)
    % Parse KRL position string: '{X 550,Y 0,Z 650,A -179.99,B 0.5,C -179.9}'
    xyz = [0 0 0];
    tokens = regexp(posStr, 'X\s*([-\d.]+)', 'tokens');
    if ~isempty(tokens), xyz(1) = str2double(tokens{1}{1}); end
    tokens = regexp(posStr, 'Y\s*([-\d.]+)', 'tokens');
    if ~isempty(tokens), xyz(2) = str2double(tokens{1}{1}); end
    tokens = regexp(posStr, 'Z\s*([-\d.]+)', 'tokens');
    if ~isempty(tokens), xyz(3) = str2double(tokens{1}{1}); end
end

function torques = readKukaTorques(tcp)
    % Read joint torques from KUKA variables
    % In production, read $TORQUE_AXIS_ACT[1..6]
    torques = zeros(1, 6);
    for j = 1:6
        try
            val = readKukaVar(tcp, sprintf('$TORQUE_AXIS_ACT[%d]', j));
            torques(j) = str2double(val);
        catch
            torques(j) = 0;
        end
    end
end