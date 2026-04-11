%% =========================================================================
%  runRoarmPaintingRL.m  —  RL Adaptive Painting Speed for RoArm-M2-S
% ==========================================================================
%
%  The RoArm-M2-S robots perform painting strokes on workpieces.
%  This RL agent learns the optimal SPEED PROFILE for each stroke to:
%    • Minimise tracking error (→ uniform paint coverage)
%    • Maximise throughput (→ faster cycle times)
%    • Ensure smooth motion (→ no paint splatter)
%    • Slow down at endpoints/dwells (→ clean transitions)
%
%  The learned speed profile maps to the RoArm JSON 'spd' parameter
%  in commands like: {"T":122,"b":..,"s":..,"e":..,"h":..,"spd":XX,"acc":10}
%
%  Requirements:
%    • MATLAB R2022a+, Reinforcement Learning Toolbox, Deep Learning Toolbox
%    • roarm_position_rl_dataset.csv in the same folder
%    • RoarmPaintingEnv.m in the same folder
%
%  Usage:   >> runRoarmPaintingRL
% ==========================================================================

% NOTE: This is intentionally a SCRIPT (not a function) so that
% main_rl_pipeline can wrap it inside a function-scope call.
% The 'clear' below only clears this scope when called from a function.
% If you run standalone, it clears the base workspace as expected.

close all; clc;
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  RoArm-M2-S — Adaptive Painting Speed RL                   ║\n');
fprintf('║  Idea 3: Learn optimal speed per waypoint for paint strokes ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% ── 0. RESOLVE DATASET ──────────────────────────────────────────────────
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end

csvPath = fullfile(scriptDir, 'roarm_position_rl_dataset.csv');
if ~isfile(csvPath)
    % Fallback: try parent RL folder
    csvPath = fullfile(scriptDir, '..', 'roarm_position_rl_dataset.csv');
end
assert(isfile(csvPath), 'Cannot find roarm_position_rl_dataset.csv at:\n  %s', csvPath);

%% ── 1. LOAD & PARSE DATASET ────────────────────────────────────────────
fprintf('[1/7] Loading dataset …\n');
data = readtable(csvPath, 'VariableNamingRule', 'preserve');
N    = height(data);
dt   = data.dt(1);
fprintf('       %d total timesteps, dt=%.2fs\n', N, dt);

% --- Identify unique arms ---
arms = unique(data.arm_id);
fprintf('       Arms found: %s\n', strjoin(arms, ', '));

% --- Identify episodes from run_id ---
episodes = unique(data.run_id);
fprintf('       Episodes found: %d\n', length(episodes));

% ─── For training, use roarm_1 data (primary painting arm) ───
armMask  = strcmp(data.arm_id, 'roarm_1');
armData  = data(armMask, :);
Na       = height(armData);
fprintf('       Using roarm_1: %d timesteps\n\n', Na);

%% ── 2. FEATURE ENGINEERING ──────────────────────────────────────────────
fprintf('[2/7] Engineering features …\n');

target_xyz = [armData.target_x, armData.target_y, armData.target_z];
meas_xyz   = [armData.meas_x,   armData.meas_y,   armData.meas_z];
time_s     = armData.time_s;

% --- Tracking error (mm) ---
tracking_error = vecnorm(target_xyz - meas_xyz, 2, 2);
fprintf('       Tracking error: mean=%.2f mm, max=%.2f mm\n', ...
    mean(tracking_error), max(tracking_error));

% --- End-effector velocity (mm/s) ---
ee_velocity = zeros(Na, 1);
for k = 2:Na
    ee_velocity(k) = norm(target_xyz(k,:) - target_xyz(k-1,:)) / dt;
end
ee_velocity(1) = ee_velocity(2);

% --- End-effector acceleration (mm/s²) ---
ee_accel = zeros(Na, 1);
for k = 2:Na
    ee_accel(k) = (ee_velocity(k) - ee_velocity(k-1)) / dt;
end
ee_accel(1) = ee_accel(2);

% --- Jerk (mm/s³) — key for paint uniformity ---
jerk = zeros(Na, 1);
for k = 2:Na
    jerk(k) = (ee_accel(k) - ee_accel(k-1)) / dt;
end
jerk(1) = jerk(2);

% --- Path curvature (rad) ---
curvature = zeros(Na, 1);
for k = 3:Na
    v1 = target_xyz(k-1,:) - target_xyz(k-2,:);
    v2 = target_xyz(k,:)   - target_xyz(k-1,:);
    n1 = norm(v1); n2 = norm(v2);
    if n1 > 1e-6 && n2 > 1e-6
        curvature(k) = acos(max(-1, min(1, dot(v1,v2)/(n1*n2))));
    end
end

% --- Progress along trajectory (0→1) ---
cum_path = zeros(Na, 1);
for k = 2:Na
    cum_path(k) = cum_path(k-1) + norm(target_xyz(k,:) - target_xyz(k-1,:));
end
progress = cum_path / max(cum_path(end), 1e-6);

% --- Dwell detection ---
is_dwell = false(Na, 1);
for k = 2:Na
    if norm(target_xyz(k,:) - target_xyz(k-1,:)) < 0.5
        is_dwell(k) = true;
    end
end
is_dwell(1) = (norm(target_xyz(1,:) - target_xyz(min(2,Na),:)) < 0.5);
fprintf('       Dwell steps detected: %d / %d (%.1f%%)\n', ...
    sum(is_dwell), Na, 100*sum(is_dwell)/Na);

% --- Stroke phase detection ---
stroke_direction = zeros(Na, 1);
for k = 2:Na
    dy = target_xyz(k,2) - target_xyz(k-1,2);
    if abs(dy) > 1
        stroke_direction(k) = sign(dy);
    end
end

% --- Pack into struct ---
F.tracking_error   = tracking_error;
F.ee_velocity      = ee_velocity;
F.ee_accel         = ee_accel;
F.jerk             = jerk;
F.curvature        = curvature;
F.progress         = progress;
F.is_dwell         = is_dwell;
F.stroke_direction = stroke_direction;
F.target_xyz       = target_xyz;
F.meas_xyz         = meas_xyz;
F.time_s           = time_s;
F.dt               = dt;

fprintf('       Vel max=%.1f mm/s, Accel max=%.1f mm/s², Jerk max=%.1f mm/s³\n', ...
    max(abs(ee_velocity)), max(abs(ee_accel)), max(abs(jerk)));
fprintf('       Feature engineering complete.\n\n');

%% ── 3. PLOT DATASET ─────────────────────────────────────────────────────
fprintf('[3/7] Plotting dataset …\n');

fig1 = figure('Name','RoArm Dataset Overview', ...
              'Position',[50 50 1400 800], 'Color','w');
tl = tiledlayout(3, 2, 'TileSpacing','compact', 'Padding','compact');
title(tl, 'RoArm-M2-S — Dataset Feature Overview', ...
      'FontSize',13, 'FontWeight','bold');

% (1) 3D trajectory
nexttile;
plot3(target_xyz(:,1), target_xyz(:,2), target_xyz(:,3), 'b-', 'LineWidth',1.5);
hold on;
plot3(meas_xyz(:,1), meas_xyz(:,2), meas_xyz(:,3), 'r--', 'LineWidth',0.8);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Target vs Measured Trajectory'); legend('Target','Measured');
grid on; view(30, 25);

% (2) Tracking error over time
nexttile;
plot(time_s, tracking_error, 'r-', 'LineWidth',1);
hold on;
plot(time_s, movmean(tracking_error, 10), 'k-', 'LineWidth',2);
xlabel('Time (s)'); ylabel('Error (mm)');
title(sprintf('Tracking Error (mean=%.2f mm)', mean(tracking_error)));
legend('Per-step','Smoothed'); grid on;

% (3) Velocity profile
nexttile;
plot(time_s, ee_velocity, 'b-', 'LineWidth',1);
xlabel('Time (s)'); ylabel('Velocity (mm/s)');
title('End-Effector Velocity'); grid on;

% (4) Curvature
nexttile;
plot(time_s, curvature, 'm-', 'LineWidth',1);
xlabel('Time (s)'); ylabel('Curvature (rad)');
title('Path Curvature'); grid on;

% (5) Dwell detection
nexttile;
stem(time_s, double(is_dwell), 'g', 'MarkerSize',3);
xlabel('Time (s)'); ylabel('Dwell');
title(sprintf('Dwell Steps (%d of %d)', sum(is_dwell), Na)); grid on;

% (6) Progress
nexttile;
plot(time_s, progress, 'k-', 'LineWidth',1.5);
xlabel('Time (s)'); ylabel('Progress (0→1)');
title('Trajectory Progress'); grid on;

% Now plot both-arm comparison
arm2Mask = strcmp(data.arm_id, 'roarm_2');
if any(arm2Mask)
    arm2Data = data(arm2Mask, :);
    target2  = [arm2Data.target_x, arm2Data.target_y, arm2Data.target_z];
    meas2    = [arm2Data.meas_x,   arm2Data.meas_y,   arm2Data.meas_z];

    fig1b = figure('Name','Both-Arm Comparison','Position',[100 100 1000 500],'Color','w');
    subplot(1,2,1);
    plot3(target_xyz(:,1), target_xyz(:,2), target_xyz(:,3), 'b-', 'LineWidth',1.5);
    hold on;
    plot3(target2(:,1), target2(:,2), target2(:,3), 'r-', 'LineWidth',1.5);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Both Arms — Target Paths'); legend('roarm\_1','roarm\_2');
    grid on; view(30,25);

    trackErr2 = vecnorm(target2 - meas2, 2, 2);
    subplot(1,2,2);
    histogram(tracking_error, 20, 'FaceColor','b', 'FaceAlpha',0.6, ...
              'DisplayName','roarm\_1');
    hold on;
    histogram(trackErr2, 20, 'FaceColor','r', 'FaceAlpha',0.6, ...
              'DisplayName','roarm\_2');
    xlabel('Tracking Error (mm)'); ylabel('Count');
    title('Error Distribution'); legend; grid on;
end
fprintf('       Both-arm comparison plotted.\n\n');

%% ── 4. BUILD TD3 AGENT ─────────────────────────────────────────────────
fprintf('[4/7] Building TD3 agent …\n');

try
    % ── Observation & action specs ────────────────────────────────────
    % Observation vector:
    %   [trackErr, velocity, accel, curvature, progress, isDwell,
    %    prevSpeed, errDelta]
    obsDim = 8;
    obsInfo = rlNumericSpec([obsDim 1], ...
        'LowerLimit', -ones(obsDim,1)*Inf, ...
        'UpperLimit',  ones(obsDim,1)*Inf);
    obsInfo.Name = 'RoArmObs';
    obsInfo.Description = 'trackErr,vel,accel,curv,progress,dwell,prevSpd,errDelta';

    % Action: normalised speed command [0, 1]
    actInfo = rlNumericSpec([1 1], 'LowerLimit', 0, 'UpperLimit', 1);
    actInfo.Name = 'SpeedCmd';
    actInfo.Description = 'Normalised painting speed [0=stop, 1=max]';

    % ── Build actor network ───────────────────────────────────────────
    actorNet = [
        featureInputLayer(obsDim, 'Normalization','none', 'Name','obs')
        fullyConnectedLayer(128, 'Name','fc1')
        reluLayer('Name','relu1')
        fullyConnectedLayer(128, 'Name','fc2')
        reluLayer('Name','relu2')
        fullyConnectedLayer(64,  'Name','fc3')
        reluLayer('Name','relu3')
        fullyConnectedLayer(1,   'Name','fc_out')
        sigmoidLayer('Name','sigmoid')      % output in [0, 1]
    ];
    actorNet = dlnetwork(actorNet);
    actor = rlContinuousDeterministicActor(actorNet, obsInfo, actInfo);

    % ── Build twin critic networks ────────────────────────────────────
    critic1 = buildCriticNet(obsDim, obsInfo, actInfo, 'critic1');
    critic2 = buildCriticNet(obsDim, obsInfo, actInfo, 'critic2');

    % ── Agent options ─────────────────────────────────────────────────
    agentOpts = rlTD3AgentOptions( ...
        'SampleTime',               dt, ...
        'DiscountFactor',           0.99, ...
        'ExperienceBufferLength',   1e5, ...
        'MiniBatchSize',            128, ...
        'NumWarmStartSteps',        500, ...
        'TargetSmoothFactor',       0.005);

    % Set learning rates
    agentOpts.ActorOptimizerOptions  = rlOptimizerOptions('LearnRate', 1e-4);
    agentOpts.CriticOptimizerOptions = rlOptimizerOptions('LearnRate', 3e-4);

    % Exploration noise
    agentOpts.ExplorationModel = rl.option.GaussianActionNoise( ...
        'StandardDeviation', 0.2, ...
        'StandardDeviationDecayRate', 1e-5, ...
        'StandardDeviationMin', 0.02);

    % ── Create TD3 agent ──────────────────────────────────────────────
    agent = rlTD3Agent(actor, [critic1, critic2], agentOpts);

    fprintf('       TD3 agent built successfully.\n');
    fprintf('       Actor:  %d parameters\n', countParams(actorNet));
    fprintf('       Obs dim: %d, Act dim: 1\n\n', obsDim);

catch ME
    fprintf('\n  ✗ Agent build FAILED:\n');
    fprintf('    %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('    Line %d in %s\n', ME.stack(1).line, ME.stack(1).name);
    end
    rethrow(ME);
end

%% ── 5. CREATE RL ENVIRONMENT ────────────────────────────────────────────
fprintf('[5/7] Creating RL environment …\n');

try
    env = RoarmPaintingEnv();
    fprintf('       Environment created: MaxSteps=%d, Ts=%.2f s\n', ...
        env.MaxSteps, env.Ts);
    validateEnvironment(env);
    fprintf('       Environment validated ✓\n\n');
catch ME
    fprintf('  ⚠ RoarmPaintingEnv not available or validation failed.\n');
    fprintf('    %s\n', ME.message);
    fprintf('  Creating a data-driven replay environment instead …\n');
    env = createDataReplayEnv(F, obsInfo, actInfo, dt);
    fprintf('       Data-replay environment created.\n\n');
end

%% ── 6. TRAIN ────────────────────────────────────────────────────────────
fprintf('[6/7] Training TD3 agent …\n');

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes',          1000, ...
    'MaxStepsPerEpisode',   Na, ...
    'ScoreAveragingWindowLength', 20, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue',    1e6, ...   % effectively no early stop
    'Verbose',              true, ...
    'Plots',                'training-progress');

trainStats = train(agent, env, trainOpts);

% Save agent
savedAgentsDir = fullfile(scriptDir, 'savedAgents');
if ~isfolder(savedAgentsDir), mkdir(savedAgentsDir); end
agentFile = fullfile(savedAgentsDir, 'roarm_td3_agent_improved.mat');
save(agentFile, 'agent', 'trainStats');
fprintf('       Agent saved to: %s\n\n', agentFile);

%% ── 7. EVALUATE & PLOT ─────────────────────────────────────────────────
fprintf('[7/7] Evaluating trained agent …\n');

% Run one evaluation episode
simOpts = rlSimulationOptions('MaxSteps', Na, 'NumSimulations', 1);
experience = sim(env, agent, simOpts);

% Extract results
rewards = squeeze(experience.Reward.Data);
cumReward = sum(rewards);
meanReward = mean(rewards);

fprintf('       Evaluation episode:\n');
fprintf('         Total reward:  %.4f\n', cumReward);
fprintf('         Mean reward:   %.4f\n', meanReward);
fprintf('         Steps:         %d\n', numel(rewards));

% Training progress plot
fig2 = figure('Name','Training Progress','Position',[100 100 900 400],'Color','w');
subplot(1,2,1);
plot(trainStats.EpisodeReward, 'Color',[0.7 0.7 0.7], 'LineWidth',0.5);
hold on;
plot(movmean(trainStats.EpisodeReward, 20), 'r-', 'LineWidth',2);
xlabel('Episode'); ylabel('Episode Reward');
title('Training Reward'); legend('Per-episode','Smoothed (20)'); grid on;

subplot(1,2,2);
plot(trainStats.EpisodeSteps, 'b-', 'LineWidth',0.5);
hold on;
plot(movmean(trainStats.EpisodeSteps, 20), 'k-', 'LineWidth',2);
xlabel('Episode'); ylabel('Steps');
title('Episode Length'); legend('Per-episode','Smoothed (20)'); grid on;

figuresDir = fullfile(scriptDir, 'figures');
if ~isfolder(figuresDir), mkdir(figuresDir); end
exportgraphics(fig2, fullfile(figuresDir, 'roarm_training_progress.png'), 'Resolution',150);

% Per-step reward from evaluation
fig3 = figure('Name','Evaluation Rewards','Position',[150 150 700 350],'Color','w');
plot(rewards, 'Color',[0.7 0.7 0.7], 'LineWidth',0.5);
hold on;
plot(movmean(rewards, 15), 'r-', 'LineWidth',2);
xlabel('Step'); ylabel('Reward');
title(sprintf('Evaluation Per-Step Reward (mean=%.4f)', meanReward));
legend('Per-step','Smoothed'); grid on;
exportgraphics(fig3, fullfile(figuresDir, 'roarm_eval_rewards.png'), 'Resolution',150);

fprintf('\n  ✓ RoArm RL pipeline complete.\n');
fprintf('    Figures saved to: %s\n', figuresDir);

%% ═══════════════════════════════════════════════════════════════════════
%  LOCAL HELPER FUNCTIONS
% ════════════════════════════════════════════════════════════════════════

function critic = buildCriticNet(obsDim, obsInfo, actInfo, netName)
    % Observation path
    obsPath = [
        featureInputLayer(obsDim, 'Normalization','none', 'Name',[netName '_obs'])
        fullyConnectedLayer(128, 'Name',[netName '_obs_fc1'])
        reluLayer('Name',[netName '_obs_relu1'])
    ];
    % Action path
    actPath = [
        featureInputLayer(1, 'Normalization','none', 'Name',[netName '_act'])
        fullyConnectedLayer(128, 'Name',[netName '_act_fc1'])
        reluLayer('Name',[netName '_act_relu1'])
    ];
    % Common path
    commonPath = [
        additionLayer(2, 'Name',[netName '_add'])
        fullyConnectedLayer(128, 'Name',[netName '_fc2'])
        reluLayer('Name',[netName '_relu2'])
        fullyConnectedLayer(64, 'Name',[netName '_fc3'])
        reluLayer('Name',[netName '_relu3'])
        fullyConnectedLayer(1, 'Name',[netName '_qvalue'])
    ];

    criticNet = layerGraph();
    criticNet = addLayers(criticNet, obsPath);
    criticNet = addLayers(criticNet, actPath);
    criticNet = addLayers(criticNet, commonPath);
    criticNet = connectLayers(criticNet, [netName '_obs_relu1'], [netName '_add/in1']);
    criticNet = connectLayers(criticNet, [netName '_act_relu1'], [netName '_add/in2']);
    criticNet = dlnetwork(criticNet);

    critic = rlQValueFunction(criticNet, obsInfo, actInfo, ...
        'ObservationInputNames', {[netName '_obs']}, ...
        'ActionInputNames', {[netName '_act']});
end

function n = countParams(net)
    n = 0;
    for i = 1:numel(net.Learnables.Value)
        n = n + numel(net.Learnables.Value{i});
    end
end

function env = createDataReplayEnv(F, obsInfo, actInfo, dt)
    % Fallback: create a simple function-handle-based environment
    % that replays the dataset for training when RoarmPaintingEnv.m
    % is not available or fails validation.

    Na = numel(F.tracking_error);

    % Normalise features for observation
    velNorm   = F.ee_velocity / max(abs(F.ee_velocity) + 1e-6);
    accelNorm = F.ee_accel / max(abs(F.ee_accel) + 1e-6);
    curvNorm  = F.curvature / (pi + 1e-6);
    errNorm   = F.tracking_error / max(F.tracking_error + 1e-6);

    stepIdx = 1;
    prevSpeed = 0.5;
    prevErr   = 0;

    resetFcn = @() resetEnv();
    stepFcn  = @(action, loggedSignals) stepEnv(action, loggedSignals);

    env = rlFunctionEnv(obsInfo, actInfo, stepFcn, resetFcn);

    function [obs, loggedSignals] = resetEnv()
        stepIdx = 1;
        prevSpeed = 0.5;
        prevErr = 0;
        obs = getObs(stepIdx, prevSpeed, prevErr);
        loggedSignals.stepIdx = stepIdx;
    end

    function [obs, reward, isDone, loggedSignals] = stepEnv(action, loggedSignals)
        stepIdx = stepIdx + 1;
        if stepIdx > Na
            stepIdx = Na;
        end

        spdCmd = max(0, min(1, action));

        % Compute reward
        err = F.tracking_error(stepIdx);
        errThresh = 1.5;
        errN = err / errThresh;

        r_track  = -2.0 * errN^2;
        r_speed  =  0.3 * spdCmd * max(0, 1 - errN);
        cN = min(1, F.curvature(stepIdx)/pi);
        idealSpd = 1.0*(1-cN) + 0.1*cN;
        r_curv   = -0.4 * (spdCmd - idealSpd)^2;
        r_smooth = -0.5 * abs(spdCmd - prevSpeed);
        r_dwell  = 0;
        if F.is_dwell(stepIdx) && spdCmd < 0.2
            r_dwell = 0.3;
        end

        reward = r_track + r_speed + r_curv + r_smooth + r_dwell;
        isDone = (stepIdx >= Na);

        prevSpeed = spdCmd;
        prevErr   = err;
        obs = getObs(stepIdx, prevSpeed, prevErr);
        loggedSignals.stepIdx = stepIdx;
    end

    function obs = getObs(idx, pSpd, pErr)
        obs = [
            errNorm(idx);
            velNorm(idx);
            accelNorm(idx);
            curvNorm(idx);
            F.progress(idx);
            double(F.is_dwell(idx));
            pSpd;
            errNorm(idx) - (pErr / max(F.tracking_error + 1e-6))
        ];
    end
end