%% =========================================================================
%  runSprayRL.m  –  Master script for Adaptive Spray Parameter RL
% ==========================================================================
%  This single file contains EVERYTHING needed to:
%    1. Load & visualise the kuka_rl_dataset.csv
%    2. Engineer features
%    3. Build the RL environment (inline classdef via temp-file trick)
%    4. Create a TD3 (or PPO) agent
%    5. Train the agent
%    6. Evaluate the trained agent vs a fixed-parameter baseline
%    7. Export results to CSV + figures
%
%  Requirements:
%    • MATLAB R2022a or later  (for inline rlQValueFunction / rlTD3Agent)
%    • Reinforcement Learning Toolbox
%    • Deep Learning Toolbox
%    • kuka_rl_dataset.csv in the SAME folder as this script
%
%  Usage:
%    1.  Place this file next to kuka_rl_dataset.csv
%    2.  Open MATLAB → cd to that folder
%    3.  >> runSprayRL
%
%  Author : COLLINS MASIMBA
%  Date   : 2026-03-28
% ==========================================================================
clear; clc; close all;
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   KUKA KR10 R1100 — Adaptive Spray RL Pipeline         ║\n');
fprintf('╚══════��═══════════════════════════════════════════════════╝\n\n');

%% ── 0. RESOLVE DATASET PATH ─────────────────────────────────────────────
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end
csvPath = fullfile(scriptDir, 'kuka_rl_dataset.csv');
if ~isfile(csvPath)
    % fallback: look in the hard-coded location from the user's machine
    csvPath = fullfile('C:','Users','SYNAPTICON','Desktop', ...
        'multirobot_painting_control_system','MATLAB','RL', ...
        'kuka_rl_dataset.csv');
end
assert(isfile(csvPath), 'Cannot find kuka_rl_dataset.csv at:\n  %s', csvPath);

%% ── 1. LOAD & ENGINEER FEATURES ─────────────────────────────────────────
fprintf('[1/6] Loading dataset …\n');
data = readtable(csvPath, 'VariableNamingRule', 'preserve');
N    = height(data);
fprintf('       %d timesteps loaded from\n       %s\n', N, csvPath);

% Core signals
target_xyz = [data.target_x, data.target_y, data.target_z];
meas_xyz   = [data.meas_x,   data.meas_y,   data.meas_z];
time_s     = data.time_s;
dt_val     = data.dt(1);                                         % 0.03 s

% 1a. Tracking error
tracking_error = vecnorm(target_xyz - meas_xyz, 2, 2);

% 1b. End-effector velocity (mm/s)
ee_velocity    = zeros(N,1);
for k = 2:N
    ee_velocity(k) = norm(target_xyz(k,:) - target_xyz(k-1,:)) / dt_val;
end
ee_velocity(1) = ee_velocity(2);

% 1c. End-effector acceleration (mm/s²)
ee_accel       = zeros(N,1);
for k = 2:N
    ee_accel(k) = (ee_velocity(k) - ee_velocity(k-1)) / dt_val;
end
ee_accel(1) = ee_accel(2);

% 1d. Path curvature (rad)
curvature = zeros(N,1);
for k = 3:N
    v1 = target_xyz(k-1,:) - target_xyz(k-2,:);
    v2 = target_xyz(k,:)   - target_xyz(k-1,:);
    n1 = norm(v1); n2 = norm(v2);
    if n1 > 1e-6 && n2 > 1e-6
        c = dot(v1,v2)/(n1*n2);
        curvature(k) = acos(max(-1, min(1, c)));
    end
end

% 1e. Joint aggregates
torques   = [data.torque_j1,  data.torque_j2,  data.torque_j3, ...
             data.torque_j4,  data.torque_j5,  data.torque_j6];
torque_mag = vecnorm(torques, 2, 2);

vels      = [data.velocity_j1, data.velocity_j2, data.velocity_j3, ...
             data.velocity_j4, data.velocity_j5, data.velocity_j6];
vel_mag   = vecnorm(vels, 2, 2);

currs     = [data.current_j1, data.current_j2, data.current_j3, ...
             data.current_j4, data.current_j5, data.current_j6];
curr_mag  = vecnorm(currs, 2, 2);

% 1f. Progress (0→1)
cum_path = zeros(N,1);
for k = 2:N
    cum_path(k) = cum_path(k-1) + norm(target_xyz(k,:)-target_xyz(k-1,:));
end
progress = cum_path / max(cum_path(end), 1e-6);

% 1g. Spray distance (mm above surface; surface ≈ place_z = 204 mm)
surface_z      = data.place_z(1);                                % 204 mm
spray_distance = abs(target_xyz(:,3) - surface_z);

% 1h. Simulated baseline thickness (no RL, fixed speed factor = 1)
baseline_thickness = 80 .* (300 ./ max(ee_velocity,1)) ...
                        .* (300 ./ max(spray_distance,50));
baseline_thickness = min(baseline_thickness, 500);

% Pack into struct for convenience
F.tracking_error   = tracking_error;
F.ee_velocity      = ee_velocity;
F.ee_accel         = ee_accel;
F.curvature        = curvature;
F.torque_mag       = torque_mag;
F.vel_mag          = vel_mag;
F.curr_mag         = curr_mag;
F.progress         = progress;
F.spray_distance   = spray_distance;
F.target_xyz       = target_xyz;
F.meas_xyz         = meas_xyz;
F.time_s           = time_s;

% Normalisation ceilings (computed once from the dataset)
NORM.vel      = max(ee_velocity);
NORM.accel    = max(abs(ee_accel));
NORM.curv     = max(max(curvature), 1e-6);
NORM.trkErr   = max(tracking_error);
NORM.sprayD   = max(spray_distance);
NORM.torque   = max(torque_mag);

fprintf('       Feature engineering done — 11 features.\n\n');

%% ── 2. VISUALISE THE RAW DATASET ────────────────────────────────────────
fprintf('[2/6] Plotting raw dataset overview …\n');
fig1 = figure('Name','Dataset Overview','Position',[60 60 1300 820]);

subplot(2,3,1);
plot3(target_xyz(:,1), target_xyz(:,2), target_xyz(:,3), 'b-', 'LineWidth',1.4); hold on;
plot3(meas_xyz(:,1),   meas_xyz(:,2),   meas_xyz(:,3),  'r--');
legend('Target','Measured','Location','best');
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3-D Trajectory'); grid on;

subplot(2,3,2);
plot(time_s, ee_velocity, 'b', 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('mm/s'); title('EE Velocity'); grid on;

subplot(2,3,3);
plot(time_s, tracking_error, 'r', 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('mm');   title('Tracking Error'); grid on;

subplot(2,3,4);
plot(time_s, spray_distance, 'Color',[0 .55 .25], 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('mm');   title('Spray Distance'); grid on;

subplot(2,3,5);
plot(time_s, baseline_thickness, 'm', 'LineWidth',1.2); hold on;
yline(50,'g--','Min ideal'); yline(100,'g--','Max ideal');
yline(150,'r:','Drip');
xlabel('Time (s)'); ylabel('µm');   title('Baseline Thickness (no RL)'); grid on;

subplot(2,3,6);
plot(time_s, torque_mag, 'Color',[.8 .35 0], 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Nm (agg)'); title('Joint Torque'); grid on;

sgtitle('KUKA KR10 R1100 — Raw Dataset','FontSize',14,'FontWeight','bold');
drawnow;

%% ── 3. WRITE THE RL ENVIRONMENT CLASS TO A TEMP FILE ────────────────────
%  MATLAB requires classdef files on disk.  We auto-create it here so the
%  user only needs to run this ONE script.
fprintf('[3/6] Creating RL environment class …\n');

envFile = fullfile(scriptDir, 'SprayEnv.m');
fid = fopen(envFile, 'w');
fprintf(fid, '%s\n', [...
"classdef SprayEnv < rl.env.MATLABEnvironment"
"% SprayEnv  Custom RL environment for adaptive spray-parameter control."
"% "
"%   State  (7×1, normalised 0–1):"
"%     [ee_velocity, |ee_accel|, curvature, spray_distance,"
"%      torque_mag, tracking_error, progress]"
"% "
"%   Action (2×1, continuous):"
"%     speed_factor   ∈ [0.1 , 1.0]   — scales feed-rate override"
"%     pulse_factor   ∈ [0.5 , 1.5]   — scales spray-pulse duration"
""
"    properties"
"        F              % features struct"
"        NM             % normalisation struct"
"        NumSteps"
"        StepIdx"
"        PrevSpeed"
"        ThickLog"
"    end"
""
"    methods"
"        function this = SprayEnv(features, normStruct)"
"            obsInfo = rlNumericSpec([7 1],'LowerLimit',zeros(7,1),'UpperLimit',ones(7,1));"
"            obsInfo.Name = 'state';"
"            actInfo = rlNumericSpec([2 1],'LowerLimit',[0.1;0.5],'UpperLimit',[1.0;1.5]);"
"            actInfo.Name = 'action';"
"            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);"
"            this.F        = features;"
"            this.NM       = normStruct;"
"            this.NumSteps = length(features.ee_velocity);"
"        end"
""
"        function [obs,reward,isDone,info] = step(this,action)"
"            sf = max(0.1, min(1.0, action(1)));"
"            pf = max(0.5, min(1.5, action(2)));"
"            i  = this.StepIdx;"
""
"            vel  = this.F.ee_velocity(i);"
"            sd   = this.F.spray_distance(i);"
"            te   = this.F.tracking_error(i);"
""
"            effSpd = vel * sf;"
"            thick  = 80*(300/max(effSpd,1))*(300/max(sd,50))*pf;"
"            thick  = min(thick, 500);"
"            this.ThickLog(i) = thick;"
""
"            reward = 0;"
"            if thick>=50 && thick<=100"
"                reward = reward + 10;"
"            elseif thick>100 && thick<=150"
"                reward = reward + 5*(1-(thick-100)/50);"
"            elseif thick>150"
"                reward = reward - 20;"
"            elseif thick>=30 && thick<50"
"                reward = reward + 5*(1-(50-thick)/20);"
"            else"
"                reward = reward - 15;"
"            end"
""
"            reward = reward - 5*abs(sf - this.PrevSpeed);"
"            if te > 20"
"                reward = reward - 2*(te/max(this.NM.trkErr,1e-6));"
"            end"
"            cm = this.F.curr_mag(i);"
"            reward = reward + 1*(1-min(cm/20,1));"
""
"            this.PrevSpeed = sf;"
"            this.StepIdx   = this.StepIdx + 1;"
"            isDone = (this.StepIdx >= this.NumSteps);"
""
"            if ~isDone"
"                obs = getObs(this);"
"            else"
"                obs = zeros(7,1);"
"                avg = mean(this.ThickLog(this.ThickLog>0));"
"                if avg>=50 && avg<=100, reward = reward+50; end"
"            end"
"            info.thickness    = thick;"
"            info.speed_factor = sf;"
"            info.pulse_factor = pf;"
"        end"
""
"        function obs = reset(this)"
"            this.StepIdx   = 1;"
"            this.PrevSpeed = 0.5;"
"            this.ThickLog  = zeros(this.NumSteps,1);"
"            obs = getObs(this);"
"        end"
""
"        function obs = getObs(this)"
"            i = this.StepIdx;"
"            obs = ["
"                this.F.ee_velocity(i)      / max(this.NM.vel,1e-6);"
"                abs(this.F.ee_accel(i))    / max(this.NM.accel,1e-6);"
"                this.F.curvature(i)        / max(this.NM.curv,1e-6);"
"                this.F.spray_distance(i)   / max(this.NM.sprayD,1e-6);"
"                this.F.torque_mag(i)       / max(this.NM.torque,1e-6);"
"                this.F.tracking_error(i)   / max(this.NM.trkErr,1e-6);"
"                this.F.progress(i)"
"            ];"
"            obs = max(0, min(1, obs));"
"        end"
"    end"
"end"
]);
fclose(fid);
fprintf('       SprayEnv.m written to %s\n', envFile);
rehash;                       % make MATLAB see the new file immediately
fprintf('       Environment class ready.\n\n');

%% ── 4. BUILD THE RL AGENT ──────────────────────────────��────────────────
fprintf('[4/6] Building TD3 agent …\n');

env     = SprayEnv(F, NORM);
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);
obsDim  = obsInfo.Dimension(1);   % 7
actDim  = actInfo.Dimension(1);   % 2

% ----- Critic (twin Q-networks) -----
%  Two identical networks; TD3 uses the minimum of the pair.
buildCritic = @() createCriticNet(obsDim, actDim);

critic1 = rlQValueFunction(buildCritic(), obsInfo, actInfo, ...
    'ObservationInputNames','stateIn','ActionInputNames','actionIn');
critic2 = rlQValueFunction(buildCritic(), obsInfo, actInfo, ...
    'ObservationInputNames','stateIn','ActionInputNames','actionIn');

% ----- Actor -----
actorNet = dlnetwork([ ...
    featureInputLayer(obsDim, 'Normalization','none', 'Name','stateIn')
    fullyConnectedLayer(256, 'Name','afc1')
    reluLayer('Name','ar1')
    fullyConnectedLayer(128, 'Name','afc2')
    reluLayer('Name','ar2')
    fullyConnectedLayer(actDim, 'Name','afc3')
    sigmoidLayer('Name','sig')         % output ∈ (0,1)
    scalingLayer('Name','scale', ...   % map to action bounds
        'Scale', actInfo.UpperLimit - actInfo.LowerLimit, ...
        'Bias',  actInfo.LowerLimit)
]);
actor = rlContinuousDeterministicActor(actorNet, obsInfo, actInfo, ...
    'ObservationInputNames','stateIn');

% ----- TD3 options -----
agentOpts = rlTD3AgentOptions( ...
    'SampleTime',              dt_val, ...
    'DiscountFactor',          0.99,   ...
    'MiniBatchSize',           64,     ...
    'ExperienceBufferLength',  1e5,    ...
    'TargetSmoothFactor',      5e-3,   ...
    'NumStepsToLookAhead',     1);
agentOpts.ExplorationModel.StandardDeviation       = 0.3*ones(actDim,1);
agentOpts.ExplorationModel.StandardDeviationDecayRate = 1e-4;

agent = rlTD3Agent(actor, [critic1 critic2], agentOpts);
fprintf('       TD3 agent created (obs=%d, act=%d).\n\n', obsDim, actDim);

%% ── 5. TRAIN ────────────────────────────────────────────────────────────
fprintf('[5/6] Training …\n');
fprintf('       Max episodes  : 500\n');
fprintf('       Steps/episode : %d\n', N-1);
fprintf('       Stop criterion: avg reward > −5 (window=20)\n\n');

outDir = fullfile(scriptDir, 'savedAgents');
if ~isfolder(outDir), mkdir(outDir); end

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes',              500,   ...
    'MaxStepsPerEpisode',       N-1,   ...
    'ScoreAveragingWindowLength', 20,  ...
    'StopTrainingCriteria',     'AverageReward', ...
    'StopTrainingValue',        -5,    ...
    'SaveAgentCriteria',        'EpisodeReward', ...
    'SaveAgentValue',           -10,   ...
    'SaveAgentDirectory',       outDir, ...
    'Verbose',                  true,  ...
    'Plots',                    'training-progress');

trainingStats = train(agent, env, trainOpts);

agentFile = fullfile(outDir, 'spray_td3_final.mat');
save(agentFile, 'agent', 'trainingStats', 'NORM', 'F');
fprintf('\n       Agent saved → %s\n\n', agentFile);

%% ── 6. EVALUATE & COMPARE ──────────────────────────────────────────────
fprintf('[6/6] Evaluating trained agent vs baseline …\n');

% Storage
rl_thick = zeros(N-1,1);
rl_spd   = zeros(N-1,1);
rl_pls   = zeros(N-1,1);
rl_rew   = zeros(N-1,1);
bl_thick = zeros(N-1,1);   % baseline: speed_factor=0.5, pulse_factor=1.0

obs = reset(env);
for k = 1:(N-1)
    act = getAction(agent, {obs});
    act = act{1};
    [obs, rew, done, info] = step(env, act);
    rl_thick(k) = info.thickness;
    rl_spd(k)   = info.speed_factor;
    rl_pls(k)   = info.pulse_factor;
    rl_rew(k)   = rew;

    % Baseline computation
    v  = F.ee_velocity(k);
    sd = F.spray_distance(k);
    bl_thick(k) = min(80*(300/max(v*0.5,1))*(300/max(sd,50))*1.0, 500);

    if done, break; end
end
nEval = find(rl_thick~=0, 1, 'last');
if isempty(nEval), nEval = N-1; end
tEval = F.time_s(1:nEval);

% ── Metrics table ──
fprintf('\n  %-32s %-14s %-14s\n', 'Metric', 'Baseline', 'RL Agent');
fprintf('  %s\n', repmat('─',1,60));
printRow = @(nm,b,r) fprintf('  %-32s %-14.1f %-14.1f\n', nm, b, r);
printRow('Mean thickness (µm)',           mean(bl_thick(1:nEval)),    mean(rl_thick(1:nEval)));
printRow('Std thickness (µm)',            std(bl_thick(1:nEval)),     std(rl_thick(1:nEval)));
fprintf('  %-32s %-13.1f%% %-13.1f%%\n', 'In-spec  50–100 µm', ...
    100*sum(bl_thick(1:nEval)>=50 & bl_thick(1:nEval)<=100)/nEval, ...
    100*sum(rl_thick(1:nEval)>=50 & rl_thick(1:nEval)<=100)/nEval);
fprintf('  %-32s %-13.1f%% %-13.1f%%\n', 'Drip  (>150 µm)', ...
    100*sum(bl_thick(1:nEval)>150)/nEval, ...
    100*sum(rl_thick(1:nEval)>150)/nEval);
fprintf('  %-32s %-13.1f%% %-13.1f%%\n', 'Bare  (<30 µm)', ...
    100*sum(bl_thick(1:nEval)<30)/nEval, ...
    100*sum(rl_thick(1:nEval)<30)/nEval);
fprintf('  %-32s %-14s %-14.1f\n', 'Total reward', '—', sum(rl_rew(1:nEval)));

% ── Big comparison figure ──
fig2 = figure('Name','RL vs Baseline','Position',[40 40 1400 900]);

subplot(3,2,1);
plot(tEval, bl_thick(1:nEval),'b-','LineWidth',1.1); hold on;
plot(tEval, rl_thick(1:nEval),'r-','LineWidth',1.4);
fill([tEval(1) tEval(end) tEval(end) tEval(1)],[50 50 100 100], ...
    'g','FaceAlpha',.08,'EdgeColor','none');
yline(150,'k:','Drip','LineWidth',1);
xlabel('Time (s)'); ylabel('µm');
title('Paint Thickness'); legend('Baseline','RL Agent','Location','best'); grid on;

subplot(3,2,2);
plot(tEval, rl_spd(1:nEval), 'Color',[.8 .15 0],'LineWidth',1.2); hold on;
yline(0.5,'b--','Baseline');
xlabel('Time (s)'); ylabel('Factor'); title('RL Speed Factor'); grid on; ylim([0 1.1]);

subplot(3,2,3);
plot(tEval, rl_pls(1:nEval), 'Color',[0 .6 .3],'LineWidth',1.2); hold on;
yline(1.0,'b--','Baseline');
xlabel('Time (s)'); ylabel('Factor'); title('RL Pulse Width Factor'); grid on;

subplot(3,2,4);
plot(tEval, rl_rew(1:nEval),'Color',[.4 .4 .4]); hold on;
plot(tEval, movmean(rl_rew(1:nEval),20),'r','LineWidth',2);
xlabel('Time (s)'); ylabel('Reward'); title('Per-Step Reward');
legend('Raw','Moving avg 20','Location','best'); grid on;

subplot(3,2,5);
yyaxis left;  plot(tEval, F.ee_velocity(1:nEval),'b','LineWidth',1); ylabel('Vel (mm/s)');
yyaxis right; plot(tEval, rl_spd(1:nEval),'r','LineWidth',1.2);      ylabel('Speed Factor');
xlabel('Time (s)'); title('Velocity ↔ Speed Correlation'); grid on;

subplot(3,2,6);
histogram(bl_thick(1:nEval),30,'FaceColor','b','FaceAlpha',.45); hold on;
histogram(rl_thick(1:nEval),30,'FaceColor','r','FaceAlpha',.45);
xline(50,'g--','LineWidth',2); xline(100,'g--','LineWidth',2);
xlabel('µm'); ylabel('Count'); title('Thickness Distribution');
legend('Baseline','RL','Location','best'); grid on;

sgtitle('Adaptive Spray — RL Agent Evaluation','FontSize',14,'FontWeight','bold');

% ── Export CSV ──
resultFile = fullfile(scriptDir, 'spray_rl_results.csv');
T = table(tEval, rl_spd(1:nEval), rl_pls(1:nEval), ...
          rl_thick(1:nEval), bl_thick(1:nEval), rl_rew(1:nEval), ...
    'VariableNames', {'Time_s','RL_SpeedFactor','RL_PulseFactor', ...
                      'RL_Thickness_um','Baseline_Thickness_um','Reward'});
writetable(T, resultFile);
fprintf('\n  Results exported → %s\n', resultFile);

% ── Save figures ──
savefig(fig1, fullfile(scriptDir, 'dataset_overview.fig'));
savefig(fig2, fullfile(scriptDir, 'rl_evaluation.fig'));
fprintf('  Figures saved → dataset_overview.fig, rl_evaluation.fig\n');

fprintf('\n╔══════════════════════════════════════════════════════════╗\n');
fprintf('║   Pipeline complete.  Review the training-progress      ║\n');
fprintf('║   window and the two figures for results.               ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');

%% ═════════════════════════════════════════════════════════════════════════
%  LOCAL FUNCTION — builds one critic dlnetwork (called twice for TD3)
%  ═════════════════════════════════════════════════════════════════════════
function net = createCriticNet(obsDim, actDim)
    statePath = [
        featureInputLayer(obsDim, 'Normalization','none', 'Name','stateIn')
        fullyConnectedLayer(128, 'Name','sfc1')
        reluLayer('Name','sr1')
    ];
    actionPath = [
        featureInputLayer(actDim, 'Normalization','none', 'Name','actionIn')
        fullyConnectedLayer(128, 'Name','afc1')
        reluLayer('Name','ar1')
    ];
    commonPath = [
        additionLayer(2, 'Name','add')
        fullyConnectedLayer(256, 'Name','cfc1')
        reluLayer('Name','cr1')
        fullyConnectedLayer(128, 'Name','cfc2')
        reluLayer('Name','cr2')
        fullyConnectedLayer(1, 'Name','qval')
    ];
    lg = layerGraph();
    lg = addLayers(lg, statePath);
    lg = addLayers(lg, actionPath);
    lg = addLayers(lg, commonPath);
    lg = connectLayers(lg, 'sr1', 'add/in1');
    lg = connectLayers(lg, 'ar1', 'add/in2');
    net = dlnetwork(lg);
end