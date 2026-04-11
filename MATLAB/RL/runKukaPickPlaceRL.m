%% =========================================================================
%  runKukaPickPlaceRL.m  —  RL for KUKA KR10 Pick-and-Place Optimisation
% ==========================================================================
%  The KUKA performs pick-and-place: it moves workpieces between a pick
%  station and a place station (the painting station served by RoArms).
%
%  The RL agent learns an optimal VELOCITY PROFILE along the trajectory
%  to minimise:  cycle time + tracking error + jerk + energy
%
%  Requirements:
%    • MATLAB R2022a+, Reinforcement Learning Toolbox, Deep Learning Toolbox
%    • kuka_rl_dataset.csv in the same folder as this script
%
%  Usage:   >> runKukaPickPlaceRL
% ==========================================================================
clear; clc; close all;
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║  KUKA KR10 — Pick-and-Place RL Velocity Optimisation     ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

%% ── 0. RESOLVE DATASET ──────────────────────────────────────────────────
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end
csvPath = fullfile(scriptDir, 'kuka_rl_dataset.csv');
if ~isfile(csvPath)
    csvPath = fullfile('C:','Users','SYNAPTICON','Desktop', ...
        'multirobot_painting_control_system','MATLAB','RL','kuka_rl_dataset.csv');
end
assert(isfile(csvPath), 'Cannot find kuka_rl_dataset.csv at:\n  %s', csvPath);

%% ── 1. LOAD DATA & ENGINEER FEATURES ───────────────────────────────────
fprintf('[1/6] Loading dataset …\n');
data = readtable(csvPath, 'VariableNamingRule', 'preserve');
N    = height(data);
dt   = data.dt(1);  % 0.03s
fprintf('       %d timesteps, dt=%.3fs, total=%.2fs\n', N, dt, data.time_s(end));

% --- Core signals ---
target_xyz = [data.target_x, data.target_y, data.target_z];
meas_xyz   = [data.meas_x,   data.meas_y,   data.meas_z];
time_s     = data.time_s;

pick_pt  = [data.pick_x(1), data.pick_y(1), data.pick_z(1)];    % [608, 117, 263]
place_pt = [data.place_x(1), data.place_y(1), data.place_z(1)];  % [45, 204, 204]
fprintf('       Pick:  [%.0f, %.0f, %.0f] mm\n', pick_pt);
fprintf('       Place: [%.0f, %.0f, %.0f] mm\n', place_pt);

% --- Feature: tracking error (mm) ---
tracking_error = vecnorm(target_xyz - meas_xyz, 2, 2);

% --- Feature: end-effector velocity (mm/s) ---
ee_velocity = zeros(N,1);
for k = 2:N
    ee_velocity(k) = norm(target_xyz(k,:) - target_xyz(k-1,:)) / dt;
end
ee_velocity(1) = ee_velocity(2);

% --- Feature: end-effector acceleration (mm/s²) ---
ee_accel = zeros(N,1);
for k = 2:N
    ee_accel(k) = (ee_velocity(k) - ee_velocity(k-1)) / dt;
end
ee_accel(1) = ee_accel(2);

% --- Feature: jerk (mm/s³)  — key for smooth motion ---
jerk = zeros(N,1);
for k = 2:N
    jerk(k) = (ee_accel(k) - ee_accel(k-1)) / dt;
end
jerk(1) = jerk(2);

% --- Feature: path curvature (rad) ---
curvature = zeros(N,1);
for k = 3:N
    v1 = target_xyz(k-1,:) - target_xyz(k-2,:);
    v2 = target_xyz(k,:)   - target_xyz(k-1,:);
    n1 = norm(v1); n2 = norm(v2);
    if n1>1e-6 && n2>1e-6
        curvature(k) = acos(max(-1, min(1, dot(v1,v2)/(n1*n2))));
    end
end

% --- Feature: joint torque magnitude (Nm aggregate) ---
torques = [data.torque_j1, data.torque_j2, data.torque_j3, ...
           data.torque_j4, data.torque_j5, data.torque_j6];
torque_mag = vecnorm(torques, 2, 2);

% --- Feature: joint current magnitude (energy proxy) ---
currents = [data.current_j1, data.current_j2, data.current_j3, ...
            data.current_j4, data.current_j5, data.current_j6];
curr_mag = vecnorm(currents, 2, 2);

% --- Feature: progress along trajectory (0→1) ---
cum_path = zeros(N,1);
for k = 2:N
    cum_path(k) = cum_path(k-1) + norm(target_xyz(k,:)-target_xyz(k-1,:));
end
progress = cum_path / max(cum_path(end), 1e-6);

% --- Feature: distance to pick / distance to place ---
dist_to_pick  = vecnorm(target_xyz - pick_pt,  2, 2);
dist_to_place = vecnorm(target_xyz - place_pt, 2, 2);

% --- Feature: payload phase ---
%  Near pick point = approaching/grasping, near place = releasing
%  This tells the agent to slow down near endpoints
total_dist    = norm(place_pt - pick_pt);
norm_to_pick  = dist_to_pick  / total_dist;
norm_to_place = dist_to_place / total_dist;

% --- Pack into struct ---
F.tracking_error = tracking_error;
F.ee_velocity    = ee_velocity;
F.ee_accel       = ee_accel;
F.jerk           = jerk;
F.curvature      = curvature;
F.torque_mag     = torque_mag;
F.curr_mag       = curr_mag;
F.progress       = progress;
F.norm_to_pick   = norm_to_pick;
F.norm_to_place  = norm_to_place;
F.target_xyz     = target_xyz;
F.meas_xyz       = meas_xyz;
F.time_s         = time_s;

% --- Normalisation ceilings ---
NM.vel     = max(ee_velocity);
NM.accel   = max(abs(ee_accel));
NM.jerk    = max(abs(jerk));
NM.curv    = max(max(curvature), 1e-6);
NM.trkErr  = max(tracking_error);
NM.torque  = max(torque_mag);
NM.curr    = max(curr_mag);

fprintf('       Feature engineering done — pick-and-place features.\n\n');

%% ── 2. VISUALISE RAW DATASET ───────────────────────────────────────────
fprintf('[2/6] Plotting dataset …\n');
fig1 = figure('Name','KUKA Pick-Place Dataset','Position',[60 60 1300 820]);

subplot(2,3,1);
plot3(target_xyz(:,1), target_xyz(:,2), target_xyz(:,3), 'b-', 'LineWidth',1.4); hold on;
plot3(meas_xyz(:,1), meas_xyz(:,2), meas_xyz(:,3), 'r--', 'LineWidth',0.8);
scatter3(pick_pt(1), pick_pt(2), pick_pt(3), 200, 'g', 'filled', 'DisplayName','PICK');
scatter3(place_pt(1), place_pt(2), place_pt(3), 200, 'm', 'filled', 'DisplayName','PLACE');
legend('Target','Measured','PICK','PLACE','Location','best');
xlabel('X'); ylabel('Y'); zlabel('Z'); title('3D Pick→Place Trajectory'); grid on;

subplot(2,3,2);
plot(time_s, ee_velocity, 'b', 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('mm/s'); title('End-Effector Velocity'); grid on;

subplot(2,3,3);
plot(time_s, tracking_error, 'r', 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('mm'); title('Tracking Error'); grid on;

subplot(2,3,4);
plot(time_s, torque_mag, 'Color',[.8 .35 0], 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Nm (agg)'); title('Joint Torque Magnitude'); grid on;

subplot(2,3,5);
plot(time_s, curr_mag, 'Color',[.4 0 .6], 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('A (agg)'); title('Joint Current (Energy)'); grid on;

subplot(2,3,6);
yyaxis left;  plot(time_s, dist_to_pick, 'g', 'LineWidth',1.2); ylabel('Dist to Pick (mm)');
yyaxis right; plot(time_s, dist_to_place,'m', 'LineWidth',1.2);  ylabel('Dist to Place (mm)');
xlabel('Time (s)'); title('Distance to Pick / Place'); grid on;

sgtitle('KUKA KR10 — Pick-and-Place Raw Data','FontSize',14,'FontWeight','bold');
drawnow;

%% ── 3. WRITE RL ENVIRONMENT CLASS ──────────────────────────────────────
fprintf('[3/6] Creating pick-and-place RL environment …\n');
envFile = fullfile(scriptDir, 'KukaPickPlaceEnv.m');
fid = fopen(envFile, 'w');
envCode = {
'classdef KukaPickPlaceEnv < rl.env.MATLABEnvironment'
'%  RL environment for KUKA KR10 pick-and-place velocity optimisation.'
'%'
'%  State (8x1 normalised 0-1):'
'%    [ee_velocity, |ee_accel|, curvature, tracking_error,'
'%     torque_mag,  progress,  dist_to_pick_norm, dist_to_place_norm]'
'%'
'%  Action (1x1 continuous):'
'%    speed_override in [0.1, 1.0]  — scales the KUKA $OV_PRO velocity'
'%'
'%  Reward: minimise cycle_time + tracking_error + jerk + energy'
''
'    properties'
'        F;  NM;  NumSteps;  StepIdx;  PrevSpeed;'
'        CumEnergy;  CumTrackErr;  CumJerk;'
'    end'
''
'    methods'
'        function this = KukaPickPlaceEnv(features, normStruct)'
'            obsInfo = rlNumericSpec([8 1],''LowerLimit'',zeros(8,1),''UpperLimit'',ones(8,1));'
'            obsInfo.Name = ''state'';'
'            actInfo = rlNumericSpec([1 1],''LowerLimit'',0.1,''UpperLimit'',1.0);'
'            actInfo.Name = ''speed_override'';'
'            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);'
'            this.F  = features;'
'            this.NM = normStruct;'
'            this.NumSteps = length(features.ee_velocity);'
'        end'
''
'        function [obs,reward,isDone,info] = step(this,action)'
'            spd = max(0.1, min(1.0, action(1)));'
'            i   = this.StepIdx;'
''
'            %% --- Reward components (all normalised to ~[-1, +1]) ---'
''
'            % R1: Cycle time penalty — faster is better'
'            %     Higher speed_override = less time per step = good'
'            r_time = 0.3 * (spd - 0.5);  % reward > 0 when spd > 0.5'
''
'            % R2: Tracking error penalty — lower is better'
'            te = this.F.tracking_error(i);'
'            r_track = -0.4 * (te / max(this.NM.trkErr, 1e-6));'
''
'            % R3: Jerk penalty — smoother is better'
'            j = abs(this.F.jerk(i));'
'            r_jerk = -0.15 * (j / max(this.NM.jerk, 1e-6));'
''
'            % R4: Energy penalty — lower current = less wear'
'            c = this.F.curr_mag(i);'
'            r_energy = -0.1 * (c / max(this.NM.curr, 1e-6));'
''
'            % R5: Smoothness — penalise abrupt speed changes'
'            r_smooth = -0.3 * abs(spd - this.PrevSpeed);'
''
'            % R6: Endpoint approach bonus — slow down near pick/place'
'            nearPick  = this.F.norm_to_pick(i)  < 0.1;'
'            nearPlace = this.F.norm_to_place(i) < 0.1;'
'            if (nearPick || nearPlace) && spd < 0.4'
'                r_approach = 0.2;   % good: slow near endpoints'
'            elseif (nearPick || nearPlace) && spd > 0.7'
'                r_approach = -0.3;  % bad: too fast near endpoints'
'            else'
'                r_approach = 0;'
'            end'
''
'            reward = r_time + r_track + r_jerk + r_energy + r_smooth + r_approach;'
''
'            % Accumulate metrics'
'            this.CumEnergy   = this.CumEnergy   + c;'
'            this.CumTrackErr = this.CumTrackErr  + te;'
'            this.CumJerk     = this.CumJerk      + j;'
''
'            this.PrevSpeed = spd;'
'            this.StepIdx   = this.StepIdx + 1;'
'            isDone = (this.StepIdx >= this.NumSteps);'
''
'            if ~isDone'
'                obs = getObs(this);'
'            else'
'                obs = zeros(8,1);'
'                % Terminal bonus for completing the trajectory'
'                reward = reward + 5;'
'            end'
'            info.speed_override = spd;'
'            info.tracking_error = te;'
'            info.energy = c;'
'        end'
''
'        function obs = reset(this)'
'            this.StepIdx     = 1;'
'            this.PrevSpeed   = 0.5;'
'            this.CumEnergy   = 0;'
'            this.CumTrackErr = 0;'
'            this.CumJerk     = 0;'
'            obs = getObs(this);'
'        end'
''
'        function obs = getObs(this)'
'            i = this.StepIdx;'
'            obs = ['
'                this.F.ee_velocity(i)    / max(this.NM.vel, 1e-6);'
'                abs(this.F.ee_accel(i))  / max(this.NM.accel, 1e-6);'
'                this.F.curvature(i)      / max(this.NM.curv, 1e-6);'
'                this.F.tracking_error(i) / max(this.NM.trkErr, 1e-6);'
'                this.F.torque_mag(i)     / max(this.NM.torque, 1e-6);'
'                this.F.progress(i);'
'                this.F.norm_to_pick(i);'
'                this.F.norm_to_place(i)'
'            ];'
'            obs = max(0, min(1, obs));'
'        end'
'    end'
'end'
};
for r = 1:length(envCode)
    fprintf(fid, '%s\n', envCode{r});
end
fclose(fid);
rehash;
fprintf('       KukaPickPlaceEnv.m written.\n\n');

%% ── 4. BUILD TD3 AGENT ─────────────────────────────────────────────────
fprintf('[4/6] Building TD3 agent …\n');

env     = KukaPickPlaceEnv(F, NM);
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);
obsDim  = obsInfo.Dimension(1);   % 8
actDim  = actInfo.Dimension(1);   % 1

% --- Twin critics ---
buildCritic = @() createCriticNet(obsDim, actDim);
critic1 = rlQValueFunction(buildCritic(), obsInfo, actInfo, ...
    'ObservationInputNames','stateIn','ActionInputNames','actionIn');
critic2 = rlQValueFunction(buildCritic(), obsInfo, actInfo, ...
    'ObservationInputNames','stateIn','ActionInputNames','actionIn');

% --- Actor (outputs speed_override ∈ [0.1, 1.0]) ---
actorNet = dlnetwork([
    featureInputLayer(obsDim,'Normalization','none','Name','stateIn')
    fullyConnectedLayer(256,'Name','afc1')
    reluLayer('Name','ar1')
    fullyConnectedLayer(128,'Name','afc2')
    reluLayer('Name','ar2')
    fullyConnectedLayer(actDim,'Name','afc3')
    sigmoidLayer('Name','sig')
    scalingLayer('Name','scale', ...
        'Scale', actInfo.UpperLimit - actInfo.LowerLimit, ...
        'Bias',  actInfo.LowerLimit)
]);
actor = rlContinuousDeterministicActor(actorNet, obsInfo, actInfo, ...
    'ObservationInputNames','stateIn');

% --- TD3 options (tuned for stable training) ---
agentOpts = rlTD3AgentOptions( ...
    'SampleTime',              dt, ...
    'DiscountFactor',          0.99, ...
    'MiniBatchSize',           128, ...
    'ExperienceBufferLength',  1e5, ...
    'TargetSmoothFactor',      1e-3);      % slower target updates
agentOpts.ExplorationModel.StandardDeviation          = 0.15;
agentOpts.ExplorationModel.StandardDeviationDecayRate = 1e-4;

agent = rlTD3Agent(actor, [critic1 critic2], agentOpts);
fprintf('       TD3 agent created (obs=%d, act=%d).\n\n', obsDim, actDim);

%% ── 5. TRAIN ────────────────────────────────────────────────────────────
fprintf('[5/6] Training …\n');
outDir = fullfile(scriptDir, 'savedAgents');
if ~isfolder(outDir), mkdir(outDir); end

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes',               1000, ...
    'MaxStepsPerEpisode',        N-1, ...
    'ScoreAveragingWindowLength', 30, ...
    'StopTrainingCriteria',      'AverageReward', ...
    'StopTrainingValue',         -5, ...
    'SaveAgentCriteria',         'EpisodeReward', ...
    'SaveAgentValue',            -10, ...
    'SaveAgentDirectory',        outDir, ...
    'Verbose',                   true, ...
    'Plots',                     'training-progress');

trainingStats = train(agent, env, trainOpts);

agentFile = fullfile(outDir, 'kuka_pickplace_td3.mat');
save(agentFile, 'agent', 'trainingStats', 'NM', 'F');
fprintf('\n       Agent saved → %s\n\n', agentFile);

%% ── 6. EVALUATE ─────────────────────────────────────────────────────────
fprintf('[6/6] Evaluating …\n');

rl_spd = zeros(N-1,1);  rl_rew = zeros(N-1,1);
bl_spd = 0.5 * ones(N-1,1);  % baseline: constant 50% speed

obs = reset(env);
for k = 1:(N-1)
    act = getAction(agent, {obs}); act = act{1};
    [obs, rew, done, info] = step(env, act);
    rl_spd(k) = info.speed_override;
    rl_rew(k) = rew;
    if done, break; end
end
nE = find(rl_spd~=0,1,'last'); if isempty(nE), nE=N-1; end
tE = F.time_s(1:nE);

% --- Metrics ---
fprintf('\n  %-34s %-14s %-14s\n', 'Metric', 'Baseline', 'RL Agent');
fprintf('  %s\n', repmat('─',1,62));
fprintf('  %-34s %-14.3f %-14.3f\n', 'Mean speed override', ...
    mean(bl_spd(1:nE)), mean(rl_spd(1:nE)));
fprintf('  %-34s %-14.1f %-14.1f\n', 'Mean tracking error (mm)', ...
    mean(F.tracking_error(1:nE)), mean(F.tracking_error(1:nE)));
fprintf('  %-34s %-14.1f %-14.1f\n', 'Total reward', ...
    sum(-0.4*(F.tracking_error(1:nE)/NM.trkErr)), sum(rl_rew(1:nE)));

% --- Comparison figure ---
fig2 = figure('Name','RL Pick-Place Evaluation','Position',[40 40 1400 800]);

subplot(2,3,1);
plot(tE, rl_spd(1:nE),'r','LineWidth',1.4); hold on;
plot(tE, bl_spd(1:nE),'b--','LineWidth',1);
xlabel('Time (s)'); ylabel('Speed Override');
title('RL Velocity Profile vs Baseline'); legend('RL','Baseline 50%'); grid on;

subplot(2,3,2);
plot(tE, F.tracking_error(1:nE),'r','LineWidth',1.2);
xlabel('Time (s)'); ylabel('mm'); title('Tracking Error'); grid on;

subplot(2,3,3);
plot(tE, F.ee_velocity(1:nE),'b','LineWidth',1); hold on;
yyaxis right; plot(tE, rl_spd(1:nE),'r','LineWidth',1.2); ylabel('Speed Override');
xlabel('Time (s)'); title('Velocity ↔ RL Speed'); grid on;

subplot(2,3,4);
plot(tE, rl_rew(1:nE),'Color',[.4 .4 .4]); hold on;
plot(tE, movmean(rl_rew(1:nE),20),'r','LineWidth',2);
xlabel('Time (s)'); ylabel('Reward'); title('Per-Step Reward'); grid on;

subplot(2,3,5);
plot(tE, F.norm_to_pick(1:nE),'g','LineWidth',1.2); hold on;
plot(tE, F.norm_to_place(1:nE),'m','LineWidth',1.2);
plot(tE, rl_spd(1:nE),'r--','LineWidth',1);
xlabel('Time (s)'); title('Proximity ↔ Speed'); 
legend('Near Pick','Near Place','RL Speed'); grid on;

subplot(2,3,6);
histogram(rl_spd(1:nE), 30, 'FaceColor','r','FaceAlpha',.6); hold on;
xline(0.5,'b--','Baseline','LineWidth',2);
xlabel('Speed Override'); ylabel('Count'); title('RL Speed Distribution'); grid on;

sgtitle('KUKA KR10 Pick-and-Place — RL Agent Evaluation','FontSize',14,'FontWeight','bold');

% --- Export ---
T = table(tE, rl_spd(1:nE), rl_rew(1:nE), ...
    F.tracking_error(1:nE), F.ee_velocity(1:nE), F.torque_mag(1:nE), ...
    'VariableNames', {'Time_s','RL_SpeedOverride','Reward', ...
                      'TrackingError_mm','EE_Velocity_mms','TorqueMag_Nm'});
writetable(T, fullfile(scriptDir,'pickplace_rl_results.csv'));
savefig(fig1, fullfile(scriptDir,'pickplace_dataset.fig'));
savefig(fig2, fullfile(scriptDir,'pickplace_evaluation.fig'));
fprintf('  Results exported.\n');

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║  Pipeline complete.                                       ║\n');
fprintf('║  The agent learns WHEN to go fast (straight segments)     ║\n');
fprintf('║  and WHEN to slow down (near pick/place, high curvature). ║\n');
fprintf('╚════════════════════════════════════════════════════════════���\n');

%% ═══════ LOCAL: twin-critic network builder ═════════════════════════════
function net = createCriticNet(obsDim, actDim)
    sPath = [
        featureInputLayer(obsDim,'Normalization','none','Name','stateIn')
        fullyConnectedLayer(128,'Name','sfc1')
        reluLayer('Name','sr1')];
    aPath = [
        featureInputLayer(actDim,'Normalization','none','Name','actionIn')
        fullyConnectedLayer(128,'Name','afc1')
        reluLayer('Name','ar1')];
    cPath = [
        additionLayer(2,'Name','add')
        fullyConnectedLayer(256,'Name','cfc1')
        reluLayer('Name','cr1')
        fullyConnectedLayer(128,'Name','cfc2')
        reluLayer('Name','cr2')
        fullyConnectedLayer(1,'Name','qval')];
    lg = layerGraph();
    lg = addLayers(lg, sPath);
    lg = addLayers(lg, aPath);
    lg = addLayers(lg, cPath);
    lg = connectLayers(lg, 'sr1', 'add/in1');
    lg = connectLayers(lg, 'ar1', 'add/in2');
    net = dlnetwork(lg);
end