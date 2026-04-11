%% TRAINSPRAYAGENT - Train RL agent for adaptive spray parameter control
%  Uses PPO (Proximal Policy Optimization) for mixed continuous actions.
%
%  Prerequisites:
%    - Reinforcement Learning Toolbox (R2019b+)
%    - Deep Learning Toolbox
%    - kuka_rl_dataset.csv in the working directory
%
%  Usage:
%    >> trainSprayAgent
%
%  Author: Auto-generated for multirobot_painting_control_system project

clear; clc; close all;
fprintf('=== KUKA Adaptive Spray RL Training ===\n\n');

%% 1. Load dataset and extract features
csvPath = fullfile('C:','Users','SYNAPTICON','Desktop', ...
            'multirobot_painting_control_system','MATLAB','RL', ...
            'kuka_rl_dataset.csv');
if ~isfile(csvPath)
    error('Dataset not found: %s\nPlace kuka_rl_dataset.csv in %s', csvPath, pwd);
end
[data, features, labels] = loadKukaRLDataset(csvPath);

%% 2. Visualize dataset before training
figure('Name', 'Dataset Overview', 'Position', [100 100 1200 800]);

subplot(2,3,1);
plot3(features.target_xyz(:,1), features.target_xyz(:,2), features.target_xyz(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot3(features.meas_xyz(:,1), features.meas_xyz(:,2), features.meas_xyz(:,3), 'r--');
legend('Target', 'Measured'); xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('3D Trajectory'); grid on;

subplot(2,3,2);
plot(features.time_s, features.ee_velocity, 'b-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Velocity (mm/s)');
title('End-Effector Velocity'); grid on;

subplot(2,3,3);
plot(features.time_s, features.tracking_error, 'r-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Error (mm)');
title('Tracking Error'); grid on;

subplot(2,3,4);
plot(features.time_s, features.spray_distance, 'g-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Distance (mm)');
title('Spray Distance (to surface)'); grid on;

subplot(2,3,5);
plot(features.time_s, labels.estimated_thickness, 'm-', 'LineWidth', 1.2);
hold on;
yline(50, 'g--', 'Min ideal'); yline(100, 'g--', 'Max ideal');
yline(150, 'r--', 'Drip threshold');
xlabel('Time (s)'); ylabel('Thickness (μm)');
title('Simulated Paint Thickness (No RL)'); grid on;

subplot(2,3,6);
plot(features.time_s, features.torque_magnitude, 'Color', [0.8 0.4 0], 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Torque Magnitude');
title('Joint Torque Aggregate'); grid on;

sgtitle('KUKA KR10 R1100 - Dataset Analysis', 'FontSize', 14, 'FontWeight', 'bold');
drawnow;

%% 3. Create RL environment
env = KukaSprayEnv(features, labels);
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

fprintf('\nEnvironment created:\n');
fprintf('  Observation space: %d dimensions\n', obsInfo.Dimension(1));
fprintf('  Action space: %d dimensions (speed_factor, pulse_factor)\n', actInfo.Dimension(1));

%% 4. Create PPO agent with actor-critic networks
% --- Actor Network (outputs mean of action distribution) ---
actorLayers = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'obs_in')
    fullyConnectedLayer(128, 'Name', 'actor_fc1')
    reluLayer('Name', 'actor_relu1')
    fullyConnectedLayer(128, 'Name', 'actor_fc2')
    reluLayer('Name', 'actor_relu2')
    fullyConnectedLayer(64, 'Name', 'actor_fc3')
    reluLayer('Name', 'actor_relu3')
    fullyConnectedLayer(actInfo.Dimension(1), 'Name', 'actor_out')
];

% Create actor representation
actorOpts = rlOptimizerOptions('LearnRate', 1e-4, 'GradientThreshold', 1);

% --- Critic Network (estimates state value V(s)) ---
criticLayers = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'obs_in')
    fullyConnectedLayer(128, 'Name', 'critic_fc1')
    reluLayer('Name', 'critic_relu1')
    fullyConnectedLayer(128, 'Name', 'critic_fc2')
    reluLayer('Name', 'critic_relu2')
    fullyConnectedLayer(64, 'Name', 'critic_fc3')
    reluLayer('Name', 'critic_relu3')
    fullyConnectedLayer(1, 'Name', 'critic_out')
];

criticOpts = rlOptimizerOptions('LearnRate', 2e-4, 'GradientThreshold', 1);

% Create actor and critic
actor  = rlContinuousGaussianActor(actorLayers, obsInfo, actInfo, ...
    'ObservationInputNames', 'obs_in');
critic = rlValueFunction(criticLayers, obsInfo, ...
    'ObservationInputNames', 'obs_in');

% --- PPO Agent Configuration ---
agentOpts = rlPPOAgentOptions(...
    'ExperienceHorizon', 256, ...        % Steps before each update
    'ClipFactor', 0.2, ...               % PPO clip ratio
    'EntropyLossWeight', 0.01, ...       % Exploration encouragement
    'MiniBatchSize', 64, ...
    'NumEpoch', 4, ...                   % Epochs per update
    'AdvantageEstimateMethod', 'gae', ...
    'GAEFactor', 0.95, ...
    'DiscountFactor', 0.99, ...
    'SampleTime', 0.03, ...              % Matches your dt
    'ActorOptimizerOptions', actorOpts, ...
    'CriticOptimizerOptions', criticOpts);

agent = rlPPOAgent(actor, critic, agentOpts);
fprintf('PPO agent created with %d actor params, %d critic params\n', ...
    sum(cellfun(@numel, actor.Learnables.Value)), ...
    sum(cellfun(@numel, critic.Learnables.Value)));

%% 5. Configure training
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 2000, ...
    'MaxStepsPerEpisode', 272, ...       % Your dataset has 273 steps
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 2200, ...       % ~8.0 reward/step avg
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 2000, ...
    'SaveAgentDirectory', fullfile(pwd, 'trained_agents'), ...
    'Verbose', true, ...
    'Plots', 'training-progress');

%% 6. Train!
fprintf('\nStarting training...\n');
fprintf('Target: average reward > 2200 over 50 episodes\n');
fprintf('Max episodes: 2000 | Steps/episode: 272\n\n');

trainingStats = train(agent, env, trainOpts);

%% 7. Save trained agent
savePath = fullfile(pwd, 'trained_agents', 'spray_ppo_agent.mat');
save(savePath, 'agent', 'trainingStats');
fprintf('\nAgent saved to: %s\n', savePath);

%% 8. Post-training analysis
fprintf('\n=== Training Summary ===\n');
fprintf('Total episodes: %d\n', trainingStats.EpisodeIndex(end));
fprintf('Final avg reward: %.1f\n', trainingStats.AverageReward(end));
fprintf('Best episode reward: %.1f\n', max(trainingStats.EpisodeReward));