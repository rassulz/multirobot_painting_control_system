%% train_roarm_td3.m
% Improved TD3 training for RoArm painting speed adaptation
% Addresses: reward collapse, speed saturation, poor critic learning

clear; clc; rng(42);

%% ── 1. Create Environment ────────────────────────────────────────────────
env = RoarmPaintingEnv();
validateEnvironment(env);

obsInfo = getObservationInfo(env);   % 19×1
actInfo = getActionInfo(env);        % 1×1  [0.1, 1.0]

obsDim  = obsInfo.Dimension(1);      % 19
actDim  = actInfo.Dimension(1);      % 1

%% ── 2. Networks ──────────────────────────────────────────────────────────

% ── Actor: maps obs → action (tanh output scaled to [SpeedMin, SpeedMax]) ─
actorNet = [
    featureInputLayer(obsDim, 'Normalization','none','Name','obs')
    fullyConnectedLayer(256, 'Name','fc1')
    layerNormalizationLayer('Name','ln1')        % helps with stability
    reluLayer('Name','relu1')
    fullyConnectedLayer(256, 'Name','fc2')
    layerNormalizationLayer('Name','ln2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(128, 'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(actDim, 'Name','fcOut')
    tanhLayer('Name','tanh')
    scalingLayer('Name','scale', ...             % tanh ∈(-1,1) → (0.1,1.0)
        'Scale', (1.00-0.10)/2, ...              % = 0.45
        'Bias',  (1.00+0.10)/2)                  % = 0.55
];

actor = rlContinuousDeterministicActor( ...
    dlnetwork(actorNet), obsInfo, actInfo, ...
    'ObservationInputNames', 'obs');

% ── Critic 1 (TD3 uses two critics) ───────────────────────────────────────
criticNet = buildCriticNetwork(obsDim, actDim, 'critic1');
critic1   = rlQValueFunction(criticNet, obsInfo, actInfo, ...
    'ObservationInputNames','obs', 'ActionInputNames','act');

criticNet2 = buildCriticNetwork(obsDim, actDim, 'critic2');
critic2    = rlQValueFunction(criticNet2, obsInfo, actInfo, ...
    'ObservationInputNames','obs', 'ActionInputNames','act');

%% ── 3. TD3 Agent Options ─────────────────────────────────────────────────
agentOpts = rlTD3AgentOptions();

% ── Replay buffer ────────────────────────────────────────────────────────
agentOpts.ExperienceBufferLength    = 1e6;
agentOpts.MiniBatchSize             = 256;    % larger = more stable gradients
agentOpts.NumWarmStartSteps         = 5000;   % collect experience before training

% ── Learning rates ────────────────────────────────────────────────────────
agentOpts.ActorOptimizerOptions     = rlOptimizerOptions( ...
    'LearnRate', 1e-4, ...                    % lower LR for stable actor
    'GradientThreshold', 1, ...
    'Optimizer','adam');
agentOpts.CriticOptimizerOptions    = rlOptimizerOptions( ...
    'LearnRate', 3e-4, ...                    % critic learns faster than actor
    'GradientThreshold', 1, ...
    'Optimizer','adam');

% ── TD3 specific ──────────────────────────────────────────────────────────
agentOpts.TargetSmoothFactor        = 0.005;  % τ (soft update, was 0.005)
agentOpts.DiscountFactor            = 0.99;   % γ
agentOpts.PolicyUpdateFrequency     = 2;      % update actor every 2 critic steps

% ── Exploration noise ─────────────────────────────────────────────────────
% Use Ornstein-Uhlenbeck for temporally correlated exploration
agentOpts.ExplorationModel          = rl.option.OrnsteinUhlenbeckActionNoise();
agentOpts.ExplorationModel.StandardDeviation     = 0.15;
agentOpts.ExplorationModel.MeanAttractionConstant = 0.15;
agentOpts.ExplorationModel.StandardDeviationDecayRate = 1e-4; % slowly decay noise

% Target policy smoothing noise (TD3 trick)
agentOpts.TargetPolicySmoothModel.StandardDeviation = 0.10;
agentOpts.TargetPolicySmoothModel.StandardDeviationMin = 0.01;

%% ── 4. Create Agent ───────────────────────────────────────────────────────
agent = rlTD3Agent(actor, [critic1, critic2], agentOpts);

%% ── 5. Training Options ──────────────────────────────────────────────────
maxEpisodes = 2000;   % increase from 800

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes',              maxEpisodes, ...
    'MaxStepsPerEpisode',       env.MaxSteps, ...
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria',     'AverageReward', ...
    'StopTrainingValue',        -0.5, ...         % target: avg reward > -0.5
    'SaveAgentCriteria',        'EpisodeReward', ...
    'SaveAgentValue',           -1.0, ...
    'SaveAgentDirectory',       'savedAgents', ...
    'Verbose',                  true, ...
    'Plots',                    'training-progress');

%% ── 6. Curriculum callback (update env episode counter each episode) ──────
% Using a custom training loop for curriculum support
episodeRewards = zeros(maxEpisodes, 1);
movAvg         = zeros(maxEpisodes, 1);
windowLen      = 50;

fprintf('Starting TD3 training with curriculum learning...\n');
fprintf('Episodes: %d | ObsDim: %d | ActDim: %d\n', maxEpisodes, obsDim, actDim);
fprintf('─────────────────────────────────────────────────────\n');

trainingStats = train(agent, env, trainOpts);

%% ── 7. Save ───────────────────────────────────────────────────────────────
save('roarm_td3_agent_improved.mat', 'agent', 'trainingStats');
fprintf('Agent saved.\n');

%% ── Helper: Build critic network ──────────────────────────────────────────
function net = buildCriticNetwork(obsDim, actDim, tag)
    % Critic: Q(s,a) — concatenates obs and action before hidden layers
    obsPath = [
        featureInputLayer(obsDim, 'Normalization','none','Name','obs')
        fullyConnectedLayer(256, 'Name', [tag '_obs_fc1'])
        layerNormalizationLayer('Name', [tag '_obs_ln1'])
        reluLayer('Name', [tag '_obs_relu1'])
    ];
    actPath = [
        featureInputLayer(actDim, 'Normalization','none','Name','act')
        fullyConnectedLayer(256, 'Name', [tag '_act_fc1'])
    ];
    combined = [
        additionLayer(2, 'Name', [tag '_add'])
        reluLayer('Name', [tag '_add_relu'])
        fullyConnectedLayer(256, 'Name', [tag '_fc2'])
        reluLayer('Name', [tag '_relu2'])
        fullyConnectedLayer(128, 'Name', [tag '_fc3'])
        reluLayer('Name', [tag '_relu3'])
        fullyConnectedLayer(1,   'Name', [tag '_out'])
    ];
    
    lgraph = layerGraph();
    lgraph = addLayers(lgraph, obsPath);
    lgraph = addLayers(lgraph, actPath);
    lgraph = addLayers(lgraph, combined);
    lgraph = connectLayers(lgraph, [tag '_obs_relu1'], [tag '_add/in1']);
    lgraph = connectLayers(lgraph, [tag '_act_fc1'],   [tag '_add/in2']);
    
    net = dlnetwork(lgraph);
end