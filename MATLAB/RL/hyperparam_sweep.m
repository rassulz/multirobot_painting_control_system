%% hyperparam_sweep.m
% Quick parallel sweep over key hyperparameters
% Runs N_TRIALS short training sessions and reports best config

clear; clc;

N_TRIALS   = 8;
N_EPISODES = 300;   % short runs for sweep

configs = struct();
% Define sweep grid
lrActor  = [1e-4, 3e-4];
lrCritic = [3e-4, 1e-3];
batchSz  = [128,  256];

k = 1;
for la = lrActor
    for lc = lrCritic
        for bs = batchSz
            configs(k).lrActor  = la;
            configs(k).lrCritic = lc;
            configs(k).batch    = bs;
            k = k+1;
        end
    end
end

results = zeros(numel(configs), 1);

for k = 1:numel(configs)
    cfg = configs(k);
    fprintf('[Sweep %d/%d] lr_a=%.0e lr_c=%.0e batch=%d\n', ...
            k, numel(configs), cfg.lrActor, cfg.lrCritic, cfg.batch);
    
    env = RoarmPaintingEnv();
    [actor, critic1, critic2] = buildNetworks(env);
    
    agOpts = rlTD3AgentOptions();
    agOpts.ExperienceBufferLength   = 2e5;
    agOpts.MiniBatchSize            = cfg.batch;
    agOpts.NumWarmStartSteps        = 1000;
    agOpts.ActorOptimizerOptions    = rlOptimizerOptions('LearnRate',cfg.lrActor);
    agOpts.CriticOptimizerOptions   = rlOptimizerOptions('LearnRate',cfg.lrCritic);
    agOpts.DiscountFactor           = 0.99;
    agOpts.TargetSmoothFactor       = 0.005;
    
    agent = rlTD3Agent(actor, [critic1, critic2], agOpts);
    
    trOpts = rlTrainingOptions( ...
        'MaxEpisodes', N_EPISODES, ...
        'MaxStepsPerEpisode', env.MaxSteps, ...
        'Verbose', false, 'Plots','none');
    
    stats = train(agent, env, trOpts);
    results(k) = mean(stats.EpisodeReward(max(1,end-20):end));
    fprintf('  → Avg last-20 reward: %.4f\n', results(k));
end

[bestVal, bestIdx] = max(results);
fprintf('\n Best config: lr_a=%.0e lr_c=%.0e batch=%d → reward=%.4f\n', ...
        configs(bestIdx).lrActor, configs(bestIdx).lrCritic, ...
        configs(bestIdx).batch, bestVal);

%% helper
function [actor, c1, c2] = buildNetworks(env)
    obsDim = env.getObservationInfo().Dimension(1);
    actDim = 1;
    % (reuse buildCriticNetwork from train script)
    run('train_roarm_td3.m');   % defines helper
    actor  = buildActor(obsDim, actDim);
    c1     = buildCritic(obsDim, actDim, 'c1');
    c2     = buildCritic(obsDim, actDim, 'c2');
end