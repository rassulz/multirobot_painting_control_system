%% EVALUATESPRAYAGENT - Evaluate trained RL agent and compare with baseline
%
%  Runs the trained PPO agent through the KUKA dataset trajectory
%  and compares adaptive spray parameters vs. fixed parameters.
%
%  Usage:
%    >> evaluateSprayAgent

clear; clc; close all;
fprintf('=== Evaluating Spray RL Agent ===\n\n');

%% 1. Load dataset and agent
[data, features, labels] = loadKukaRLDataset('kuka_rl_dataset.csv');

agentPath = fullfile(pwd, 'trained_agents', 'spray_ppo_agent.mat');
if ~isfile(agentPath)
    error('Trained agent not found at %s. Run trainSprayAgent first.', agentPath);
end
loaded = load(agentPath);
agent = loaded.agent;
fprintf('Loaded trained agent from: %s\n', agentPath);

%% 2. Create environment and run evaluation
env = KukaSprayEnv(features, labels);
N = length(features.time_s);

% Storage for RL results
rl_thickness    = zeros(N-1, 1);
rl_speed_factor = zeros(N-1, 1);
rl_pulse_factor = zeros(N-1, 1);
rl_rewards      = zeros(N-1, 1);

% Storage for baseline (fixed parameters) results
base_thickness = zeros(N-1, 1);

% Run RL agent
obs = reset(env);
for i = 1:(N-1)
    action = getAction(agent, obs);
    action = action{1};  % extract from cell

    [obs, reward, isDone, info] = step(env, action);

    rl_thickness(i)    = info.thickness;
    rl_speed_factor(i) = info.speed_factor;
    rl_pulse_factor(i) = info.pulse_factor;
    rl_rewards(i)      = reward;

    % Baseline: fixed speed_factor=0.5, pulse_factor=1.0
    vel = features.ee_velocity(i);
    sdist = features.spray_distance(i);
    eff_speed = vel * 0.5;
    base_thickness(i) = 80 * (300/max(eff_speed,1)) * (300/max(sdist,50)) * 1.0;
    base_thickness(i) = min(base_thickness(i), 500);

    if isDone, break; end
end

time_eval = features.time_s(1:length(rl_thickness));

%% 3. Compute quality metrics
fprintf('\n--- Quality Comparison ---\n');
fprintf('%-30s %-15s %-15s\n', 'Metric', 'Baseline', 'RL Agent');
fprintf('%s\n', repmat('-', 1, 60));

% Mean thickness
fprintf('%-30s %-15.1f %-15.1f\n', 'Mean thickness (μm)', ...
    mean(base_thickness), mean(rl_thickness));

% Std deviation
fprintf('%-30s %-15.1f %-15.1f\n', 'Std dev thickness (μm)', ...
    std(base_thickness), std(rl_thickness));

% In-spec percentage (50–100 μm)
base_inspec = sum(base_thickness >= 50 & base_thickness <= 100) / length(base_thickness) * 100;
rl_inspec   = sum(rl_thickness >= 50 & rl_thickness <= 100) / length(rl_thickness) * 100;
fprintf('%-30s %-15.1f%% %-15.1f%%\n', 'In-spec (50-100 μm)', base_inspec, rl_inspec);

% Drip percentage (>150 μm)
base_drip = sum(base_thickness > 150) / length(base_thickness) * 100;
rl_drip   = sum(rl_thickness > 150) / length(rl_thickness) * 100;
fprintf('%-30s %-15.1f%% %-15.1f%%\n', 'Drip zones (>150 μm)', base_drip, rl_drip);

% Bare spots (<30 μm)
base_bare = sum(base_thickness < 30) / length(base_thickness) * 100;
rl_bare   = sum(rl_thickness < 30) / length(rl_thickness) * 100;
fprintf('%-30s %-15.1f%% %-15.1f%%\n', 'Bare spots (<30 μm)', base_bare, rl_bare);

% Total reward
fprintf('%-30s %-15s %-15.1f\n', 'Total episode reward', 'N/A', sum(rl_rewards));

%% 4. Visualization
figure('Name', 'RL vs Baseline Spray Comparison', 'Position', [50 50 1400 900]);

% Paint thickness comparison
subplot(3,2,1);
plot(time_eval, base_thickness, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Baseline (fixed)');
hold on;
plot(time_eval, rl_thickness, 'r-', 'LineWidth', 1.5, 'DisplayName', 'RL Agent');
yline(50, 'g--', 'LineWidth', 1); yline(100, 'g--', 'LineWidth', 1);
yline(150, 'k--', 'Drip limit', 'LineWidth', 1);
fill([time_eval(1) time_eval(end) time_eval(end) time_eval(1)], ...
     [50 50 100 100], 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
xlabel('Time (s)'); ylabel('Thickness (μm)');
title('Paint Thickness: RL vs Baseline'); legend('Location', 'best'); grid on;

% RL speed factor over time
subplot(3,2,2);
plot(time_eval, rl_speed_factor, 'Color', [0.8 0.2 0], 'LineWidth', 1.2);
hold on;
yline(0.5, 'b--', 'Baseline speed', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Speed Factor');
title('RL Adaptive Speed Factor'); grid on;
ylim([0 1.1]);

% RL pulse width factor
subplot(3,2,3);
plot(time_eval, rl_pulse_factor, 'Color', [0 0.6 0.3], 'LineWidth', 1.2);
hold on;
yline(1.0, 'b--', 'Baseline pulse', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Pulse Factor');
title('RL Adaptive Pulse Width Factor'); grid on;

% Reward per step
subplot(3,2,4);
plot(time_eval, rl_rewards, 'k-', 'LineWidth', 1);
hold on;
plot(time_eval, movmean(rl_rewards, 20), 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Reward');
title('Per-Step Reward'); legend('Raw', 'Moving avg (20)'); grid on;

% Speed factor vs trajectory features
subplot(3,2,5);
yyaxis left;
plot(time_eval, features.ee_velocity(1:length(time_eval)), 'b-', 'LineWidth', 1);
ylabel('EE Velocity (mm/s)');
yyaxis right;
plot(time_eval, rl_speed_factor, 'r-', 'LineWidth', 1.2);
ylabel('Speed Factor');
xlabel('Time (s)');
title('Velocity ↔ Speed Factor Correlation'); grid on;

% Thickness distribution histogram
subplot(3,2,6);
histogram(base_thickness, 30, 'FaceColor', 'b', 'FaceAlpha', 0.5, 'DisplayName', 'Baseline');
hold on;
histogram(rl_thickness, 30, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'DisplayName', 'RL Agent');
xline(50, 'g--', 'LineWidth', 2); xline(100, 'g--', 'LineWidth', 2);
xlabel('Thickness (μm)'); ylabel('Count');
title('Thickness Distribution'); legend('Location', 'best'); grid on;

sgtitle('Adaptive Spray Parameters — RL Agent Evaluation', 'FontSize', 14, 'FontWeight', 'bold');

%% 5. Export results to CSV
results = table(time_eval, rl_speed_factor, rl_pulse_factor, ...
    rl_thickness, base_thickness, rl_rewards, ...
    'VariableNames', {'Time_s', 'RL_SpeedFactor', 'RL_PulseFactor', ...
                      'RL_Thickness_um', 'Baseline_Thickness_um', 'Reward'});
writetable(results, 'spray_rl_results.csv');
fprintf('\nResults exported to spray_rl_results.csv\n');