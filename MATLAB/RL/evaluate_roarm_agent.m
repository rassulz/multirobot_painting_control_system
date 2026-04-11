%% evaluate_roarm_agent.m
% Full evaluation with plots matching your existing dashboard style

clear; clc;

%% Load
load('roarm_td3_agent_improved.mat', 'agent');
env = RoarmPaintingEnv();

%% Run evaluation episode
obs     = reset(env);
isDone  = false;
steps   = 0;

% Preallocate logs
maxLen  = env.MaxSteps;
log.time        = zeros(maxLen,1);
log.speedRL     = zeros(maxLen,1);
log.speedBL     = 0.5 * ones(maxLen,1);   % baseline
log.trackErr    = zeros(maxLen,1);
log.reward      = zeros(maxLen,1);
log.curvature   = zeros(maxLen,1);
log.velocity    = zeros(maxLen,1);

t = 0;
while ~isDone
    steps = steps + 1;
    
    % Deterministic action (no exploration noise)
    action = getAction(agent, {obs});
    action = action{1};
    
    [obs, reward, isDone, ~] = step(env, action);
    
    log.time(steps)     = t;
    log.speedRL(steps)  = action;
    log.trackErr(steps) = env.PrevError;   % expose via public getter or property
    log.reward(steps)   = reward;
    log.curvature(steps)= env.PathCurvature(min(env.StepIdx, env.RunEndStep));
    log.velocity(steps) = env.PathVelocity(min(env.StepIdx, env.RunEndStep));
    
    t = t + env.Ts;
end

% Trim
n = steps;
fields = fieldnames(log);
for k = 1:numel(fields), log.(fields{k}) = log.(fields{k})(1:n); end

%% ── Compute metrics ───────────────────────────────────────────────────────
rmse_rl  = rms(log.trackErr);
rmse_bl  = computeBaselineRMSE(env);   % helper below
pct_improve = 100*(rmse_bl - rmse_rl)/rmse_bl;

fprintf('\n══════════════════════════════════════════\n');
fprintf(' Evaluation Results\n');
fprintf('──────────────────────────────────────────\n');
fprintf(' RMSE tracking error (RL)       : %.3f mm\n', rmse_rl);
fprintf(' RMSE tracking error (Baseline) : %.3f mm\n', rmse_bl);
fprintf(' Improvement                    : %.1f %%\n',  pct_improve);
fprintf(' Mean RL speed cmd              : %.4f\n', mean(log.speedRL));
fprintf(' Speed std                      : %.4f\n', std(log.speedRL));
fprintf('══════════════════════════════════════════\n');

%% ── Dashboard plot (mirrors your existing layout) ─────────────────────────
fig = figure('Name','RoArm-M2-S — Adaptive Painting Speed RL Evaluation', ...
             'Position',[50 50 1440 810], 'Color','w');

tl = tiledlayout(2, 3, 'TileSpacing','compact', 'Padding','compact');
title(tl, 'RoArm-M2-S — Adaptive Painting Speed RL Evaluation', ...
      'FontSize',13, 'FontWeight','bold');

% (1) Speed profile
ax1 = nexttile;
hold on;
plot(log.time, log.speedRL,   'r-',  'LineWidth',1.5, 'DisplayName','RL');
plot(log.time, log.speedBL,   'b--', 'LineWidth',1.5, 'DisplayName','Baseline 50%');
% shade dwell zones (low curvature)
dwellMask = log.curvature < 0.1;
fillDwellZones(ax1, log.time, dwellMask);
xlabel('Time (s)'); ylabel('Speed Command');
title('RL Speed Profile vs Baseline');
legend('Location','best'); ylim([0 1.1]); grid on;

% (2) Tracking error
ax2 = nexttile;
plot(log.time, log.trackErr, 'r-', 'LineWidth',1);
hold on;
plot(log.time, movmean(log.trackErr,15), 'k-', 'LineWidth',2);
yline(rmse_bl, 'b--', sprintf('Baseline RMSE=%.2f mm', rmse_bl), ...
      'LineWidth',1.5);
xlabel('Time (s)'); ylabel('mm');
title(sprintf('Tracking Error  (RL RMSE=%.2f mm, Δ=%.1f%%)', rmse_rl, pct_improve));
legend('Per-step','Smoothed','Baseline','Location','best'); grid on;

% (3) Velocity vs RL speed (dual axis)
ax3 = nexttile;
yyaxis left;  
plot(log.time, log.velocity, 'b-', 'LineWidth',0.8);
ylabel('Path Velocity (mm/s)');
yyaxis right; 
plot(log.time, log.speedRL, 'r-', 'LineWidth',1.2);
ylabel('RL Speed Cmd');
xlabel('Time (s)'); title('Velocity ↔ RL Speed'); grid on;

% (4) Per-step reward
ax4 = nexttile;
plot(log.time, log.reward, 'Color',[0.7 0.7 0.7], 'LineWidth',0.5);
hold on;
plot(log.time, movmean(log.reward,15), 'r-', 'LineWidth',2);
xlabel('Time (s)'); ylabel('Reward');
title(sprintf('Per-Step Reward  (mean=%.3f)', mean(log.reward)));
legend('Per-step','Smoothed','Location','best'); grid on;

% (5) Speed vs Curvature (scatter coloured by error)
ax5 = nexttile;
scatter(log.curvature, log.speedRL, 30, log.trackErr, 'filled');
colormap(ax5, hot); cb = colorbar; cb.Label.String = 'Track Error (mm)';
xlabel('Curvature (rad/mm)'); ylabel('RL Speed Cmd');
title('Speed vs Curvature  (colour = error)'); grid on;

% (6) Speed distribution
ax6 = nexttile;
histogram(log.speedRL,   20, 'FaceColor','r', 'FaceAlpha',0.6, ...
          'DisplayName','RL');
hold on;
histogram(log.speedBL,   5,  'FaceColor','b', 'FaceAlpha',0.4, ...
          'DisplayName','Baseline');
xline(0.5,'b--','LineWidth',2);
xlabel('Speed Command'); ylabel('Count');
title('RL Speed Distribution'); legend; grid on;

exportgraphics(fig, 'eval_dashboard_improved.png', 'Resolution',150);
fprintf('Dashboard saved to eval_dashboard_improved.png\n');

%% ── Helper functions ──────────────────────────────────────────────────────
function rmse = computeBaselineRMSE(env)
    obs = reset(env);
    isDone = false; errs = [];
    while ~isDone
        [obs, ~, isDone, ~] = step(env, env.SpeedBaseline);
        errs(end+1) = env.PrevError; %#ok<AGROW>
    end
    rmse = rms(errs);
end

function fillDwellZones(ax, t, mask)
    % Shade regions where mask is true (dwell / straight zones)
    yl = get(ax,'YLim');
    inDwell = false; t0 = 0;
    for k = 1:numel(t)
        if mask(k) && ~inDwell
            t0 = t(k); inDwell = true;
        elseif ~mask(k) && inDwell
            patch(ax, [t0 t(k) t(k) t0], [yl(1) yl(1) yl(2) yl(2)], ...
                  'c', 'FaceAlpha',0.15, 'EdgeColor','none', ...
                  'DisplayName','Dwell');
            inDwell = false;
        end
    end
end