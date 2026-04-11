%% analyse_reward.m
% Sweep reward weights and preview expected reward landscape
% Run this BEFORE training to ensure reward is well-conditioned

clear; clc;

% Simulate a range of (trackErr, speed, curvature) combinations
errRange   = linspace(0, 6, 50);       % [mm]
speedRange = linspace(0.1, 1.0, 50);
curvRange  = [0, pi/4, pi/2, pi];

% Default weights
W_track = 2.0;  W_smooth = 0.5;  W_speed = 0.3;  W_curv = 0.4;
errThresh = 1.5;

fprintf('Reward landscape analysis\n');
fprintf('─────────────────────────────────────────────\n');

figure('Name','Reward Landscape','Position',[100 100 1200 500]);
tl = tiledlayout(1, numel(curvRange), 'TileSpacing','compact');
title(tl,'Reward(trackErr, speed) at different curvatures','FontSize',12);

for ci = 1:numel(curvRange)
    curv = curvRange(ci);
    R = zeros(numel(errRange), numel(speedRange));
    for ei = 1:numel(errRange)
        for si = 1:numel(speedRange)
            err = errRange(ei);
            spd = speedRange(si);
            errNorm    = err / errThresh;
            r_track    = -W_track * errNorm^2;
            r_speed    = W_speed * spd * max(0, 1 - errNorm);
            curvNorm   = min(1, curv/pi);
            idealSpeed = 1.0*(1-curvNorm) + 0.1*curvNorm;
            r_curv     = -W_curv * (spd - idealSpeed)^2;
            R(ei,si)   = r_track + r_speed + r_curv;
        end
    end
    
    nexttile;
    imagesc(speedRange, errRange, R);
    colorbar; axis xy;
    xlabel('Speed Cmd'); ylabel('Track Error (mm)');
    title(sprintf('κ = %.2f rad', curv));
    clim([-5 1]);
    
    % Print optimal speed at err=0
    [~, si_opt] = max(R(1,:));
    fprintf('κ=%.2f → optimal speed at zero error: %.2f\n', ...
            curv, speedRange(si_opt));
end

colormap(hot);
fprintf('\nIf the optimal speed decreases with curvature → reward is well-shaped.\n');
fprintf('If all optima cluster at speed=1.0 → reduce W_speed or increase W_curv.\n');