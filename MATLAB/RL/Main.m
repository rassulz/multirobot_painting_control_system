%% =========================================================================
%  main_rl_pipeline.m  —  Master RL Orchestrator
% ==========================================================================
%  Multi-Robot Painting Control System — RL Pipeline Runner
%
%  This script runs the ENTIRE RL pipeline step-by-step:
%
%    PHASE 1: Environment setup & toolbox checks
%    PHASE 2: KUKA KR10 pick-and-place RL (runKukaPickPlaceRL)
%    PHASE 3: RoArm-M2-S adaptive painting RL (runRoarmPaintingRL)
%    PHASE 4: Combined multi-robot dashboard
%    PHASE 5: (Optional) Live hardware deployment
%
%  Required folder structure:
%    MATLAB/RL/
%    ├── main_rl_pipeline.m               ← THIS FILE
%    ├── runKukaPickPlaceRL.m             ← KUKA RL trainer
%    ├── runRoarmPaintingRL.m             ← RoArm RL trainer
%    ├── KukaPickPlaceEnv.m              ← KUKA RL environment class
%    ├── RoarmPaintingEnv.m              ← RoArm RL environment class
%    ├── kuka_rl_dataset.csv             ← KUKA pick-and-place data
%    └── roarm_position_rl_dataset.csv   ← RoArm painting data
%
%  Requirements:
%    MATLAB R2022a+, Reinforcement Learning Toolbox, Deep Learning Toolbox
%
%  Usage:
%    >> main_rl_pipeline              % Run everything
%    >> main_rl_pipeline('kuka')      % Run KUKA pipeline only
%    >> main_rl_pipeline('roarm')     % Run RoArm pipeline only
%    >> main_rl_pipeline('deploy')    % Skip training, deploy saved agents
% ==========================================================================

function main_rl_pipeline(mode)

if nargin < 1, mode = 'all'; end
mode = lower(mode);

tStart = tic;
% NOTE: Do NOT use 'clear' here — it would wipe the 'mode' argument.
%       Use 'close all; clc;' only.
close all; clc;

fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║     MULTI-ROBOT PAINTING SYSTEM — RL MASTER PIPELINE       ║\n');
fprintf('║                                                             ║\n');
fprintf('║  KUKA KR10 R1100-2  →  Pick & Place velocity optimisation  ║\n');
fprintf('║  RoArm-M2-S (×2)   →  Adaptive painting speed control     ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');
fprintf('  Mode:  %s\n', upper(mode));
fprintf('  Time:  %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

%% ═════���═════════════════════════════════════════════════════════════════
%  PHASE 1: ENVIRONMENT SETUP & VALIDATION
% ════════════════════════════════════════════════════════════════════════
printPhase(1, 'ENVIRONMENT SETUP & VALIDATION');

% ── 1.1 Resolve paths ────────────────────────────────────────────────
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end
fprintf('  [1.1] Script directory: %s\n', scriptDir);

% Add RL folder and parent folders to MATLAB path
addpath(scriptDir);
parentDir = fileparts(scriptDir);
addpath(fullfile(parentDir, 'KUKA_control'));
addpath(fullfile(parentDir, 'Roarm_control'));
fprintf('  [1.1] Added RL, KUKA_control, Roarm_control to MATLAB path.\n');

% ── 1.2 Check required toolboxes ─────────────────────────────────────
fprintf('  [1.2] Checking MATLAB toolboxes …\n');
requiredToolboxes = {
    'Reinforcement Learning Toolbox',   'rl';
    'Deep Learning Toolbox',            'nnet';
};
allToolboxesPresent = true;
installedToolboxes = ver;
installedNames = {installedToolboxes.Name};

for i = 1:size(requiredToolboxes, 1)
    tbName = requiredToolboxes{i, 1};
    found = any(contains(installedNames, tbName, 'IgnoreCase', true));
    if found
        fprintf('        ✓ %s\n', tbName);
    else
        fprintf('        ✗ %s — MISSING!\n', tbName);
        allToolboxesPresent = false;
    end
end

if ~allToolboxesPresent
    error(['main_rl_pipeline:missingToolbox', ...
        '\n  One or more required toolboxes are missing.', ...
        '\n  Install them via Add-Ons → Get Add-Ons in MATLAB.']);
end
fprintf('        All toolboxes present.\n');

% ── 1.3 Check dataset files ──────────────────────────────────────────
fprintf('  [1.3] Checking dataset files …\n');
kukaCSV  = fullfile(scriptDir, 'kuka_position_rl_dataset.csv');
roarmCSV = fullfile(scriptDir, 'roarm_position_rl_dataset.csv');

datasets = {kukaCSV, 'KUKA pick-and-place'; roarmCSV, 'RoArm painting'};
for i = 1:size(datasets, 1)
    if isfile(datasets{i, 1})
        info = dir(datasets{i, 1});
        fprintf('        ✓ %s dataset (%.1f KB)\n', datasets{i, 2}, info.bytes/1024);
    else
        fprintf('        ✗ %s dataset — NOT FOUND at:\n          %s\n', ...
            datasets{i, 2}, datasets{i, 1});
    end
end

% ── 1.4 Check environment & agent class files ────────────────────────
fprintf('  [1.4] Checking RL class files …\n');
classFiles = {
    'KukaPickPlaceEnv.m',   'KUKA RL environment';
    'RoarmPaintingEnv.m',   'RoArm RL environment';
    'runKukaPickPlaceRL.m', 'KUKA training script';
    'runRoarmPaintingRL.m', 'RoArm training script';
};
for i = 1:size(classFiles, 1)
    fPath = fullfile(scriptDir, classFiles{i, 1});
    if isfile(fPath)
        fprintf('        ✓ %s\n', classFiles{i, 2});
    else
        fprintf('        ✗ %s — NOT FOUND (%s)\n', classFiles{i, 2}, classFiles{i, 1});
    end
end

% ── 1.5 Create output directories ────────────────────────────────────
fprintf('  [1.5] Creating output directories …\n');
savedAgentsDir = fullfile(scriptDir, 'savedAgents');
resultsDir     = fullfile(scriptDir, 'results');
figuresDir     = fullfile(scriptDir, 'figures');

dirs = {savedAgentsDir, resultsDir, figuresDir};
for i = 1:length(dirs)
    if ~isfolder(dirs{i})
        mkdir(dirs{i});
        fprintf('        Created: %s\n', dirs{i});
    else
        fprintf('        Exists:  %s\n', dirs{i});
    end
end

fprintf('\n  Phase 1 complete.\n\n');

%% ═══════════════════════════════════════════════════════════════════════
%  PHASE 2: KUKA KR10 PICK-AND-PLACE RL
% ════════════════════════════════════════════════════════════════════════
if any(strcmp(mode, {'all', 'kuka'}))
    printPhase(2, 'KUKA KR10 PICK-AND-PLACE RL');
    try
        kukaResults = runKukaPickPlaceRL_safe(scriptDir, savedAgentsDir, ...
                                              resultsDir, figuresDir);
        fprintf('\n  ✓ KUKA pipeline completed successfully.\n');
        fprintf('    Mean reward (last 20 ep): %.4f\n', kukaResults.meanReward);
    catch ME
        fprintf('\n  ✗ KUKA RL pipeline FAILED:\n');
        fprintf('    %s\n', ME.message);
        if ~isempty(ME.stack)
            fprintf('    Line %d in %s\n', ME.stack(1).line, ME.stack(1).name);
        end
        kukaResults = struct('meanReward', NaN, 'agent', []);
    end
else
    fprintf('  [Skipped] KUKA pipeline (mode = %s)\n', mode);
    kukaResults = struct('meanReward', NaN, 'agent', []);
end

%% ═══════════════════════════════════════════════════════════════════════
%  PHASE 3: ROARM-M2-S ADAPTIVE PAINTING SPEED RL
% ════════════════════════════════════════════════════════════════════════
if any(strcmp(mode, {'all', 'roarm'}))
    printPhase(3, 'ROARM-M2-S ADAPTIVE PAINTING SPEED RL');
    try
        roarmResults = runRoarmPaintingRL_safe(scriptDir, savedAgentsDir, ...
                                               resultsDir, figuresDir);
        fprintf('\n  ✓ RoArm pipeline completed successfully.\n');
        fprintf('    Mean reward (last 20 ep): %.4f\n', roarmResults.meanReward);
    catch ME
        fprintf('\n  ✗ RoArm RL pipeline FAILED:\n');
        fprintf('    %s\n', ME.message);
        if ~isempty(ME.stack)
            fprintf('    Line %d in %s\n', ME.stack(1).line, ME.stack(1).name);
        end
        roarmResults = struct('meanReward', NaN, 'agent', []);
    end
else
    fprintf('  [Skipped] RoArm pipeline (mode = %s)\n', mode);
    roarmResults = struct('meanReward', NaN, 'agent', []);
end

%% ═══════════════════════════════════════════════════════════════════════
%  PHASE 4: COMBINED MULTI-ROBOT RESULTS DASHBOARD
% ════════════════════════════════════════════════════════════════════════
if any(strcmp(mode, {'all', 'kuka', 'roarm'}))
    printPhase(4, 'COMBINED MULTI-ROBOT RESULTS DASHBOARD');
    try
        buildCombinedDashboard(kukaResults, roarmResults, figuresDir);
        fprintf('\n  ✓ Dashboard generated.\n');
    catch ME
        fprintf('\n  ⚠ Dashboard generation failed: %s\n', ME.message);
    end
else
    printPhase(4, 'COMBINED MULTI-ROBOT RESULTS DASHBOARD');
    fprintf('  ⚠ No results files found — skipping dashboard.\n');
    fprintf('    Run pipeline with mode ''all'', ''kuka'', or ''roarm'' first.\n');
end

%% ═══════════════════════════════════════════════════════════════════════
%  PHASE 5: LIVE HARDWARE DEPLOYMENT (OPTIONAL)
% ════════════════════════════════════════════════════════════════════════
if any(strcmp(mode, {'all', 'deploy'}))
    printPhase(5, 'LIVE HARDWARE DEPLOYMENT (OPTIONAL)');

    agentFiles = dir(fullfile(savedAgentsDir, '*.mat'));
    if isempty(agentFiles)
        fprintf('  ⚠ No saved agents found in: %s\n', savedAgentsDir);
        fprintf('    Run training first (mode ''all'', ''kuka'', or ''roarm'').\n');
    else
        fprintf('  Found %d saved agent file(s):\n', numel(agentFiles));
        for i = 1:numel(agentFiles)
            fprintf('    • %s\n', agentFiles(i).name);
        end
        fprintf('\n  To deploy, call:\n');
        fprintf('    deployAgent(''%s/<agentFile>.mat'', ''<robotType>'')\n', savedAgentsDir);
    end
end

%% ═══════════════���═══════════════════════════════════════════════════════
%  PIPELINE SUMMARY
% ════════════════════════════════════════════════════════════════════════
elapsed = toc(tStart);
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                    PIPELINE COMPLETE                        ║\n');
fprintf('╠══════════════════════════════════════════════════════════════╣\n');
fprintf('║  Total elapsed time : %.1f seconds                        ║\n', elapsed);
fprintf('║  Mode               : %-37s║\n', upper(mode));
fprintf('║  Results directory  : …%s ║\n', shortenPath(resultsDir, 35));
fprintf('║  Figures directory  : …%s ║\n', shortenPath(figuresDir, 35));
fprintf('║  Agents directory   : …%s ║\n', shortenPath(savedAgentsDir, 35));
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

end  % main_rl_pipeline


%% ═══════════════════════════════════════════════════════════════════════
%  HELPER FUNCTIONS
% ════════════════════════════════════════════════════════════════════════

function printPhase(n, name)
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║  PHASE %d: %-50s║\n', n, name);
    fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');
end

function s = shortenPath(p, maxLen)
    if length(p) > maxLen
        s = p(end-maxLen+1:end);
    else
        s = pad(p, maxLen);
    end
end

%% ─── Safe wrapper for KUKA RL (prevents workspace clearing) ─────────
function results = runKukaPickPlaceRL_safe(scriptDir, savedAgentsDir, ...
                                           resultsDir, figuresDir)
    % This function wraps the KUKA pipeline so that 'clear' inside the
    % script does not destroy the caller's workspace.
    kukaCSV = fullfile(scriptDir, 'kuka_position_rl_dataset.csv');
    if ~isfile(kukaCSV)
        error('KUKA dataset not found: %s', kukaCSV);
    end

    % Call the KUKA training script inside a function scope
    fprintf('  Running KUKA RL pipeline …\n');
    origDir = cd(scriptDir);
    cleanupObj = onCleanup(@() cd(origDir));

    % Run the script — its 'clear' only affects this function's workspace
    runKukaPickPlaceRL;

    % Collect results if training produced an agent
    results.meanReward = NaN;
    results.agent = [];
    agentFile = fullfile(savedAgentsDir, 'kuka_td3_agent.mat');
    if isfile(agentFile)
        S = load(agentFile);
        if isfield(S, 'agent'), results.agent = S.agent; end
    end
    % Try to read reward from workspace if the script left variables
    if exist('trainStats', 'var')
        results.meanReward = mean(trainStats.EpisodeReward(max(1,end-20):end));
    end
end

%% ─── Safe wrapper for RoArm RL (prevents workspace clearing) ────────
function results = runRoarmPaintingRL_safe(scriptDir, savedAgentsDir, ...
                                           resultsDir, figuresDir)
    roarmCSV = fullfile(scriptDir, 'roarm_position_rl_dataset.csv');
    if ~isfile(roarmCSV)
        error('RoArm dataset not found: %s', roarmCSV);
    end

    fprintf('  Running RoArm RL pipeline …\n');
    origDir = cd(scriptDir);
    cleanupObj = onCleanup(@() cd(origDir));

    % Run inside function scope — 'clear' only wipes this scope
    runRoarmPaintingRL;

    results.meanReward = NaN;
    results.agent = [];
    agentFile = fullfile(savedAgentsDir, 'roarm_td3_agent_improved.mat');
    if isfile(agentFile)
        S = load(agentFile);
        if isfield(S, 'agent'), results.agent = S.agent; end
    end
    if exist('trainStats', 'var')
        results.meanReward = mean(trainStats.EpisodeReward(max(1,end-20):end));
    end
end

%% ─── Combined dashboard ─────────────────────────────────────────────
function buildCombinedDashboard(kukaResults, roarmResults, figuresDir)
    fig = figure('Name', 'Multi-Robot RL Dashboard', ...
                 'Position', [50 50 1200 600], 'Color', 'w');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, 'Multi-Robot Painting System — RL Results', ...
          'FontSize', 14, 'FontWeight', 'bold');

    % KUKA tile
    nexttile;
    if ~isnan(kukaResults.meanReward)
        bar(1, kukaResults.meanReward, 'FaceColor', [0.2 0.6 0.9]);
        ylabel('Mean Reward (last 20 episodes)');
        title('KUKA KR10 — Pick & Place');
        set(gca, 'XTickLabel', {'KUKA'});
        grid on;
    else
        text(0.5, 0.5, {'KUKA results', 'not available'}, ...
             'HorizontalAlignment', 'center', 'FontSize', 14);
        axis off;
        title('KUKA KR10 — Pick & Place');
    end

    % RoArm tile
    nexttile;
    if ~isnan(roarmResults.meanReward)
        bar(1, roarmResults.meanReward, 'FaceColor', [0.9 0.3 0.3]);
        ylabel('Mean Reward (last 20 episodes)');
        title('RoArm-M2-S — Adaptive Speed');
        set(gca, 'XTickLabel', {'RoArm'});
        grid on;
    else
        text(0.5, 0.5, {'RoArm results', 'not available'}, ...
             'HorizontalAlignment', 'center', 'FontSize', 14);
        axis off;
        title('RoArm-M2-S — Adaptive Speed');
    end

    outFile = fullfile(figuresDir, 'combined_dashboard.png');
    exportgraphics(fig, outFile, 'Resolution', 150);
    fprintf('    Dashboard saved to: %s\n', outFile);
end