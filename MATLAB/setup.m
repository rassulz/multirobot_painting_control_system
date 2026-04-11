%% Setup Environment for Multirobot Adaptive Control
%  Run this ONCE when you start MATLAB.
%  Auto-detects paths — no hardcoded directories.

clear all; close all; clc;

% --- Auto-detect MATLAB/ root from this script's location ---
thisScript = mfilename('fullpath');
if isempty(thisScript)
    matlabDir = pwd;
else
    matlabDir = fileparts(thisScript);
end

fprintf('=== Multirobot Adaptive Control — Setup ===\n');
fprintf('MATLAB dir: %s\n\n', matlabDir);

% --- Add all subsystem directories ---
dirsToAdd = {
    fullfile(matlabDir, 'KUKA_control'),
    fullfile(matlabDir, 'KUKA_control', 'srcs'),
    fullfile(matlabDir, 'Roarm_control'),
    fullfile(matlabDir, 'RL'),
};

for i = 1:numel(dirsToAdd)
    if isfolder(dirsToAdd{i})
        addpath(genpath(dirsToAdd{i}));
        fprintf('  + %s\n', dirsToAdd{i});
    else
        fprintf('  x Not found: %s\n', dirsToAdd{i});
    end
end
fprintf('\n');

% --- Verify critical functions ---
required_functions = {
    'run_full_multirobot_pipeline'       % Unified pipeline
    'create_position_only_rl_dataset'    % Dataset builder
    'collect_collision_dataset'          % Collision data
    'train_collision_agent'              % Collision RL
    'loadDHParams'                       % KUKA kinematics
    'printlog'                           % KUKA logging
    'plotWaypoints'                      % KUKA visualization
};

fprintf('=== Checking Functions ===\n');
all_found = true;
for i = 1:length(required_functions)
    fp = which(required_functions{i});
    if isempty(fp)
        fprintf('  x Missing: %s\n', required_functions{i});
        all_found = false;
    else
        fprintf('  + Found:   %s\n', required_functions{i});
    end
end

fprintf('\n');
if all_found
    fprintf('Setup complete!\n');
else
    fprintf('Some functions missing — pipeline may still work for available phases.\n');
end
fprintf('\nNext step:\n');
fprintf('  >> run_full_multirobot_pipeline()        %% full synthetic\n');
fprintf('  >> run_full_multirobot_pipeline(cfg)      %% custom config\n\n');

try savepath; catch, end