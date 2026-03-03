%% Setup Environment for Direct KUKA Control
% Run this ONCE when you start MATLAB

clear all; close all; clc;

% Navigate to KUKA_control directory
cd('C:\Users\rasul\Documents\MATLAB\multirobot_painting_control_system\MATLAB\KUKA_control')
% CHANGE THE PATH ABOVE TO YOUR ACTUAL PATH

% Add all source directories to MATLAB path
addpath(genpath('srcs'));

% Verify critical functions are available
required_functions = {
    'loadDHParams'
    'printlog'
    'plotWaypoints'
};

fprintf('=== Checking Required Functions ===\n');
all_found = true;
for i = 1:length(required_functions)
    func_path = which(required_functions{i});
    if isempty(func_path)
        fprintf('✗ Missing: %s\n', required_functions{i});
        all_found = false;
    else
        fprintf('✓ Found: %s\n', required_functions{i});
    end
end

if all_found
    fprintf('\n✓ Environment setup complete!\n');
else
    error('Some required functions are missing. Check your paths.');
end

% Save path for future sessions
savepath;