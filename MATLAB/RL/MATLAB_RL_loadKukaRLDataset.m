function [data, features, labels] = loadKukaRLDataset(csvPath)
% LOADKUKARLDATASET Load and preprocess the KUKA RL dataset
%
%   Inputs:
%       csvPath - path to kuka_rl_dataset.csv (string)
%   Outputs:
%       data     - raw table from CSV
%       features - struct with engineered features per timestep
%       labels   - struct with derived quality metrics

    %% 1. Load raw data
    if nargin < 1 || isempty(csvPath)
        csvPath = fullfile('C:','Users','SYNAPTICON','Desktop', ...
            'multirobot_painting_control_system','MATLAB','RL', ...
            'kuka_rl_dataset.csv');
    end
    data = readtable(csvPath, 'VariableNamingRule', 'preserve');
    N = height(data);
    fprintf('Loaded %d timesteps from %s\n', N, csvPath);

    %% 2. Extract core signals
    target_xyz = [data.target_x, data.target_y, data.target_z];
    meas_xyz   = [data.meas_x,   data.meas_y,   data.meas_z];
    time_s     = data.time_s;
    dt_val     = data.dt(1);

    %% 3. Tracking error
    tracking_error = vecnorm(target_xyz - meas_xyz, 2, 2);

    %% 4. End-effector velocity
    ee_velocity = zeros(N, 1);
    for i = 2:N
        ee_velocity(i) = norm(target_xyz(i,:) - target_xyz(i-1,:)) / dt_val;
    end
    ee_velocity(1) = ee_velocity(2);

    %% 5. End-effector acceleration
    ee_accel = zeros(N, 1);
    for i = 2:N
        ee_accel(i) = (ee_velocity(i) - ee_velocity(i-1)) / dt_val;
    end
    ee_accel(1) = ee_accel(2);

    %% 6. Path curvature
    curvature = zeros(N, 1);
    for i = 3:N
        v1 = target_xyz(i-1,:) - target_xyz(i-2,:);
        v2 = target_xyz(i,:)   - target_xyz(i-1,:);
        n1 = norm(v1); n2 = norm(v2);
        if n1 > 1e-6 && n2 > 1e-6
            cos_angle = dot(v1, v2) / (n1 * n2);
            cos_angle = max(-1, min(1, cos_angle));
            curvature(i) = acos(cos_angle);
        end
    end

    %% 7-9. Joint magnitudes
    torques = [data.torque_j1, data.torque_j2, data.torque_j3, ...
               data.torque_j4, data.torque_j5, data.torque_j6];
    torque_magnitude = vecnorm(torques, 2, 2);

    velocities = [data.velocity_j1, data.velocity_j2, data.velocity_j3, ...
                  data.velocity_j4, data.velocity_j5, data.velocity_j6];
    velocity_magnitude = vecnorm(velocities, 2, 2);

    currents = [data.current_j1, data.current_j2, data.current_j3, ...
                data.current_j4, data.current_j5, data.current_j6];
    current_magnitude = vecnorm(currents, 2, 2);

    %% 10. Distance to pick/place
    pick_pt  = [data.pick_x(1), data.pick_y(1), data.pick_z(1)];
    place_pt = [data.place_x(1), data.place_y(1), data.place_z(1)];
    dist_to_pick  = vecnorm(target_xyz - pick_pt, 2, 2);
    dist_to_place = vecnorm(target_xyz - place_pt, 2, 2);

    %% 11. Progress along trajectory
    cumulative_path = zeros(N, 1);
    for i = 2:N
        cumulative_path(i) = cumulative_path(i-1) + ...
            norm(target_xyz(i,:) - target_xyz(i-1,:));
    end
    progress = cumulative_path / max(cumulative_path(end), 1e-6);

    %% 12. Spray distance
    surface_z = place_pt(3);
    spray_distance = abs(target_xyz(:,3) - surface_z);

    %% Package outputs
    features.tracking_error     = tracking_error;
    features.ee_velocity        = ee_velocity;
    features.ee_accel           = ee_accel;
    features.curvature          = curvature;
    features.torque_magnitude   = torque_magnitude;
    features.velocity_magnitude = velocity_magnitude;
    features.current_magnitude  = current_magnitude;
    features.dist_to_pick       = dist_to_pick;
    features.dist_to_place      = dist_to_place;
    features.progress           = progress;
    features.spray_distance     = spray_distance;
    features.target_xyz         = target_xyz;
    features.meas_xyz           = meas_xyz;
    features.time_s             = time_s;

    labels.estimated_thickness = computeSimulatedThickness(ee_velocity, spray_distance);
    labels.uniformity_score    = computeUniformityScore(tracking_error, curvature);

    fprintf('Feature engineering complete. %d features extracted.\n', 11);
end

function thickness = computeSimulatedThickness(velocity, spray_dist)
    base_deposition = 80;
    ref_speed = 300;
    ref_dist = 300;
    speed_factor = ref_speed ./ max(velocity, 1);
    dist_factor  = ref_dist ./ max(spray_dist, 50);
    thickness = base_deposition * speed_factor .* dist_factor;
    thickness = min(thickness, 500);
end

function score = computeUniformityScore(tracking_error, curvature)
    error_penalty = tracking_error / max(tracking_error);
    curve_penalty = curvature / max(max(curvature), 1e-6);
    score = max(0, 1 - 0.5*error_penalty - 0.3*curve_penalty);
end