classdef KukaSprayEnv < rl.env.MATLABEnvironment
% KUKASPRAYENV RL environment for adaptive spray parameter optimization.
%
% State (8-dim):  [ee_velocity, ee_accel, curvature, tracking_error,
%                  spray_distance, torque_mag, progress, prev_speed_cmd]
%
% Action (2-dim): [speed_factor (0.1–1.0), pulse_width_factor (0.5–1.5)]
%   - speed_factor scales the feedrate override (maps to KUKA $OV_PRO)
%   - pulse_width_factor adjusts spray pulse timing
%
% Reward: Encourages ideal paint thickness (50–100 μm), penalizes
%         drips (>150 μm), bare spots (<30 μm), and jerky speed changes.

    properties
        % Dataset-derived trajectory data
        Features        % struct from loadKukaRLDataset
        Labels          % struct from loadKukaRLDataset
        NumSteps        % total timesteps in dataset

        % Current state
        StepIdx = 1
        PrevSpeedCmd = 0.5  % normalized [0,1]

        % Paint accumulation tracking
        ThicknessMap    % accumulated thickness per step
        
        % Reward shaping parameters
        IdealThicknessLow  = 50   % μm
        IdealThicknessHigh = 100  % μm
        DripThreshold      = 150  % μm
        BareThreshold      = 30   % μm
        
        % Normalization constants (from dataset analysis)
        MaxVelocity = 600     % mm/s
        MaxAccel    = 20000   % mm/s^2
        MaxCurvature = 3.14   % rad
        MaxTrackErr = 120     % mm
        MaxSprayDist = 500    % mm
        MaxTorque   = 250     % Nm aggregate
    end

    methods
        function this = KukaSprayEnv(features, labels)
            % Define observation: 8 continuous values in [0, 1]
            ObsInfo = rlNumericSpec([8 1], ...
                'LowerLimit', zeros(8,1), ...
                'UpperLimit', ones(8,1));
            ObsInfo.Name = 'SprayState';
            ObsInfo.Description = 'Normalized [vel, accel, curv, trackErr, sprayDist, torque, progress, prevSpd]';

            % Define action: 2 continuous values
            ActInfo = rlNumericSpec([2 1], ...
                'LowerLimit', [0.1; 0.5], ...
                'UpperLimit', [1.0; 1.5]);
            ActInfo.Name = 'SprayAction';
            ActInfo.Description = '[speed_factor, pulse_width_factor]';

            this = this@rl.env.MATLABEnvironment(ObsInfo, ActInfo);

            % Store dataset
            this.Features = features;
            this.Labels   = labels;
            this.NumSteps = length(features.time_s);
            this.ThicknessMap = zeros(this.NumSteps, 1);
        end

        function [obs, reward, isDone, info] = step(this, action)
            % Clamp actions to valid range
            speed_factor = max(0.1, min(1.0, action(1)));
            pulse_factor = max(0.5, min(1.5, action(2)));

            % Current trajectory state from dataset
            idx = this.StepIdx;
            vel   = this.Features.ee_velocity(idx);
            sdist = this.Features.spray_distance(idx);

            % Compute resulting paint thickness at this step
            % Physics model: thickness ∝ (1/speed) × (1/distance) × pulse_width
            ref_speed = 300; ref_dist = 300; base_depo = 80;
            effective_speed = vel * speed_factor;
            effective_pulse = 12 * pulse_factor;  % base pulse_ms = 12

            speed_ratio = ref_speed / max(effective_speed, 1);
            dist_ratio  = ref_dist / max(sdist, 50);
            pulse_ratio = effective_pulse / 12;

            thickness = base_depo * speed_ratio * dist_ratio * pulse_ratio;
            thickness = min(thickness, 500);
            this.ThicknessMap(idx) = thickness;

            % --- REWARD COMPUTATION ---
            reward = 0;

            % R1: Thickness quality (main objective)
            if thickness >= this.IdealThicknessLow && thickness <= this.IdealThicknessHigh
                reward = reward + 10;  % Ideal range bonus
            elseif thickness > this.IdealThicknessHigh && thickness <= this.DripThreshold
                % Slightly too thick — mild penalty
                overshoot = (thickness - this.IdealThicknessHigh) / ...
                            (this.DripThreshold - this.IdealThicknessHigh);
                reward = reward + 5 * (1 - overshoot);
            elseif thickness > this.DripThreshold
                % Drip zone — heavy penalty
                reward = reward - 20;
            elseif thickness >= this.BareThreshold && thickness < this.IdealThicknessLow
                % Slightly thin
                undershoot = (this.IdealThicknessLow - thickness) / ...
                             (this.IdealThicknessLow - this.BareThreshold);
                reward = reward + 5 * (1 - undershoot);
            else
                % Bare spot — heavy penalty
                reward = reward - 15;
            end

            % R2: Smoothness penalty (avoid jerky speed changes)
            speed_change = abs(speed_factor - this.PrevSpeedCmd);
            reward = reward - 5 * speed_change;

            % R3: Tracking error penalty (high speed → high error)
            track_err = this.Features.tracking_error(idx);
            if track_err > 20  % mm
                reward = reward - 2 * (track_err / this.MaxTrackErr);
            end

            % R4: Energy efficiency bonus (lower current = less wear)
            curr_mag = this.Features.current_magnitude(idx);
            reward = reward + 1 * (1 - min(curr_mag / 20, 1));

            % --- ADVANCE STATE ---
            this.PrevSpeedCmd = speed_factor;
            this.StepIdx = this.StepIdx + 1;

            % Check if episode is done
            isDone = (this.StepIdx >= this.NumSteps);

            % Get next observation
            if ~isDone
                obs = this.getObservation();
            else
                obs = zeros(8, 1);  % Terminal state
                % End-of-episode bonus for overall quality
                avg_thick = mean(this.ThicknessMap(this.ThicknessMap > 0));
                if avg_thick >= 50 && avg_thick <= 100
                    reward = reward + 50;  % Completion bonus
                end
            end

            info = struct('thickness', thickness, 'speed_factor', speed_factor, ...
                          'pulse_factor', pulse_factor, 'tracking_error', track_err);
        end

        function obs = reset(this)
            this.StepIdx = 1;
            this.PrevSpeedCmd = 0.5;
            this.ThicknessMap = zeros(this.NumSteps, 1);
            obs = this.getObservation();
        end

        function obs = getObservation(this)
            idx = this.StepIdx;
            obs = [
                this.Features.ee_velocity(idx)        / this.MaxVelocity;
                this.Features.ee_accel(idx)            / this.MaxAccel;
                this.Features.curvature(idx)           / this.MaxCurvature;
                this.Features.tracking_error(idx)      / this.MaxTrackErr;
                this.Features.spray_distance(idx)      / this.MaxSprayDist;
                this.Features.torque_magnitude(idx)    / this.MaxTorque;
                this.Features.progress(idx);
                this.PrevSpeedCmd
            ];
            % Clamp to [0, 1]
            obs = max(0, min(1, obs));
        end
    end
end