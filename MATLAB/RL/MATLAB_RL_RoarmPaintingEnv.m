classdef RoarmPaintingEnv < rl.env.MATLABEnvironment
%  RoarmPaintingEnv  —  RL environment for RoArm-M2-S painting speed control
%
%  The RoArm-M2-S performs painting strokes. This environment lets an RL
%  agent choose the SPEED for each waypoint along the stroke to minimise
%  tracking error while maximising throughput.
%
%  State (7×1, normalised 0–1):
%    [ee_velocity_norm, |ee_accel_norm|, curvature_norm, tracking_error_norm,
%     progress, is_dwell, prev_speed]
%
%  Action (1×1, continuous):
%    speed_cmd ∈ [0.1, 1.0]  — scales the RoArm 'spd' parameter (0–100)
%
%  Reward: encourages low tracking error, smooth speed, fast completion,
%          and slow careful motion during dwell/hold phases.

    properties
        F           % Feature struct from dataset
        NM          % Normalisation ceilings
        NumSteps    % Total timesteps in this episode
        StepIdx     % Current step index
        PrevSpeed   % Previous speed command (for smoothness penalty)
        CumTrackErr % Accumulated tracking error
        CumJerk     % Accumulated jerk
    end

    methods
        function this = RoarmPaintingEnv(features, normStruct)
            % Define observation and action spaces
            obsInfo = rlNumericSpec([7 1], ...
                'LowerLimit', zeros(7,1), ...
                'UpperLimit', ones(7,1));
            obsInfo.Name = 'state';

            actInfo = rlNumericSpec([1 1], ...
                'LowerLimit', 0.1, ...
                'UpperLimit', 1.0);
            actInfo.Name = 'speed_cmd';

            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);
            this.F  = features;
            this.NM = normStruct;
            this.NumSteps = length(features.ee_velocity);
        end

        function [obs, reward, isDone, info] = step(this, action)
            spd = max(0.1, min(1.0, action(1)));
            i   = this.StepIdx;

            % ── Reward Components ──

            % R1: Tracking error penalty (most important for paint quality)
            %     Lower error = better paint coverage
            te = this.F.tracking_error(i);
            r_track = -0.45 * (te / max(this.NM.trkErr, 1e-6));

            % R2: Cycle time reward — faster completion is better
            %     Higher speed = less time per step
            r_time = 0.25 * (spd - 0.5);

            % R3: Jerk penalty — smoother motion = more uniform paint
            j = abs(this.F.jerk(i));
            r_jerk = -0.10 * (j / max(this.NM.jerk, 1e-6));

            % R4: Speed smoothness — penalise abrupt speed changes
            r_smooth = -0.25 * abs(spd - this.PrevSpeed);

            % R5: Dwell phase handling
            %     During dwell (arm holding at paint station), going slow
            %     is correct; going fast wastes energy and causes vibration
            if this.F.is_dwell(i)
                if spd < 0.3
                    r_dwell = 0.15;   % good: slow during dwell
                else
                    r_dwell = -0.20;  % bad: fast during hold
                end
            else
                r_dwell = 0;
            end

            % R6: Curvature-aware penalty
            %     High curvature + high speed = bad tracking
            curv = this.F.curvature(i) / max(this.NM.curv, 1e-6);
            if curv > 0.5 && spd > 0.7
                r_curv = -0.15;  % penalise fast motion on tight curves
            elseif curv < 0.1 && spd > 0.6
                r_curv = 0.10;   % reward fast motion on straight segments
            else
                r_curv = 0;
            end

            reward = r_track + r_time + r_jerk + r_smooth + r_dwell + r_curv;

            % Accumulate metrics
            this.CumTrackErr = this.CumTrackErr + te;
            this.CumJerk     = this.CumJerk     + j;

            this.PrevSpeed = spd;
            this.StepIdx   = this.StepIdx + 1;
            isDone = (this.StepIdx >= this.NumSteps);

            if ~isDone
                obs = getObs(this);
            else
                obs = zeros(7,1);
                % Terminal bonus for completing the stroke
                reward = reward + 3;
            end

            info.speed_cmd      = spd;
            info.tracking_error = te;
            info.curvature      = this.F.curvature(i);
        end

        function obs = reset(this)
            this.StepIdx     = 1;
            this.PrevSpeed   = 0.5;
            this.CumTrackErr = 0;
            this.CumJerk     = 0;
            obs = getObs(this);
        end

        function obs = getObs(this)
            i = this.StepIdx;
            obs = [
                this.F.ee_velocity(i)    / max(this.NM.vel, 1e-6);
                abs(this.F.ee_accel(i))  / max(this.NM.accel, 1e-6);
                this.F.curvature(i)      / max(this.NM.curv, 1e-6);
                this.F.tracking_error(i) / max(this.NM.trkErr, 1e-6);
                this.F.progress(i);
                double(this.F.is_dwell(i));
                this.PrevSpeed
            ];
            obs = max(0, min(1, obs));
        end
    end
end