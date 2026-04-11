classdef KukaPickPlaceEnv < rl.env.MATLABEnvironment
%  RL environment for KUKA KR10 pick-and-place velocity optimisation.
%
%  State (8x1 normalised 0-1):
%    [ee_velocity, |ee_accel|, curvature, tracking_error,
%     torque_mag,  progress,  dist_to_pick_norm, dist_to_place_norm]
%
%  Action (1x1 continuous):
%    speed_override in [0.1, 1.0]  — scales the KUKA $OV_PRO velocity
%
%  Reward: minimise cycle_time + tracking_error + jerk + energy

    properties
        F;  NM;  NumSteps;  StepIdx;  PrevSpeed;
        CumEnergy;  CumTrackErr;  CumJerk;
    end

    methods
        function this = KukaPickPlaceEnv(features, normStruct)
            obsInfo = rlNumericSpec([8 1],'LowerLimit',zeros(8,1),'UpperLimit',ones(8,1));
            obsInfo.Name = 'state';
            actInfo = rlNumericSpec([1 1],'LowerLimit',0.1,'UpperLimit',1.0);
            actInfo.Name = 'speed_override';
            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);
            this.F  = features;
            this.NM = normStruct;
            this.NumSteps = length(features.ee_velocity);
        end

        function [obs,reward,isDone,info] = step(this,action)
            spd = max(0.1, min(1.0, action(1)));
            i   = this.StepIdx;

            %% --- Reward components (all normalised to ~[-1, +1]) ---

            % R1: Cycle time penalty — faster is better
            %     Higher speed_override = less time per step = good
            r_time = 0.3 * (spd - 0.5);  % reward > 0 when spd > 0.5

            % R2: Tracking error penalty — lower is better
            te = this.F.tracking_error(i);
            r_track = -0.4 * (te / max(this.NM.trkErr, 1e-6));

            % R3: Jerk penalty — smoother is better
            j = abs(this.F.jerk(i));
            r_jerk = -0.15 * (j / max(this.NM.jerk, 1e-6));

            % R4: Energy penalty — lower current = less wear
            c = this.F.curr_mag(i);
            r_energy = -0.1 * (c / max(this.NM.curr, 1e-6));

            % R5: Smoothness — penalise abrupt speed changes
            r_smooth = -0.3 * abs(spd - this.PrevSpeed);

            % R6: Endpoint approach bonus — slow down near pick/place
            nearPick  = this.F.norm_to_pick(i)  < 0.1;
            nearPlace = this.F.norm_to_place(i) < 0.1;
            if (nearPick || nearPlace) && spd < 0.4
                r_approach = 0.2;   % good: slow near endpoints
            elseif (nearPick || nearPlace) && spd > 0.7
                r_approach = -0.3;  % bad: too fast near endpoints
            else
                r_approach = 0;
            end

            reward = r_time + r_track + r_jerk + r_energy + r_smooth + r_approach;

            % Accumulate metrics
            this.CumEnergy   = this.CumEnergy   + c;
            this.CumTrackErr = this.CumTrackErr  + te;
            this.CumJerk     = this.CumJerk      + j;

            this.PrevSpeed = spd;
            this.StepIdx   = this.StepIdx + 1;
            isDone = (this.StepIdx >= this.NumSteps);

            if ~isDone
                obs = getObs(this);
            else
                obs = zeros(8,1);
                % Terminal bonus for completing the trajectory
                reward = reward + 5;
            end
            info.speed_override = spd;
            info.tracking_error = te;
            info.energy = c;
        end

        function obs = reset(this)
            this.StepIdx     = 1;
            this.PrevSpeed   = 0.5;
            this.CumEnergy   = 0;
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
                this.F.torque_mag(i)     / max(this.NM.torque, 1e-6);
                this.F.progress(i);
                this.F.norm_to_pick(i);
                this.F.norm_to_place(i)
            ];
            obs = max(0, min(1, obs));
        end
    end
end
