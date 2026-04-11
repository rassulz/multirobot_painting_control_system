classdef RoarmPaintingEnv < rl.env.MATLABEnvironment

    properties
        % Tunable environment parameters
        Ts            = 0.20
        MaxSteps      = 240
        SpeedBaseline = 0.5
        ErrThresh     = 1.5        % mm — tracking error threshold

        % Internal state (set during reset/step)
        StepIdx       = 0
        RunEndStep    = 0
        PrevError     = 0
        PrevSpeed     = 0.5

        % Path data arrays (populated in reset)
        PathCurvature = []
        PathVelocity  = []
    end

    properties (Access = private)
        % Private arrays loaded from dataset
        DataTable
    end

    methods
        function env = RoarmPaintingEnv()
            % Define observation info: [trackErr, speed, curvature, velocity, prevSpeed]
            obsInfo = rlNumericSpec([5 1], ...
                'LowerLimit', -inf, ...
                'UpperLimit',  inf);
            obsInfo.Name = 'observations';

            % Define action info: speed command in [0.1, 1.0]
            actInfo = rlNumericSpec([1 1], ...
                'LowerLimit', 0.1, ...
                'UpperLimit', 1.0);
            actInfo.Name = 'speed_command';

            % Call superclass constructor
            env = env@rl.env.MATLABEnvironment(obsInfo, actInfo);

            % Load dataset
            env.DataTable = readtable('roarm_rl_dataset.csv');
        end

        function [obs, reward, isDone, loggedSignals] = step(this, action)
            loggedSignals = [];
            speedCmd = double(action);

            % Advance step index
            this.StepIdx = this.StepIdx + 1;
            idx = min(this.StepIdx, this.RunEndStep);

            % Retrieve path state from dataset row
            row      = this.DataTable(idx, :);
            trackErr = row.track_err_mm;
            curv     = this.PathCurvature(idx);
            vel      = this.PathVelocity(idx);

            % Compute reward (matches analyse_reward.m weights)
            W_track = 2.0;  W_speed = 0.3;  W_curv = 0.4;
            errNorm    = trackErr / this.ErrThresh;
            r_track    = -W_track * errNorm^2;
            r_speed    = W_speed * speedCmd * max(0, 1 - errNorm);
            curvNorm   = min(1, curv / pi);
            idealSpeed = 1.0*(1-curvNorm) + 0.1*curvNorm;
            r_curv     = -W_curv * (speedCmd - idealSpeed)^2;
            reward     = r_track + r_speed + r_curv;

            % Update state
            this.PrevError = trackErr;
            this.PrevSpeed = speedCmd;

            % Build observation vector
            obs = [trackErr; speedCmd; curv; vel; this.PrevSpeed];

            % Episode termination
            isDone = (this.StepIdx >= this.RunEndStep);
        end

        function obs = reset(this)
            % Pick a random episode from the dataset
            runs  = unique(this.DataTable.run_id);
            runID = runs{randi(numel(runs))};
            runRows = strcmp(this.DataTable.run_id, runID);

            subT = this.DataTable(runRows, :);
            this.RunEndStep = height(subT);
            this.StepIdx    = 0;
            this.PrevError  = 0;
            this.PrevSpeed  = this.SpeedBaseline;

            % Pre-compute curvature & velocity arrays for this run
            this.PathCurvature = this.computeCurvature(subT);
            this.PathVelocity  = this.computeVelocity(subT);

            % Return initial observation
            obs = [0; this.SpeedBaseline; this.PathCurvature(1); ...
                   this.PathVelocity(1); this.SpeedBaseline];
        end
    end  % public methods

    methods (Access = private)
        function curv = computeCurvature(~, T)
            x = T.target_x;  y = T.target_y;
            n = height(T);
            curv = zeros(n, 1);
            for i = 2:n-1
                dx1 = x(i)-x(i-1);  dy1 = y(i)-y(i-1);
                dx2 = x(i+1)-x(i);  dy2 = y(i+1)-y(i);
                cross_ = dx1*dy2 - dy1*dx2;
                denom  = (dx1^2+dy1^2) * (dx2^2+dy2^2) * ((dx1-dx2)^2+(dy1-dy2)^2);
                if denom > 1e-12
                    curv(i) = abs(cross_) / sqrt(denom + 1e-12);
                end
            end
            curv(1) = curv(2);  curv(end) = curv(end-1);
        end

        function vel = computeVelocity(~, T)
            x = T.target_x;  y = T.target_y;  z = T.target_z;
            dt = T.dt(1);
            n = height(T);
            vel = zeros(n, 1);
            for i = 2:n
                vel(i) = sqrt((x(i)-x(i-1))^2 + (y(i)-y(i-1))^2 + ...
                              (z(i)-z(i-1))^2) / dt;
            end
            vel(1) = vel(2);
        end
    end  % private methods

end  % classdef