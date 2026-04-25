classdef RoarmPaintingEnv < rl.env.MATLABEnvironment

    properties
        % Tunable environment parameters
        Ts            = 0.20
        MaxSteps      = 240
        SpeedBaseline = 0.5
        ErrThresh     = 1.5

        % Internal state (set during reset/step)
        StepIdx       = 0
        RunEndStep    = 0
        PrevError     = 0
        PrevSpeed     = 0.5

        % Path data arrays for the active episode
        PathCurvature = []
        PathVelocity  = []
    end

    properties (Access = private)
        DataTable
        EpisodeTable
        PathAccel
        TrackErrorSeries
        ProgressSeries
        DwellSeries
        TrackErrorScale
        VelocityScale
        AccelScale
    end

    methods
        function env = RoarmPaintingEnv()
            % Match the 8D observation used by runRoarmPaintingRL.
            obsInfo = rlNumericSpec([8 1], ...
                'LowerLimit', -inf(8,1), ...
                'UpperLimit',  inf(8,1));
            obsInfo.Name = 'RoArmObs';
            obsInfo.Description = 'trackErr,vel,accel,curv,progress,dwell,prevSpd,errDelta';

            actInfo = rlNumericSpec([1 1], ...
                'LowerLimit', 0.0, ...
                'UpperLimit', 1.0);
            actInfo.Name = 'SpeedCmd';
            actInfo.Description = 'Normalised painting speed [0=stop, 1=max]';

            env = env@rl.env.MATLABEnvironment(obsInfo, actInfo);

            csvPath = env.resolveDatasetPath();
            env.DataTable = readtable(csvPath, 'VariableNamingRule', 'preserve');
            if isempty(env.DataTable)
                error('RoArm dataset is empty: %s', csvPath);
            end

            env.DataTable.run_id = string(env.DataTable.run_id);
            if ismember('dt', env.DataTable.Properties.VariableNames)
                env.Ts = double(env.DataTable.dt(1));
            end

            runIds = unique(env.DataTable.run_id);
            runLengths = zeros(numel(runIds), 1);
            for i = 1:numel(runIds)
                runLengths(i) = sum(env.DataTable.run_id == runIds(i));
            end
            env.MaxSteps = max(runLengths);
        end

        function [obs, reward, isDone, loggedSignals] = step(this, action)
            loggedSignals = struct();
            speedCmd = max(0, min(1, double(action(1))));

            this.StepIdx = min(this.StepIdx + 1, this.RunEndStep);
            idx = this.StepIdx;
            row = this.EpisodeTable(idx, :); %#ok<NASGU>

            trackErr = this.TrackErrorSeries(idx);
            curv = this.PathCurvature(idx);
            vel = this.PathVelocity(idx);
            isDwell = this.DwellSeries(idx);

            % Match the data-replay reward from runRoarmPaintingRL.
            errNormReward = trackErr / max(this.ErrThresh, 1e-6);
            r_track  = -2.0 * errNormReward^2;
            r_speed  =  0.3 * speedCmd * max(0, 1 - errNormReward);
            curvNorm = min(1, curv / pi);
            idealSpeed = 1.0 * (1 - curvNorm) + 0.1 * curvNorm;
            r_curv   = -0.4 * (speedCmd - idealSpeed)^2;
            r_smooth = -0.5 * abs(speedCmd - this.PrevSpeed);
            r_dwell  = 0;
            if isDwell && speedCmd < 0.2
                r_dwell = 0.3;
            end

            reward = r_track + r_speed + r_curv + r_smooth + r_dwell;
            isDone = (idx >= this.RunEndStep);

            this.PrevError = trackErr;
            this.PrevSpeed = speedCmd;

            obs = this.getObs(idx, this.PrevSpeed, this.PrevError);

            loggedSignals.stepIdx = idx;
            loggedSignals.speed_command = speedCmd;
            loggedSignals.tracking_error = trackErr;
        end

        function obs = reset(this)
            runs = unique(this.DataTable.run_id);
            runID = runs(randi(numel(runs)));
            runRows = (this.DataTable.run_id == runID);

            this.EpisodeTable = this.DataTable(runRows, :);
            this.RunEndStep = height(this.EpisodeTable);
            this.MaxSteps = this.RunEndStep;
            this.StepIdx = 0;
            this.PrevError = 0;
            this.PrevSpeed = this.SpeedBaseline;

            if ismember('dt', this.EpisodeTable.Properties.VariableNames)
                this.Ts = double(this.EpisodeTable.dt(1));
            end

            this.PathCurvature = this.computeCurvature(this.EpisodeTable);
            this.PathVelocity = this.computeVelocity(this.EpisodeTable);
            this.PathAccel = this.computeAcceleration(this.PathVelocity, this.Ts);
            this.TrackErrorSeries = this.computeTrackingError(this.EpisodeTable);
            this.ProgressSeries = this.computeProgress(this.EpisodeTable);
            this.DwellSeries = this.computeDwell(this.EpisodeTable);

            this.TrackErrorScale = max(max(this.TrackErrorSeries), 1e-6);
            this.VelocityScale = max(max(abs(this.PathVelocity)), 1e-6);
            this.AccelScale = max(max(abs(this.PathAccel)), 1e-6);

            obs = this.getObs(1, this.SpeedBaseline, 0);
        end
    end

    methods (Access = private)
        function csvPath = resolveDatasetPath(~)
            scriptDir = fileparts(mfilename('fullpath'));
            csvPath = fullfile(scriptDir, 'roarm_position_rl_dataset.csv');
            if isfile(csvPath)
                return;
            end

            altPath = fullfile(scriptDir, '..', 'roarm_position_rl_dataset.csv');
            if isfile(altPath)
                csvPath = altPath;
                return;
            end

            error(['Unable to find or open ''roarm_position_rl_dataset.csv''. ', ...
                   'Check the path and filename or file permissions.']);
        end

        function obs = getObs(this, idx, prevSpeed, prevError)
            errNorm = this.TrackErrorSeries(idx) / this.TrackErrorScale;
            prevErrNorm = prevError / this.TrackErrorScale;

            obs = [
                errNorm;
                this.PathVelocity(idx) / this.VelocityScale;
                this.PathAccel(idx) / this.AccelScale;
                this.PathCurvature(idx) / (pi + 1e-6);
                this.ProgressSeries(idx);
                double(this.DwellSeries(idx));
                prevSpeed;
                errNorm - prevErrNorm
            ];
        end

        function trackErr = computeTrackingError(~, T)
            if ismember('track_err_mm', T.Properties.VariableNames)
                trackErr = double(T.track_err_mm);
                return;
            end

            target = [T.target_x, T.target_y, T.target_z];
            meas = [T.meas_x, T.meas_y, T.meas_z];
            trackErr = vecnorm(target - meas, 2, 2);
        end

        function progress = computeProgress(~, T)
            target = [T.target_x, T.target_y, T.target_z];
            n = height(T);
            progress = zeros(n, 1);
            for i = 2:n
                progress(i) = progress(i-1) + norm(target(i,:) - target(i-1,:));
            end
            progress = progress / max(progress(end), 1e-6);
        end

        function dwell = computeDwell(~, T)
            target = [T.target_x, T.target_y, T.target_z];
            n = height(T);
            dwell = false(n, 1);
            if n == 0
                return;
            end
            if n == 1
                dwell(1) = true;
                return;
            end

            for i = 2:n
                dwell(i) = norm(target(i,:) - target(i-1,:)) < 0.5;
            end
            dwell(1) = norm(target(1,:) - target(2,:)) < 0.5;
        end

        function curv = computeCurvature(~, T)
            x = T.target_x;
            y = T.target_y;
            n = height(T);
            curv = zeros(n, 1);
            if n <= 2
                return;
            end

            for i = 2:n-1
                dx1 = x(i) - x(i-1);
                dy1 = y(i) - y(i-1);
                dx2 = x(i+1) - x(i);
                dy2 = y(i+1) - y(i);
                cross_ = dx1 * dy2 - dy1 * dx2;
                denom = (dx1^2 + dy1^2) * (dx2^2 + dy2^2) * ((dx1 - dx2)^2 + (dy1 - dy2)^2);
                if denom > 1e-12
                    curv(i) = abs(cross_) / sqrt(denom + 1e-12);
                end
            end
            curv(1) = curv(2);
            curv(end) = curv(end-1);
        end

        function vel = computeVelocity(~, T)
            x = T.target_x;
            y = T.target_y;
            z = T.target_z;
            dt = double(T.dt(1));
            n = height(T);
            vel = zeros(n, 1);
            if n <= 1
                return;
            end

            for i = 2:n
                vel(i) = sqrt((x(i) - x(i-1))^2 + (y(i) - y(i-1))^2 + ...
                              (z(i) - z(i-1))^2) / dt;
            end
            vel(1) = vel(2);
        end

        function accel = computeAcceleration(~, vel, dt)
            n = numel(vel);
            accel = zeros(n, 1);
            if n <= 1
                return;
            end

            for i = 2:n
                accel(i) = (vel(i) - vel(i-1)) / max(dt, 1e-6);
            end
            accel(1) = accel(2);
        end
    end
end
