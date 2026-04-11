function out = create_position_only_rl_dataset(cfg)
%CREATE_POSITION_ONLY_RL_DATASET Build position-only RL dataset for KUKA + RoArm.
%   out = create_position_only_rl_dataset() builds:
%     1) kuka_position_rl_dataset.csv
%     2) roarm_position_rl_dataset.csv
%     3) multirobot_position_rl_dataset.csv
%
%   Dataset contains only position-related features (no torque/current).
%
%   Optional config fields:
%     cfg.kukaCsvPath        - source KUKA CSV path
%     cfg.outputDir          - output folder
%     cfg.collectKuka        - true/false
%     cfg.collectRoarm       - true/false
%     cfg.roarmMode          - 'online' or 'synthetic'
%     cfg.roarmIps           - cell array of RoArm IPs
%     cfg.roarmDt            - step interval seconds
%     cfg.roarmEpisodes      - number of trajectory episodes per arm
%     cfg.roarmStepsPerEp    - steps per episode
%     cfg.roarmSpeed         - command speed

    if nargin < 1
        cfg = struct();
    end
    cfg = fillDefaults(cfg);

    if ~isfolder(cfg.outputDir)
        mkdir(cfg.outputDir);
    end

    kukaT = table();
    roarmT = table();

    if cfg.collectKuka
        kukaT = buildKukaPositionTable(cfg.kukaCsvPath);
        kukaOut = fullfile(cfg.outputDir, 'kuka_position_rl_dataset.csv');
        writetable(kukaT, kukaOut);
    end

    if cfg.collectRoarm
        roarmT = buildRoarmPositionTable(cfg);
        roarmOut = fullfile(cfg.outputDir, 'roarm_position_rl_dataset.csv');
        writetable(roarmT, roarmOut);
    end

    mergedT = [kukaT; roarmT];
    if ~isempty(mergedT)
        mergedOut = fullfile(cfg.outputDir, 'multirobot_position_rl_dataset.csv');
        writetable(mergedT, mergedOut);
    else
        mergedOut = '';
    end

    out = struct();
    out.kukaRows = height(kukaT);
    out.roarmRows = height(roarmT);
    out.totalRows = height(mergedT);
    out.outputDir = cfg.outputDir;
    out.mergedCsv = mergedOut;

    fprintf('KUKA rows:   %d\n', out.kukaRows);
    fprintf('RoArm rows:  %d\n', out.roarmRows);
    fprintf('Total rows:  %d\n', out.totalRows);
    if ~isempty(out.mergedCsv)
        fprintf('Saved merged dataset: %s\n', out.mergedCsv);
    else
        fprintf('No rows exported. Check config/data source.\n');
    end
end

function cfg = fillDefaults(cfg)
    if ~isfield(cfg, 'kukaCsvPath') || isempty(cfg.kukaCsvPath)
        cfg.kukaCsvPath = fullfile(fileparts(mfilename('fullpath')), 'KUKA_control', 'kuka_rl_dataset.csv');
    end
    if ~isfield(cfg, 'outputDir') || isempty(cfg.outputDir)
        cfg.outputDir = fullfile(fileparts(mfilename('fullpath')), 'datasets');
    end
    if ~isfield(cfg, 'collectKuka')
        cfg.collectKuka = true;
    end
    if ~isfield(cfg, 'collectRoarm')
        cfg.collectRoarm = true;
    end
    if ~isfield(cfg, 'roarmMode') || isempty(cfg.roarmMode)
        cfg.roarmMode = 'synthetic';
    end
    if ~isfield(cfg, 'roarmIps') || isempty(cfg.roarmIps)
        cfg.roarmIps = {'192.168.1.192', '192.168.1.101'};
    end
    if ~isfield(cfg, 'roarmDt') || isempty(cfg.roarmDt)
        cfg.roarmDt = 0.20;
    end
    if ~isfield(cfg, 'roarmEpisodes') || isempty(cfg.roarmEpisodes)
        cfg.roarmEpisodes = 6;
    end
    if ~isfield(cfg, 'roarmStepsPerEp') || isempty(cfg.roarmStepsPerEp)
        cfg.roarmStepsPerEp = 40;
    end
    if ~isfield(cfg, 'roarmSpeed') || isempty(cfg.roarmSpeed)
        cfg.roarmSpeed = 5.0;
    end
end

function T = buildKukaPositionTable(kukaCsvPath)
    if ~isfile(kukaCsvPath)
        warning('KUKA CSV not found: %s', kukaCsvPath);
        T = emptyPositionTable();
        return;
    end

    src = readtable(kukaCsvPath);
    req = {'run_id','step_idx','time_s','target_x','target_y','target_z','meas_x','meas_y','meas_z','dt'};
    missing = req(~ismember(req, src.Properties.VariableNames));
    if ~isempty(missing)
        error('KUKA CSV missing required columns: %s', strjoin(missing, ', '));
    end

    n = height(src);
    robot = repmat({'kuka'}, n, 1);
    arm_id = repmat({'kuka_main'}, n, 1);
    run_id = stringCol(src.run_id, n);

    T = table(run_id, robot, arm_id, src.step_idx, src.time_s, ...
        src.target_x, src.target_y, src.target_z, ...
        src.meas_x, src.meas_y, src.meas_z, src.dt, ...
        'VariableNames', positionColumnNames());

    T = sanitizePositionTable(T);
end

function T = buildRoarmPositionTable(cfg)
    switch lower(string(cfg.roarmMode))
        case "online"
            T = collectRoarmOnline(cfg);
        otherwise
            T = collectRoarmSynthetic(cfg);
    end
end

function T = collectRoarmOnline(cfg)
    records = [];
    runBase = datestr(now, 'yyyymmdd_HHMMSS');

    for a = 1:numel(cfg.roarmIps)
        ip = cfg.roarmIps{a};
        armTag = sprintf('roarm_%d', a);
        stepGlobal = 0;

        for ep = 1:cfg.roarmEpisodes
            waypoints = generateRoarmWaypoints(cfg.roarmStepsPerEp);
            run_id = sprintf('%s_%s_ep%02d', runBase, armTag, ep);

            for k = 1:size(waypoints,1)
                stepGlobal = stepGlobal + 1;
                t = (stepGlobal - 1) * cfg.roarmDt;
                target = waypoints(k,:);

                sendRoarmMove(ip, target, cfg.roarmSpeed);
                pause(cfg.roarmDt);

                m = readRoarmXYZ(ip);
                if any(isnan(m))
                    m = target;
                end

                records = [records; makeRow(run_id, 'roarm', armTag, stepGlobal, t, target, m, cfg.roarmDt)]; %#ok<AGROW>
            end
        end
    end

    T = rowsToTable(records);
    T = sanitizePositionTable(T);
end

function T = collectRoarmSynthetic(cfg)
    records = [];
    runBase = datestr(now, 'yyyymmdd_HHMMSS');

    for a = 1:numel(cfg.roarmIps)
        armTag = sprintf('roarm_%d', a);
        stepGlobal = 0;

        for ep = 1:cfg.roarmEpisodes
            waypoints = generateRoarmWaypoints(cfg.roarmStepsPerEp);
            run_id = sprintf('%s_%s_ep%02d', runBase, armTag, ep);

            for k = 1:size(waypoints,1)
                stepGlobal = stepGlobal + 1;
                t = (stepGlobal - 1) * cfg.roarmDt;
                target = waypoints(k,:);

                % Synthetic measured pose with small sensor/motion noise.
                meas = target + randn(1,3) .* [1.5, 1.5, 1.0];
                records = [records; makeRow(run_id, 'roarm', armTag, stepGlobal, t, target, meas, cfg.roarmDt)]; %#ok<AGROW>
            end
        end
    end

    T = rowsToTable(records);
    T = sanitizePositionTable(T);
end

function W = generateRoarmWaypoints(n)
    % Reachable region tuned for RoArm M2 workspace in mm.
    xRange = [120, 340];
    yRange = [-220, 220];
    zRange = [80, 260];

    anchors = [
        250,   0, 220;
        300, 120, 190;
        300,-120, 190;
        210, 160, 130;
        210,-160, 130;
        280,   0, 250
    ];
    idx = randi(size(anchors,1), [4,1]);
    key = anchors(idx,:);

    W = zeros(n,3);
    segN = max(1, floor(n / (size(key,1)-1)));
    ptr = 1;
    for s = 1:(size(key,1)-1)
        p0 = key(s,:);
        p1 = key(s+1,:);
        for i = 1:segN
            if ptr > n
                break;
            end
            alpha = (i-1) / max(1, segN-1);
            W(ptr,:) = (1-alpha) * p0 + alpha * p1;
            ptr = ptr + 1;
        end
    end
    while ptr <= n
        W(ptr,:) = key(end,:);
        ptr = ptr + 1;
    end

    W(:,1) = clamp(W(:,1), xRange(1), xRange(2));
    W(:,2) = clamp(W(:,2), yRange(1), yRange(2));
    W(:,3) = clamp(W(:,3), zRange(1), zRange(2));
end

function sendRoarmMove(ip, xyz, spd)
    cmd = struct('T', 104, 'x', round(xyz(1)), 'y', round(xyz(2)), 'z', round(xyz(3)), 't', 0, 'spd', spd);
    url = ['http://' ip '/js?json=' jsonencode(cmd)]; %#ok<AGROW>
    try
        opts = weboptions('Timeout', 1.2);
        webread(url, opts);
    catch
        % Ignore communication drops and continue collection.
    end
end

function xyz = readRoarmXYZ(ip)
    xyz = [nan, nan, nan];
    cmd = struct('T', 105);
    url = ['http://' ip '/js?json=' jsonencode(cmd)]; %#ok<AGROW>

    try
        opts = weboptions('Timeout', 1.2);
        resp = webread(url, opts);
        data = parseJsonResponse(resp);
        xyz(1) = getFieldNum(data, 'x', nan);
        xyz(2) = getFieldNum(data, 'y', nan);
        xyz(3) = getFieldNum(data, 'z', nan);
    catch
        % Keep NaN if read failed.
    end
end

function data = parseJsonResponse(resp)
    if isstruct(resp)
        data = resp;
        return;
    end

    txt = char(string(resp));
    s = strfind(txt, '{');
    e = strfind(txt, '}');
    if isempty(s) || isempty(e)
        error('Invalid response format.');
    end

    js = txt(s(1):e(end));
    data = jsondecode(js);
end

function v = getFieldNum(S, field, defaultVal)
    if isstruct(S) && isfield(S, field)
        v = double(S.(field));
        if ~isfinite(v)
            v = defaultVal;
        end
    else
        v = defaultVal;
    end
end

function rec = makeRow(run_id, robot, arm_id, step_idx, time_s, target, meas, dt)
    rec = struct();
    rec.run_id = string(run_id);
    rec.robot = string(robot);
    rec.arm_id = string(arm_id);
    rec.step_idx = double(step_idx);
    rec.time_s = double(time_s);
    rec.target_x = double(target(1));
    rec.target_y = double(target(2));
    rec.target_z = double(target(3));
    rec.meas_x = double(meas(1));
    rec.meas_y = double(meas(2));
    rec.meas_z = double(meas(3));
    rec.dt = double(dt);
end

function T = rowsToTable(records)
    if isempty(records)
        T = emptyPositionTable();
        return;
    end

    T = struct2table(records);
    T = T(:, positionColumnNames());
end

function T = emptyPositionTable()
    n = 0;
    T = table(strings(n,1), strings(n,1), strings(n,1), zeros(n,1), zeros(n,1), ...
        zeros(n,1), zeros(n,1), zeros(n,1), zeros(n,1), zeros(n,1), zeros(n,1), zeros(n,1), ...
        'VariableNames', positionColumnNames());
end

function names = positionColumnNames()
    names = {'run_id','robot','arm_id','step_idx','time_s', ...
             'target_x','target_y','target_z','meas_x','meas_y','meas_z','dt'};
end

function T = sanitizePositionTable(T)
    vars = T.Properties.VariableNames;
    for i = 1:numel(vars)
        col = T.(vars{i});
        if isnumeric(col)
            col = fillmissing(col, 'previous');
            col = fillmissing(col, 'next');
            col(~isfinite(col)) = 0;
            T.(vars{i}) = col;
        end
    end
end

function out = stringCol(in, n)
    if isstring(in)
        out = in;
    elseif iscellstr(in)
        out = string(in);
    elseif ischar(in)
        out = repmat(string(in), n, 1);
    else
        out = string(in);
    end
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end
