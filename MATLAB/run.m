function results = run_full_multirobot_pipeline(cfg)
%RUN_FULL_MULTIROBOT_PIPELINE  Single script: KUKA + RoArm + Collision RL.
%
%   results = run_full_multirobot_pipeline()       % synthetic
%   results = run_full_multirobot_pipeline(cfg)     % custom
%
%   CORRECT REAL-WORLD FLOW:
%     Phase 0: Setup paths
%     Phase 1: KUKA pick-and-place RL (learn velocity profile)
%     Phase 2: RoArm painting RL (learn speed profile)
%     Phase 3: Run KUKA + RoArm TOGETHER, collect live collision data
%     Phase 4: Train collision avoidance agent on that data
%     Phase 5: Summary
%
%   The key insight: collision avoidance data must come from all 3 robots
%   operating simultaneously — the KUKA is moving workpieces while the
%   RoArms are painting. The collision agent learns from THAT scenario.

    if nargin < 1, cfg = struct(); end
    cfg = fillPipelineDefaults(cfg);

    %% ═══════════════════════════════════════════════════════════════════
    %  PHASE 0 — SETUP
    %  ═══════════════════════════════════════════════════════════════════
    fprintf('\n');
    fprintf('╔══════════════════════════════════════���═══════════════════════╗\n');
    fprintf('║  MULTIROBOT ADAPTIVE CONTROL — FULL PIPELINE                ║\n');
    fprintf('║  KUKA KR10  +  2x RoArm-M2-S  +  Reinforcement Learning    ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════╝\n');
    fprintf('  Mode: %s\n\n', upper(cfg.mode));

    tStart = tic;

    matlabDir = fileparts(mfilename('fullpath'));
    if isempty(matlabDir), matlabDir = pwd; end

    rlDir    = fullfile(matlabDir, 'RL');
    kukaDir  = fullfile(matlabDir, 'KUKA_control');
    roarmDir = fullfile(matlabDir, 'Roarm_control');

    for p = {rlDir, kukaDir, fullfile(kukaDir,'srcs'), roarmDir}
        if isfolder(p{1}), addpath(genpath(p{1})); end
    end
    fprintf('[Phase 0] Paths configured.\n');

    savedAgentsDir = fullfile(rlDir, 'savedAgents');
    if ~isfolder(savedAgentsDir), mkdir(savedAgentsDir); end

    R = struct();
    R.timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    R.mode      = cfg.mode;
    R.phases    = struct('setup','PASS','kukaRL','SKIPPED','roarmRL','SKIPPED', ...
                         'liveCollect','SKIPPED','collisionRL','SKIPPED');

    %% ═══════════════════════════════════════════════════════════════════
    %  PHASE 1 — KUKA PICK-AND-PLACE RL
    %  Train the KUKA velocity profile FIRST, so it knows how to move
    %  workpieces efficiently before we run the full system.
    %  ═══════════════════════════════════════════════════════════════════
    if cfg.trainKuka
        fprintf('\n[Phase 1] KUKA PICK-AND-PLACE RL\n');
        fprintf('---------------------------------\n');
        kukaCSV = fullfile(rlDir, 'kuka_rl_dataset.csv');
        if ~isfile(kukaCSV)
            fprintf('  x kuka_rl_dataset.csv not found. Skipping.\n');
            R.phases.kukaRL = 'FAIL';
        else
            try
                fprintf('  Dataset: %s\n', kukaCSV);
                runScriptIsolated(rlDir, 'runKukaPickPlaceRL');
                R.phases.kukaRL = 'PASS';
                fprintf('  + KUKA RL complete.\n');
            catch ME
                R.phases.kukaRL = 'FAIL';
                R.kukaError = ME.message;
                fprintf('  x KUKA RL FAILED: %s\n', ME.message);
            end
        end
    else
        fprintf('\n[Phase 1] KUKA RL SKIPPED\n');
    end

    %% ═══════════════════════════════════════════════════════════════════
    %  PHASE 2 — ROARM PAINTING RL
    %  Train the RoArm painting speed profile, so it knows how to paint
    %  before we run the full system.
    %  ═══════════════════════════════════════════════════════════════════
    if cfg.trainRoarm
        fprintf('\n[Phase 2] ROARM PAINTING RL\n');
        fprintf('----------------------------\n');

        % Generate RoArm dataset if needed
        roarmCSV = fullfile(rlDir, 'roarm_position_rl_dataset.csv');
        if ~isfile(roarmCSV) && exist('create_position_only_rl_dataset','file')
            fprintf('  Generating RoArm dataset ...\n');
            dcfg = cfg.datasetCfg;
            dcfg.outputDir    = rlDir;
            dcfg.collectKuka  = false;
            dcfg.collectRoarm = true;
            create_position_only_rl_dataset(dcfg);
        end

        if ~isfile(roarmCSV)
            fprintf('  x roarm_position_rl_dataset.csv not found. Skipping.\n');
            R.phases.roarmRL = 'FAIL';
        else
            try
                fprintf('  Dataset: %s\n', roarmCSV);
                runScriptIsolated(rlDir, 'runRoarmPaintingRL');
                R.phases.roarmRL = 'PASS';
                fprintf('  + RoArm RL complete.\n');
            catch ME
                R.phases.roarmRL = 'FAIL';
                R.roarmError = ME.message;
                fprintf('  x RoArm RL FAILED: %s\n', ME.message);
            end
        end
    else
        fprintf('\n[Phase 2] RoArm RL SKIPPED\n');
    end

    %% ═══════════════════════════════════════════════════════════════════
    %  PHASE 3 — LIVE COLLECTION: RUN ALL ROBOTS TOGETHER
    %  THIS is where the real collision data comes from.
    %  The KUKA moves workpieces while both RoArms paint.
    %  We sample all 3 positions at each timestep.
    %  ═══════════════════════════════════════════════════════════════════
    collisionDataset = table();

    if cfg.trainCollision
        fprintf('\n[Phase 3] LIVE MULTI-ROBOT DATA COLLECTION\n');
        fprintf('--------------------------------------------\n');
        fprintf('  All 3 robots run together. Collecting position data\n');
        fprintf('  for collision avoidance training.\n\n');

        try
            kukaTcp = [];

            if strcmpi(cfg.mode, 'online')
                % --- ONLINE: connect to real hardware ---
                fprintf('  Connecting to KUKA at %s:%d ...\n', cfg.kukaIp, cfg.kukaPort);
                try
                    kukaTcp = tcpclient(cfg.kukaIp, cfg.kukaPort, 'Timeout', 10);
                    fprintf('  + KUKA connected.\n');
                catch ME
                    warning('  KUKA connection failed: %s\n  Using fixed position.', ME.message);
                end

                % Use synchronized_collector — reads all 3 robots each timestep
                fprintf('  Running synchronized_collector (online) ...\n');
                collisionDataset = synchronized_collector(cfg.syncCfg, kukaTcp);

                if ~isempty(kukaTcp) && isvalid(kukaTcp)
                    clear kukaTcp;
                end
            else
                % --- SYNTHETIC: simulate all 3 robots operating together ---
                % collect_collision_dataset already simulates KUKA at a fixed
                % position + 2 RoArms with trajectory noise — this IS the
                % scenario of all 3 robots in the same workspace.
                colCfg = cfg.collisionCfg.collect;
                colCfg.mode = 'synthetic';
                fprintf('  Running synthetic collection (KUKA at [%s] + 2 RoArms) ...\n', ...
                    num2str(colCfg.kukaPosition));
                collisionDataset = collect_collision_dataset(colCfg);
            end

            R.collisionRows = height(collisionDataset);
            R.phases.liveCollect = 'PASS';
            fprintf('  + Collected %d rows from all 3 robots.\n', height(collisionDataset));

        catch ME
            R.phases.liveCollect = 'FAIL';
            R.collectError = ME.message;
            fprintf('  x Collection FAILED: %s\n', ME.message);
        end
    else
        fprintf('\n[Phase 3] Live collection SKIPPED\n');
    end

    %% ═══════════════════════════════════════════════════════════════════
    %  PHASE 4 — TRAIN COLLISION AVOIDANCE AGENT
    %  Uses the data from Phase 3 (all robots operating together).
    %  ═══════════════════════════════════════════════════════════════════
    if cfg.trainCollision && strcmp(R.phases.liveCollect, 'PASS')
        fprintf('\n[Phase 4] COLLISION AVOIDANCE RL TRAINING\n');
        fprintf('------------------------------------------\n');
        try
            fprintf('  Training DDPG agent on %d rows of multi-robot data ...\n', ...
                height(collisionDataset));
            train_collision_agent(cfg.collisionCfg.train);

            % Evaluate
            agentFiles = dir(fullfile(savedAgentsDir, 'collision_agent_*.mat'));
            if ~isempty(agentFiles)
                [~, idx] = max([agentFiles.datenum]);
                agentPath = fullfile(savedAgentsDir, agentFiles(idx).name);
                fprintf('  Evaluating: %s\n', agentPath);
                evaluate_collision_agent(agentPath, cfg.collisionCfg.eval);
            end

            R.phases.collisionRL = 'PASS';
            fprintf('  + Collision avoidance training complete.\n');
        catch ME
            R.phases.collisionRL = 'FAIL';
            R.collisionError = ME.message;
            fprintf('  x Collision RL FAILED: %s\n', ME.message);
        end
    elseif cfg.trainCollision
        fprintf('\n[Phase 4] Collision RL SKIPPED — no data from Phase 3.\n');
    else
        fprintf('\n[Phase 4] Collision RL SKIPPED\n');
    end

    %% ═══════════════════════════════════════════════════════════════════
    %  PHASE 5 — SUMMARY
    %  ═══════════════════════════════════════════════════════════════════
    elapsed = toc(tStart);
    R.totalTime_s = elapsed;

    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║  PIPELINE SUMMARY                                           ║\n');
    fprintf('╚══════════════════════════════════════════════════════════════╝\n');
    fprintf('  Mode:       %s\n', upper(cfg.mode));
    fprintf('  Time:       %.1f s (%.1f min)\n\n', elapsed, elapsed/60);
    fprintf('  %-35s %s\n', 'Phase', 'Status');
    fprintf('  %s\n', repmat('-',1,50));
    fprintf('  %-35s %s\n', '0. Setup',                   R.phases.setup);
    fprintf('  %-35s %s\n', '1. KUKA Pick-Place RL',      R.phases.kukaRL);
    fprintf('  %-35s %s\n', '2. RoArm Painting RL',       R.phases.roarmRL);
    fprintf('  %-35s %s\n', '3. Live Multi-Robot Collect', R.phases.liveCollect);
    fprintf('  %-35s %s\n', '4. Collision Avoidance RL',   R.phases.collisionRL);
    fprintf('\n');

    statuses = struct2cell(R.phases);
    nPass = sum(strcmp(statuses,'PASS'));
    nFail = sum(strcmp(statuses,'FAIL'));
    nSkip = sum(strcmp(statuses,'SKIPPED'));
    fprintf('  PASS: %d | FAIL: %d | SKIPPED: %d\n', nPass, nFail, nSkip);

    resultsFile = fullfile(matlabDir, sprintf('pipeline_results_%s.mat', R.timestamp));
    pipeline_results = R; %#ok<NASGU>
    save(resultsFile, 'pipeline_results');
    fprintf('  Results: %s\n\n', resultsFile);

    results = R;
end

%% ═════════════════════════════════════════════════════════════════════════
%  HELPERS
%% ═════════════════════════════════════════════════════════════════════════

function runScriptIsolated(folder, scriptName)
% Runs a script that calls 'clear' inside a function scope so 'clear'
% only wipes this wrapper's scope, not the pipeline's workspace.
    old = pwd;
    cd(folder);
    try
        run(scriptName);
    catch ME
        cd(old);
        rethrow(ME);
    end
    cd(old);
end

function cfg = fillPipelineDefaults(cfg)
    if ~isfield(cfg,'mode'),           cfg.mode           = 'synthetic'; end
    if ~isfield(cfg,'trainKuka'),      cfg.trainKuka      = true;        end
    if ~isfield(cfg,'trainRoarm'),     cfg.trainRoarm     = true;        end
    if ~isfield(cfg,'trainCollision'), cfg.trainCollision = true;        end

    if ~isfield(cfg,'kukaIp'),   cfg.kukaIp   = '172.31.17.101';                  end
    if ~isfield(cfg,'kukaPort'), cfg.kukaPort = 7000;                              end
    if ~isfield(cfg,'roarmIps'), cfg.roarmIps = {'192.168.1.192','192.168.1.101'}; end

    % Dataset generation (for RoArm if CSV missing)
    if ~isfield(cfg,'datasetCfg'), cfg.datasetCfg = struct(); end
    dc = cfg.datasetCfg;
    if ~isfield(dc,'roarmMode'),       dc.roarmMode       = 'synthetic'; end
    if ~isfield(dc,'roarmIps'),        dc.roarmIps        = cfg.roarmIps; end
    if ~isfield(dc,'roarmEpisodes'),   dc.roarmEpisodes   = 6;           end
    if ~isfield(dc,'roarmStepsPerEp'), dc.roarmStepsPerEp = 40;          end
    if ~isfield(dc,'roarmDt'),         dc.roarmDt         = 0.20;        end
    if ~isfield(dc,'roarmSpeed'),      dc.roarmSpeed      = 5.0;         end
    cfg.datasetCfg = dc;

    % Synchronized collector (online live collection)
    if ~isfield(cfg,'syncCfg'), cfg.syncCfg = struct(); end
    sc = cfg.syncCfg;
    if ~isfield(sc,'roarmIps'),        sc.roarmIps        = cfg.roarmIps; end
    if ~isfield(sc,'roarmDt'),         sc.roarmDt         = 0.20;        end
    if ~isfield(sc,'roarmEpisodes'),   sc.roarmEpisodes   = 10;          end
    if ~isfield(sc,'roarmStepsPerEp'), sc.roarmStepsPerEp = 50;          end
    if ~isfield(sc,'roarmSpeed'),      sc.roarmSpeed      = 5;           end
    cfg.syncCfg = sc;

    % Collision avoidance
    if ~isfield(cfg,'collisionCfg'), cfg.collisionCfg = struct(); end
    cc = cfg.collisionCfg;

    if ~isfield(cc,'collect'), cc.collect = struct(); end
    col = cc.collect;
    if ~isfield(col,'mode'),         col.mode         = 'synthetic';     end
    if ~isfield(col,'episodes'),     col.episodes     = 20;              end
    if ~isfield(col,'stepsPerEp'),   col.stepsPerEp   = 50;             end
    if ~isfield(col,'roarmDt'),      col.roarmDt      = 0.20;           end
    if ~isfield(col,'roarmIps'),     col.roarmIps     = cfg.roarmIps;    end
    if ~isfield(col,'roarmSpeed'),   col.roarmSpeed   = 5;               end
    if ~isfield(col,'kukaPosition'), col.kukaPosition = [1000,0,300];    end
    if ~isfield(col,'connectKuka'),  col.connectKuka  = false;           end
    cc.collect = col;

    if ~isfield(cc,'train'), cc.train = struct(); end
    tr = cc.train;
    if ~isfield(tr,'maxEpisodes'),   tr.maxEpisodes   = 500;            end
    if ~isfield(tr,'maxSteps'),      tr.maxSteps      = 50;             end
    if ~isfield(tr,'dt'),            tr.dt            = 0.20;           end
    if ~isfield(tr,'actionLimit'),   tr.actionLimit   = 30;             end
    if ~isfield(tr,'hiddenSizes'),   tr.hiddenSizes   = [128,128];      end
    if ~isfield(tr,'actorLR'),       tr.actorLR       = 1e-4;           end
    if ~isfield(tr,'criticLR'),      tr.criticLR      = 1e-3;           end
    if ~isfield(tr,'gamma'),         tr.gamma         = 0.99;           end
    if ~isfield(tr,'bufferLength'),  tr.bufferLength  = 100000;         end
    if ~isfield(tr,'miniBatchSize'), tr.miniBatchSize = 64;             end
    if ~isfield(tr,'kukaPosition'),  tr.kukaPosition  = [1000,0,300];   end
    cc.train = tr;

    if ~isfield(cc,'eval'), cc.eval = struct(); end
    ev = cc.eval;
    if ~isfield(ev,'numEpisodes'), ev.numEpisodes = 100; end
    if ~isfield(ev,'maxSteps'),    ev.maxSteps    = 50;  end
    cc.eval = ev;

    cfg.collisionCfg = cc;
end