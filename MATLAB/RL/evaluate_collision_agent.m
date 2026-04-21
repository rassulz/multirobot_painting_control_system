function evaluate_collision_agent(agentPath, cfg)
%EVALUATE_COLLISION_AGENT  Placeholder evaluation entry point.
%   evaluate_collision_agent(agentPath, cfg) loads the marker file written
%   by train_collision_agent() and prints a summary so the pipeline can
%   complete Phase 4 cleanly. Replace with a real episode loop.

    if nargin < 2, cfg = struct(); end
    if ~isfield(cfg, 'numEpisodes'), cfg.numEpisodes = 100; end
    if ~isfield(cfg, 'maxSteps'),    cfg.maxSteps    = 50;  end

    fprintf('  evaluate_collision_agent: %s\n', agentPath);
    fprintf('    numEpisodes=%d, maxSteps=%d\n', cfg.numEpisodes, cfg.maxSteps);

    if ~isfile(agentPath)
        warning('Agent file not found: %s', agentPath);
        return;
    end

    S = load(agentPath);
    if isfield(S, 'placeholder')
        fprintf('    Loaded placeholder agent (%s).\n', S.placeholder.timestamp);
    end
end
