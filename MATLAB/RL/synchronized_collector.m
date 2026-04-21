function T = synchronized_collector(cfg, kukaTcp) %#ok<INUSD>
%SYNCHRONIZED_COLLECTOR  Online stub for live multi-robot data collection.
%   T = synchronized_collector(cfg, kukaTcp) reads positions of the KUKA
%   and the two RoArms in lock-step. The current build forwards to the
%   synthetic generator so the pipeline runs end-to-end. Replace the body
%   with HTTP calls to the RoArms and KUKAVARPROXY reads when hardware
%   is available.

    if nargin < 1, cfg = struct(); end
    if ~isfield(cfg, 'roarmEpisodes'),   cfg.roarmEpisodes   = 10;   end
    if ~isfield(cfg, 'roarmStepsPerEp'), cfg.roarmStepsPerEp = 50;   end
    if ~isfield(cfg, 'roarmDt'),         cfg.roarmDt         = 0.20; end

    fprintf('  synchronized_collector: stub forwarding to synthetic.\n');

    syntheticCfg = struct( ...
        'episodes',     cfg.roarmEpisodes, ...
        'stepsPerEp',   cfg.roarmStepsPerEp, ...
        'roarmDt',      cfg.roarmDt, ...
        'kukaPosition', [1000, 0, 300]);

    T = collect_collision_dataset(syntheticCfg);
end
