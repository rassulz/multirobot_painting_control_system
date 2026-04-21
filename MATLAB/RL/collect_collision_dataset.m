function T = collect_collision_dataset(cfg)
%COLLECT_COLLISION_DATASET  Synthetic 3-robot collision dataset.
%   T = collect_collision_dataset(cfg) generates a table simulating the
%   KUKA at a fixed Cartesian position while two RoArms move randomly
%   around the workspace. Each row records the positions of all three
%   robots and the minimum pairwise distance.
%
%   cfg fields (with defaults):
%     episodes      = 20    number of episodes
%     stepsPerEp    = 50    steps per episode
%     roarmDt       = 0.20  step interval (s)
%     kukaPosition  = [1000 0 300]  fixed KUKA TCP position (mm)
%     roarmIps      = {'192.168.1.192','192.168.1.101'}  (unused in synthetic)
%     roarmSpeed    = 5     (unused in synthetic)
%     mode          = 'synthetic'

    if nargin < 1, cfg = struct(); end
    if ~isfield(cfg, 'episodes'),     cfg.episodes     = 20;             end
    if ~isfield(cfg, 'stepsPerEp'),   cfg.stepsPerEp   = 50;             end
    if ~isfield(cfg, 'roarmDt'),      cfg.roarmDt      = 0.20;           end
    if ~isfield(cfg, 'kukaPosition'), cfg.kukaPosition = [1000, 0, 300]; end

    nTotal = cfg.episodes * cfg.stepsPerEp;
    episode = zeros(nTotal,1);
    step    = zeros(nTotal,1);
    time_s  = zeros(nTotal,1);
    kuka_x  = zeros(nTotal,1); kuka_y = zeros(nTotal,1); kuka_z = zeros(nTotal,1);
    r1_x    = zeros(nTotal,1); r1_y   = zeros(nTotal,1); r1_z   = zeros(nTotal,1);
    r2_x    = zeros(nTotal,1); r2_y   = zeros(nTotal,1); r2_z   = zeros(nTotal,1);
    min_dist = zeros(nTotal,1);

    idx = 0;
    for ep = 1:cfg.episodes
        % Random starting points for each RoArm relative to KUKA.
        r1 = cfg.kukaPosition + [randn*250, randn*250, 100 + abs(randn)*150];
        r2 = cfg.kukaPosition + [randn*250, randn*250, 100 + abs(randn)*150];

        for k = 1:cfg.stepsPerEp
            idx = idx + 1;

            % Random walk drives the RoArms around the KUKA workspace.
            r1 = r1 + randn(1,3) * 8;
            r2 = r2 + randn(1,3) * 8;

            d_k_r1 = norm(r1 - cfg.kukaPosition);
            d_k_r2 = norm(r2 - cfg.kukaPosition);
            d_r1_r2 = norm(r1 - r2);

            episode(idx) = ep;
            step(idx)    = k;
            time_s(idx)  = (k - 1) * cfg.roarmDt;
            kuka_x(idx)  = cfg.kukaPosition(1);
            kuka_y(idx)  = cfg.kukaPosition(2);
            kuka_z(idx)  = cfg.kukaPosition(3);
            r1_x(idx)    = r1(1); r1_y(idx) = r1(2); r1_z(idx) = r1(3);
            r2_x(idx)    = r2(1); r2_y(idx) = r2(2); r2_z(idx) = r2(3);
            min_dist(idx) = min([d_k_r1, d_k_r2, d_r1_r2]);
        end
    end

    T = table(episode, step, time_s, kuka_x, kuka_y, kuka_z, ...
        r1_x, r1_y, r1_z, r2_x, r2_y, r2_z, min_dist);

    fprintf('  collect_collision_dataset: %d rows generated.\n', height(T));
end
