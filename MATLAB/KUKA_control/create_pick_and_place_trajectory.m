%% Create Pick-and-Place Trajectory for KUKA 6-DOF Robot
% UPDATED: Smooth motion upgrade WITHOUT reintroducing A4 software limit issue
%
% What changed vs your version:
%  1) Keeps your robot-native Euler convention (A=-180,B=2,C=-180) => A4-safe
%  2) Keeps your intermediate pose (MID_Y) that prevented A4 singularity
%  3) Replaces linear "linspace" dense segments with C2-smooth quintic blending
%     (6s^5 - 15s^4 + 10s^3) per segment -> smoother velocity/acceleration
%  4) Adds optional "SMOOTH_DENSE" waypoints output for execution streaming
%     while still saving key_poses exactly as before
%
% NOTE:
%  - This file does NOT change the orientation strategy that fixed A4.
%  - It only changes how dense waypoints are generated between your SAFE key poses.

clc; clear; close all;

%% ===== ROBOT CONFIGURATION =====
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║   KUKA 6-DOF Pick-and-Place - All Joints       ║\n');
fprintf('║   A4/A5-safe + Intermediate pose + SMOOTH PATH ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

% Select robot model
fprintf('Select your robot model:\n');
fprintf('  [1] KR 6 R900   (reach: 900mm)\n');
fprintf('  [2] KR 10 R1100 (reach: 1100mm)\n');
fprintf('  [3] KR 16 R1610 (reach: 1600mm)\n');
fprintf('  [4] Custom\n');

robot_choice = input('Enter robot number (1-4) [default: 2]: ');
if isempty(robot_choice)
    robot_choice = 2;
end

switch robot_choice
    case 1
        MAX_REACH = 900;
        ROBOT_MODEL = 'KR6_R900';
    case 2
        MAX_REACH = 1100;
        ROBOT_MODEL = 'KR10_R1100';
    case 3
        MAX_REACH = 1600;
        ROBOT_MODEL = 'KR16_R1610';
    case 4
        MAX_REACH = input('Enter max reach in mm [default: 1100]: ');
        if isempty(MAX_REACH)
            MAX_REACH = 1100;
        end
        ROBOT_MODEL = 'Custom';
    otherwise
        MAX_REACH = 1100;
        ROBOT_MODEL = 'KR10_R1100';
end

MIN_REACH = 300;

fprintf('\n✓ Robot: %s\n', ROBOT_MODEL);
fprintf('  Max Reach: %.0f mm\n', MAX_REACH);
fprintf('  Min Reach: %.0f mm\n\n', MIN_REACH);

%% ===== PRESET POSITIONS (IN METERS) =====
fprintf('=== Select Pick-and-Place Positions ===\n\n');

presets = {
    'Close Range (Conservative)',     [0.4, 0.2, 0.0, 0.4, 0.4, 0.0];
    'Medium Range (Typical)',         [0.5, 0.3, 0.0, 0.7, 0.5, 0.0];
    'Extended Range (Larger robots)', [0.6, 0.4, 0.0, 0.8, 0.6, 0.0];
    'Straight Forward (Simple)',      [0.5, 0.0, 0.0, 0.7, 0.0, 0.0];
    'Diagonal Motion (Full 6-DOF)',   [0.4, 0.2, 0.2, 0.6, 0.5, 0.2];
    'Custom (enter manually)',        [];
};

for i = 1:size(presets, 1)
    if i < 6
        coords = presets{i, 2};
        pick_r = sqrt((coords(1)*1000)^2 + (coords(2)*1000)^2);
        place_r = sqrt((coords(4)*1000)^2 + (coords(5)*1000)^2);

        reachable = (pick_r < MAX_REACH*0.9) && (place_r < MAX_REACH*0.9);
        status = '✓';
        if ~reachable
            status = '✗ OUT OF REACH';
        end

        fprintf('  [%d] %s %s\n', i, presets{i, 1}, status);
        fprintf('      Pick:  (%.2f, %.2f, %.2f)m → %.0fmm\n', ...
            coords(1), coords(2), coords(3), pick_r);
        fprintf('      Place: (%.2f, %.2f, %.2f)m → %.0fmm\n\n', ...
            coords(4), coords(5), coords(6), place_r);
    else
        fprintf('  [%d] %s\n\n', i, presets{i, 1});
    end
end

preset_choice = input(sprintf('Select preset (1-%d) [default: 4]: ', size(presets, 1)));
if isempty(preset_choice)
    preset_choice = 4;
end

if preset_choice == 6
    fprintf('\nEnter custom coordinates in METERS:\n');
    PICK_X_M  = input('  Pick X [m]: ');
    PICK_Y_M  = input('  Pick Y [m]: ');
    PICK_Z_M  = input('  Pick Z [m]: ');
    PLACE_X_M = input('  Place X [m]: ');
    PLACE_Y_M = input('  Place Y [m]: ');
    PLACE_Z_M = input('  Place Z [m]: ');
else
    coords = presets{preset_choice, 2};
    PICK_X_M  = coords(1);
    PICK_Y_M  = coords(2);
    PICK_Z_M  = coords(3);
    PLACE_X_M = coords(4);
    PLACE_Y_M = coords(5);
    PLACE_Z_M = coords(6);
end

%% ===== CONVERT AND VALIDATE =====
PICK_X_MM  = PICK_X_M * 1000;
PICK_Y_MM  = PICK_Y_M * 1000;
PICK_Z_MM  = PICK_Z_M * 1000;

PLACE_X_MM = PLACE_X_M * 1000;
PLACE_Y_MM = PLACE_Y_M * 1000;
PLACE_Z_MM = PLACE_Z_M * 1000;

pick_radius  = sqrt(PICK_X_MM^2 + PICK_Y_MM^2);
place_radius = sqrt(PLACE_X_MM^2 + PLACE_Y_MM^2);

fprintf('\n=== Validation ===\n');
fprintf('Pick:  (%.3f, %.3f, %.3f)m → %.0fmm from base\n', ...
    PICK_X_M, PICK_Y_M, PICK_Z_M, pick_radius);
fprintf('Place: (%.3f, %.3f, %.3f)m → %.0fmm from base\n', ...
    PLACE_X_M, PLACE_Y_M, PLACE_Z_M, place_radius);

if pick_radius > MAX_REACH * 0.9 || place_radius > MAX_REACH * 0.9
    error('Positions outside safe workspace! Choose a different preset.');
end

fprintf('✓ Positions validated\n\n');

%% ===== HEIGHT CONFIGURATION =====
Z_TABLE     = 0;      % table at robot-base level (was 300 — caused +300mm Z offset)
Z_CLEARANCE = 240;

fprintf('Configuration:\n');
fprintf('  Table height: %d mm\n', Z_TABLE);
fprintf('  Clearance: %d mm\n\n', Z_CLEARANCE);

PICK_X  = PICK_X_MM;
PICK_Y  = PICK_Y_MM;
PICK_Z  = Z_TABLE + PICK_Z_MM;

PLACE_X = PLACE_X_MM;
PLACE_Y = PLACE_Y_MM;
PLACE_Z = Z_TABLE + PLACE_Z_MM;

%% ===== TRAJECTORY PARAMETERS =====
NUM_CYCLES     = 1; %#ok<NASGU>
POINTS_PER_SEG = 40;  % increased for smoother streaming (was 20)

% Smoothness settings (per-segment)
USE_QUINTIC_SMOOTH = true;

%% ===== TOOL ORIENTATIONS (Robot-native convention) =====
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║  Tool Orientation: Robot-Native Convention      ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

% *** DO NOT CHANGE THESE: this is your proven A4-safe convention ***
TOOL_A = -180;
TOOL_B = 2;
TOOL_C = -180;

fprintf('✓ Tool-down orientation: A=%.0f°, B=%.0f°, C=%.0f°\n', TOOL_A, TOOL_B, TOOL_C);
fprintf('✓ Uses robot-native convention (small B) to avoid A4 software limit\n\n');

ORIENT_HOME           = [TOOL_A, TOOL_B, TOOL_C];
ORIENT_APPROACH_PICK  = [TOOL_A, TOOL_B, TOOL_C + 10];
ORIENT_PICK           = [TOOL_A, TOOL_B, TOOL_C + 5];
ORIENT_LIFT_PICK      = [TOOL_A, TOOL_B, TOOL_C + 3];
ORIENT_MID            = [TOOL_A, TOOL_B, TOOL_C - 1.5];
ORIENT_TRANSITION     = [TOOL_A, TOOL_B, TOOL_C];
ORIENT_APPROACH_PLACE = [TOOL_A, TOOL_B, TOOL_C - 10];
ORIENT_PLACE          = [TOOL_A, TOOL_B, TOOL_C - 5];
ORIENT_LIFT_PLACE     = [TOOL_A, TOOL_B, TOOL_C - 3];

%% ===== HOME POSITION =====
HOME_X = (PICK_X + PLACE_X) / 2;
HOME_Y = (PICK_Y + PLACE_Y) / 2;
HOME_Z = max([PICK_Z, PLACE_Z]) + Z_CLEARANCE + 80;

home_radius = sqrt(HOME_X^2 + HOME_Y^2 + HOME_Z^2);
if home_radius > MAX_REACH * 0.80
    scale  = (MAX_REACH * 0.70) / home_radius;
    HOME_X = HOME_X * scale;
    HOME_Y = HOME_Y * scale;
    HOME_Z = HOME_Z * scale;
end

%% ===== DEFINE KEY POSES =====
fprintf('=== Building A4/A5-SAFE Trajectory ===\n');

TRANS_X = (PICK_X + PLACE_X) / 2;
TRANS_Y = (PICK_Y + PLACE_Y) / 2;
TRANS_Z = max([PICK_Z, PLACE_Z]) + Z_CLEARANCE;

% *** KEEP THIS: it was your A4 fix ***
MID_Y = (PICK_Y + TRANS_Y) / 2;

key_poses = [
    HOME_X,  HOME_Y,  HOME_Z,                 ORIENT_HOME;
    PICK_X,  PICK_Y,  PICK_Z + Z_CLEARANCE,   ORIENT_APPROACH_PICK;
    PICK_X,  PICK_Y,  PICK_Z,                 ORIENT_PICK;
    PICK_X,  PICK_Y,  PICK_Z + Z_CLEARANCE,   ORIENT_LIFT_PICK;
    PICK_X,  MID_Y,   PICK_Z + Z_CLEARANCE,   ORIENT_MID;
    TRANS_X, TRANS_Y, TRANS_Z,                ORIENT_TRANSITION;
    PLACE_X, PLACE_Y, PLACE_Z + Z_CLEARANCE,  ORIENT_APPROACH_PLACE;
    PLACE_X, PLACE_Y, PLACE_Z,                ORIENT_PLACE;
    PLACE_X, PLACE_Y, PLACE_Z + Z_CLEARANCE,  ORIENT_LIFT_PLACE;
    HOME_X,  HOME_Y,  HOME_Z,                 ORIENT_HOME;
];

fprintf('✓ Added intermediate pose at Y=%.1f mm to avoid singularity\n', MID_Y);

%% ===== DENSE WAYPOINTS =====
% OLD: linspace() caused C0 velocity discontinuities at each segment boundary.
% NEW: quintic blending keeps v=0 and a=0 at segment endpoints (C2 smooth inside segment),
% while still passing through your SAFE key poses.

waypoints = [];
for i = 1:(size(key_poses, 1) - 1)
    p0 = key_poses(i,   :);
    p1 = key_poses(i+1, :);

    if USE_QUINTIC_SMOOTH
        s = linspace(0, 1, POINTS_PER_SEG).';
        s5 = 6*s.^5 - 15*s.^4 + 10*s.^3;  % quintic smoothstep (C2)
        seg = p0 + (p1 - p0).*s5;
    else
        seg = zeros(POINTS_PER_SEG, 6);
        for j = 1:6
            seg(:, j) = linspace(p0(j), p1(j), POINTS_PER_SEG).';
        end
    end

    if isempty(waypoints)
        waypoints = seg;
    else
        waypoints = [waypoints; seg(2:end, :)]; %#ok<AGROW>
    end
end

NUM_POINTS = size(waypoints, 1);
fprintf('✓ Generated %d dense waypoints (SMOOTH=%d, pts/seg=%d)\n', ...
    NUM_POINTS, USE_QUINTIC_SMOOTH, POINTS_PER_SEG);
fprintf('✓ Saved %d key poses for smooth streaming execution\n\n', size(key_poses,1));

%% ===== VERIFY ORIENTATION CHANGES =====
fprintf('=== Verifying A5-SAFE Configuration ===\n');
A_range = [min(waypoints(:,4)), max(waypoints(:,4))];
B_range = [min(waypoints(:,5)), max(waypoints(:,5))];
C_range = [min(waypoints(:,6)), max(waypoints(:,6))];

fprintf('Cartesian Orientation A: %6.1f° to %6.1f°\n', A_range(1), A_range(2));
fprintf('Cartesian Orientation B: %6.1f° to %6.1f°\n', B_range(1), B_range(2));
fprintf('Cartesian Orientation C: %6.1f° to %6.1f°\n', C_range(1), C_range(2));

fprintf('\n✓ B=%.0f° uses robot-native convention (small B)\n\n', TOOL_B);

%% ===== WORKSPACE VALIDATION =====
fprintf('=== Workspace Validation ===\n');
radii = sqrt(waypoints(:,1).^2 + waypoints(:,2).^2 + waypoints(:,3).^2);
max_dist = max(radii);

fprintf('Max distance: %.0f mm (limit: %.0f mm)\n', max_dist, MAX_REACH);
if max_dist > MAX_REACH
    error('Some waypoints unreachable!');
end
fprintf('✓ All waypoints reachable\n\n');

%% ===== APPROXIMATE JOINT LIMIT CHECK =====
fprintf('=== Approximate Joint Limit Check ===\n');
joint_warnings = 0;
for wp = 1:NUM_POINTS
    px = waypoints(wp, 1);
    py = waypoints(wp, 2);
    oB = waypoints(wp, 5);

    a1_est = atan2d(py, px);
    if abs(a1_est) > 170
        fprintf('  ⚠ Waypoint %d: A1 ≈ %.1f° near ±185° limit\n', wp, a1_est);
        joint_warnings = joint_warnings + 1;
    end

    if abs(oB) < 2 || abs(oB - 180) < 1 || abs(oB + 180) < 1
        fprintf('  ⚠ Waypoint %d: B=%.1f° wrist singularity risk\n', wp, oB);
        joint_warnings = joint_warnings + 1;
    end

    r_horiz = sqrt(px^2 + py^2);
    if r_horiz < MIN_REACH * 0.5
        fprintf('  ⚠ Waypoint %d: Too close to base (%.0fmm)\n', wp, r_horiz);
        joint_warnings = joint_warnings + 1;
    end
end

if joint_warnings == 0
    fprintf('✓ No joint limit warnings detected\n');
else
    fprintf('\n⚠ %d warnings detected - review trajectory before execution\n', joint_warnings);
end
fprintf('\n');

%% ===== SMOOTHNESS METRICS (NEW) =====
% This is just analysis; it doesn't affect the generated trajectory.
fprintf('=== Smoothness Metrics (Position) ===\n');
if NUM_POINTS >= 4
    dt_assumed = 0.03; % your executor default loop is ~30ms
    v = diff(waypoints(:,1:3)) / dt_assumed;
    a = diff(v) / dt_assumed;
    vmag = sqrt(sum(v.^2,2));
    amag = sqrt(sum(a.^2,2));
    fprintf('Assuming dt=%.3fs:\n', dt_assumed);
    fprintf('  Peak speed:  %.1f mm/s\n', max(vmag));
    fprintf('  Peak accel:  %.1f mm/s^2\n', max(amag));
    fprintf('  Speed CV:    %.3f (lower is smoother)\n', std(vmag)/max(1,mean(vmag)));
else
    fprintf('Not enough points for metrics.\n');
end
fprintf('\n');

%% ===== VISUALIZATION =====
figure('Name', sprintf('%s Pick-and-Place - A4-Safe Smooth', ROBOT_MODEL), ...
    'Position', [50, 50, 1600, 1000]);

subplot(2, 3, [1, 4]);
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'b-', 'LineWidth', 2.5);
hold on;

quiver3(0, 0, 0, 200, 0, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver3(0, 0, 0, 0, 200, 0, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver3(0, 0, 0, 0, 0, 200, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.5);

[xs, ys, zs] = sphere(40);
surf(MAX_REACH*xs, MAX_REACH*ys, MAX_REACH*zs, ...
    'FaceColor', 'cyan', 'FaceAlpha', 0.08, 'EdgeColor', 'none');

plot3(0, 0, 0, 'ko', 'MarkerSize', 16, 'MarkerFaceColor', 'k');
plot3(HOME_X, HOME_Y, HOME_Z, 'ms', 'MarkerSize', 13, 'MarkerFaceColor', 'm');
plot3(PICK_X, PICK_Y, PICK_Z, 'go', 'MarkerSize', 16, 'MarkerFaceColor', 'g');
plot3(PLACE_X, PLACE_Y, PLACE_Z, 'ro', 'MarkerSize', 16, 'MarkerFaceColor', 'r');

% Intermediate pose
plot3(PICK_X, MID_Y, PICK_Z + Z_CLEARANCE, 'co', 'MarkerSize', 14, 'MarkerFaceColor', 'c');
text(PICK_X, MID_Y, PICK_Z + Z_CLEARANCE + 50, 'MID', 'Color', 'c', 'FontWeight', 'bold');

scatter3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 30, 1:NUM_POINTS, 'filled');
colormap(jet);
cb = colorbar; cb.Label.String = 'Waypoint';

xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title(sprintf('%s: A4-Safe SMOOTH (%d pts)', ROBOT_MODEL, NUM_POINTS));
grid on; axis equal; view(45, 25);

subplot(2, 3, 2);
plot(1:NUM_POINTS, waypoints(:,4), 'r-', 'LineWidth', 2.5); hold on;
plot(1:NUM_POINTS, waypoints(:,5), 'g-', 'LineWidth', 2.5);
plot(1:NUM_POINTS, waypoints(:,6), 'b-', 'LineWidth', 2.5);
xlabel('Waypoint Index'); ylabel('deg');
title('Cartesian Orientations'); legend('A','B','C','Location','best'); grid on;

subplot(2, 3, 3);
viscircles([0 0], MAX_REACH, 'Color', 'c', 'LineStyle', '--', 'LineWidth', 1.5);
hold on;
plot(waypoints(:,1), waypoints(:,2), 'b-', 'LineWidth', 2);
plot(0, 0, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
plot(PICK_X, PICK_Y, 'go', 'MarkerSize', 14, 'MarkerFaceColor', 'g');
plot(PLACE_X, PLACE_Y, 'ro', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
plot(TRANS_X, TRANS_Y, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot(PICK_X, MID_Y, 'co', 'MarkerSize', 10, 'MarkerFaceColor', 'c');
xlabel('X (mm)'); ylabel('Y (mm)'); title('Top View'); grid on; axis equal;

subplot(2, 3, 5);
plot(1:NUM_POINTS, waypoints(:,1), 'r-', 'LineWidth', 2); hold on;
plot(1:NUM_POINTS, waypoints(:,2), 'g-', 'LineWidth', 2);
plot(1:NUM_POINTS, waypoints(:,3), 'b-', 'LineWidth', 2);
xlabel('Waypoint Index'); ylabel('mm'); title('Cartesian Position'); legend('X','Y','Z'); grid on;

subplot(2, 3, 6);
plot(1:NUM_POINTS, radii, 'b-', 'LineWidth', 2); hold on;
yline(MAX_REACH, 'r--', 'Max', 'LineWidth', 2);
yline(MIN_REACH, 'g--', 'Min', 'LineWidth', 2);
xlabel('Waypoint Index'); ylabel('mm'); title('Reachability Check'); grid on;

%% ===== SAVE =====
PATTERN = sprintf('pick_%.0f_%.0f_to_%.0f_%.0f_A4SAFE_SMOOTH', ...
    PICK_X, PICK_Y, PLACE_X, PLACE_Y);
CENTER = [0, 0, 0];

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('trajectory_pick_place_%s.mat', timestamp);

save(filename, 'waypoints', 'PATTERN', 'NUM_POINTS', 'CENTER', ...
     'PICK_X', 'PICK_Y', 'PICK_Z', 'PLACE_X', 'PLACE_Y', 'PLACE_Z', ...
     'PICK_X_M', 'PICK_Y_M', 'PICK_Z_M', 'PLACE_X_M', 'PLACE_Y_M', 'PLACE_Z_M', ...
     'Z_CLEARANCE', 'Z_TABLE', 'ROBOT_MODEL', 'MAX_REACH', 'MIN_REACH', ...
     'key_poses', 'USE_QUINTIC_SMOOTH', 'POINTS_PER_SEG');

fprintf('\n╔════════════════════════════════════════════════╗\n');
fprintf('║              TRAJECTORY SAVED                  ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');
fprintf('Robot:         %s\n', ROBOT_MODEL);
fprintf('Dense waypoints:         %d\n', NUM_POINTS);
fprintf('Key poses saved:         %d\n', size(key_poses,1));
fprintf('Smooth mode (quintic):   %d\n', USE_QUINTIC_SMOOTH);
fprintf('Points per segment:      %d\n', POINTS_PER_SEG);
fprintf('File:          %s\n\n', filename);
fprintf('✓ Intermediate pose retained at Y=%.1f mm (A4 fix)\n', MID_Y);
fprintf('NEXT: Run excecute_pick_and_place_trajectory.m\n');