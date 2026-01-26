%% ROARM M2 COMPLETE EXAMPLE - FULL FEATURE DEMONSTRATION
% This example shows how to use all features of RoarmM2_MotionControl_v2
% 
% Hardware: Waveshare RoArm M2
% Reference: https://github.com/waveshareteam/roarm_m2
% Documentation: https://www.waveshare.com/wiki/RoArm-M2-S

clear all; close all; clc;

fprintf('==========================================================\n');
fprintf('  ROARM M2 COMPREHENSIVE CONTROL EXAMPLE\n');
fprintf('==========================================================\n\n');

%% Step 1: Initialize Robot
% =========================================================================
fprintf('STEP 1: Initializing Robot\n');
fprintf('----------------------------------------------------------\n');

% For simulation (no hardware):
robot = RoarmM2_MotionControl_v2();

% For actual hardware connection:
% robot = RoarmM2_MotionControl_v2('COM3');
% (Replace 'COM3' with your actual COM port)

fprintf('Robot initialized in %s mode\n\n', ...
    iif(robot.IsConnected, 'CONNECTED', 'SIMULATION'));

%% Step 2: Display Robot Information
% =========================================================================
fprintf('STEP 2: Robot Specifications\n');
fprintf('----------------------------------------------------------\n');
fprintf('Degrees of Freedom: %d\n', robot.DOF);
fprintf('Joint Names: %s\n', sprintf('%s, ', robot.JointNames{:}));
fprintf('Home Position: [%.1f° %.1f° %.1f° %.1f°]\n', robot.HomeAngles);
fprintf('\nJoint Limits (degrees):\n');
for i = 1:robot.DOF
    fprintf('  %s: [%.1f, %.1f]\n', robot.JointNames{i}, ...
        robot.JointLimits_Lower(i), robot.JointLimits_Upper(i));
end
fprintf('\nWorkspace:\n');
fprintf('  Radius: ~%d mm\n', robot.WorkspaceRadius);
fprintf('  Communication: UART 115200 baud\n');
fprintf('  Protocol: JSON commands\n\n');

%% Step 3: Move to Home Position
% =========================================================================
fprintf('STEP 3: Moving to Home Position\n');
fprintf('----------------------------------------------------------\n');
robot.moveHome();
pause(1);
fprintf('✓ Reached home position\n\n');

%% Step 4: Joint-by-Joint Movement
% =========================================================================
fprintf('STEP 4: Joint-by-Joint Movement\n');
fprintf('----------------------------------------------------------\n');

fprintf('Moving Base (Joint 1):\n');
fprintf('  45° left...\n');
robot.moveSingleJoint(1, 45);
pause(0.5);

fprintf('  90° left...\n');
robot.moveSingleJoint(1, 90);
pause(0.5);

fprintf('  Center...\n');
robot.moveSingleJoint(1, 0);
pause(0.5);

fprintf('Moving Shoulder (Joint 2):\n');
fprintf('  Forward 45°...\n');
robot.moveSingleJoint(2, 45);
pause(0.5);

fprintf('  Backward 45°...\n');
robot.moveSingleJoint(2, -45);
pause(0.5);

fprintf('  Center...\n');
robot.moveSingleJoint(2, 0);
pause(0.5);

fprintf('Moving Elbow (Joint 3):\n');
fprintf('  Down 150°...\n');
robot.moveSingleJoint(3, 150);
pause(0.5);

fprintf('  Up 45°...\n');
robot.moveSingleJoint(3, 45);
pause(0.5);

fprintf('  Home (90°)...\n');
robot.moveSingleJoint(3, 90);
pause(0.5);

fprintf('✓ Joint movement complete\n\n');

%% Step 5: Multi-Joint Coordinated Motion
% =========================================================================
fprintf('STEP 5: Multi-Joint Coordinated Motion\n');
fprintf('----------------------------------------------------------\n');

positions = {
    [0, 0, 90, 180], 'Home position';
    [30, 30, 100, 150], 'Reach forward low';
    [60, 0, 80, 120], 'Reach right';
    [-60, 0, 80, 120], 'Reach left';
    [0, -30, 110, 170], 'Reach up forward';
};

for i = 1:size(positions, 1)
    angles = positions{i, 1};
    description = positions{i, 2};
    fprintf('Moving to: %s\n', description);
    robot.moveToJointAngles(angles);
    pause(0.8);
end

fprintf('✓ Multi-joint motion complete\n\n');

%% Step 6: Speed and Acceleration Control
% =========================================================================
fprintf('STEP 6: Speed and Acceleration Control\n');
fprintf('----------------------------------------------------------\n');

fprintf('Moving with SLOW speed (speed=10, accel=5):\n');
robot.moveSingleJoint(1, 45, 10, 5);
pause(1);

fprintf('Moving with MEDIUM speed (speed=50, accel=10):\n');
robot.moveSingleJoint(1, -45, 50, 10);
pause(0.5);

fprintf('Moving with FAST speed (speed=80, accel=20):\n');
robot.moveSingleJoint(1, 0, 80, 20);
pause(0.5);

fprintf('✓ Speed control demonstration complete\n\n');

%% Step 7: Cartesian Position Control
% =========================================================================
fprintf('STEP 7: Cartesian Position Control (XYZ Coordinates)\n');
fprintf('----------------------------------------------------------\n');
fprintf('Coordinate System (right-hand rule):\n');
fprintf('  X-axis: positive = forward (away from base)\n');
fprintf('  Y-axis: positive = left (when facing forward)\n');
fprintf('  Z-axis: positive = up\n\n');

cartesian_positions = {
    [250, 0, 300], 'Forward center';
    [200, 150, 280], 'Forward and left';
    [200, -150, 280], 'Forward and right';
    [300, 0, 350], 'Forward and high';
    [250, 0, 250], 'Forward and low';
};

for i = 1:size(cartesian_positions, 1)
    pos = cartesian_positions{i, 1};
    desc = cartesian_positions{i, 2};
    fprintf('Position: %s\n  X=%.0fmm, Y=%.0fmm, Z=%.0fmm\n', desc, pos(1), pos(2), pos(3));
    robot.moveToCartesian(pos);
    pause(0.8);
end

fprintf('Return to home...\n');
robot.moveHome();
pause(0.5);

fprintf('✓ Cartesian motion complete\n\n');

%% Step 8: End Effector (Gripper) Control
% =========================================================================
fprintf('STEP 8: End Effector Control\n');
fprintf('----------------------------------------------------------\n');

gripper_states = {
    45, 'Fully open (45°)';
    90, 'Semi-open (90°)';
    135, 'Semi-closed (135°)';
    160, 'Nearly closed (160°)';
    180, 'Fully closed (180°)';
};

for i = 1:size(gripper_states, 1)
    angle = gripper_states{i, 1};
    state = gripper_states{i, 2};
    fprintf('Gripper: %s\n', state);
    robot.controlEndEffector(angle);
    pause(0.4);
end

fprintf('✓ Gripper demonstration complete\n\n');

%% Step 9: Automated Pick and Place Sequence
% =========================================================================
fprintf('STEP 9: Automated Pick & Place Sequence\n');
fprintf('----------------------------------------------------------\n');

fprintf('Scenario: Pick object from location A, place at location B\n\n');

% Location A (pickup)
pickup_loc = [250, 100, 350];
% Location B (place)
place_loc = [-150, 0, 300];

fprintf('1. Move to home position\n');
robot.moveHome();
pause(1);

fprintf('2. Move above pickup location [%.0f, %.0f, %.0f]\n', pickup_loc(1), pickup_loc(2), pickup_loc(3));
robot.moveToCartesian(pickup_loc);
pause(1);

fprintf('3. Lower to pickup level\n');
robot.moveToCartesian([pickup_loc(1), pickup_loc(2), pickup_loc(3)-150]);
pause(0.5);

fprintf('4. Close gripper (pick object)\n');
robot.controlEndEffector(180);
pause(0.5);

fprintf('5. Lift object\n');
robot.moveToCartesian(pickup_loc);
pause(1);

fprintf('6. Move to placement location [%.0f, %.0f, %.0f]\n', place_loc(1), place_loc(2), place_loc(3));
robot.moveToCartesian(place_loc);
pause(1);

fprintf('7. Lower to placement level\n');
robot.moveToCartesian([place_loc(1), place_loc(2), place_loc(3)-100]);
pause(0.5);

fprintf('8. Open gripper (release object)\n');
robot.controlEndEffector(45);
pause(0.5);

fprintf('9. Retract (move back up)\n');
robot.moveToCartesian(place_loc);
pause(0.5);

fprintf('10. Return to home\n');
robot.moveHome();
pause(0.5);

fprintf('✓ Pick & place sequence complete\n\n');

%% Step 10: Complex Motion Patterns
% =========================================================================
fprintf('STEP 10: Complex Motion Patterns\n');
fprintf('----------------------------------------------------------\n');

% Pattern 1: Circular sweep in horizontal plane
fprintf('Pattern 1: Horizontal circular motion (XY plane)\n');
center = [300, 0, 300];
radius = 100;
num_points = 12;

for angle = 0:360/num_points:360
    x = center(1) + radius * cosd(angle);
    y = center(2) + radius * sind(angle);
    z = center(3);
    robot.moveToCartesian([x, y, z]);
    pause(0.3);
end

fprintf('✓ Circular motion complete\n\n');

% Pattern 2: Vertical sweep
fprintf('Pattern 2: Vertical sweep (Z axis variation)\n');
base_pos = [300, 0];
for z = 200:30:400
    robot.moveToCartesian([base_pos(1), base_pos(2), z]);
    pause(0.3);
end

fprintf('✓ Vertical sweep complete\n\n');

fprintf('Return to home...\n');
robot.moveHome();
pause(0.5);

%% Step 11: Robot Status Display
% =========================================================================
fprintf('STEP 11: Current Robot Status\n');
fprintf('----------------------------------------------------------\n');

status = robot.getStatus();

fprintf('Connection Status: %s\n', iif(status.IsConnected, 'CONNECTED', 'SIMULATION'));
fprintf('Current Joint Angles (degrees):\n');
for i = 1:robot.DOF
    fprintf('  %12s: %8.2f°\n', status.JointNames{i}, status.CurrentAngles(i));
end

fprintf('\nTarget Joint Angles (degrees):\n');
for i = 1:robot.DOF
    fprintf('  %12s: %8.2f°\n', status.JointNames{i}, status.TargetAngles(i));
end

fprintf('\n✓ Status display complete\n\n');

%% Step 12: Error Handling Examples
% =========================================================================
fprintf('STEP 12: Error Handling\n');
fprintf('----------------------------------------------------------\n');

% Test with valid angles
fprintf('Testing valid joint angles...\n');
valid_angles = [30, 45, 100, 150];
is_valid = robot.checkJointLimits(valid_angles);
fprintf('Angles [%.0f, %.0f, %.0f, %.0f]: %s\n', valid_angles(1), valid_angles(2), ...
    valid_angles(3), valid_angles(4), iif(is_valid, 'VALID', 'INVALID'));

% Test with invalid angles
fprintf('Testing invalid joint angles (exceeds limits)...\n');
invalid_angles = [200, 45, 100, 150];  % Base exceeds 180°
is_valid = robot.checkJointLimits(invalid_angles);
fprintf('Angles [%.0f, %.0f, %.0f, %.0f]: %s\n', invalid_angles(1), invalid_angles(2), ...
    invalid_angles(3), invalid_angles(4), iif(is_valid, 'VALID', 'INVALID'));

fprintf('✓ Error handling demonstration complete\n\n');

%% Step 13: Cleanup
% =========================================================================
fprintf('STEP 13: Cleanup\n');
fprintf('----------------------------------------------------------\n');

fprintf('Disconnecting from robot...\n');
robot.disconnect();

fprintf('Clearing robot object...\n');
clear robot;

fprintf('✓ Cleanup complete\n\n');

fprintf('==========================================================\n');
fprintf('  DEMONSTRATION COMPLETE\n');
fprintf('==========================================================\n\n');

fprintf('Summary:\n');
fprintf('  - Initialized RoArm M2 robot\n');
fprintf('  - Performed joint control\n');
fprintf('  - Performed Cartesian control\n');
fprintf('  - Controlled gripper\n');
fprintf('  - Demonstrated pick & place\n');
fprintf('  - Showed complex motion patterns\n');
fprintf('  - Verified error handling\n\n');

fprintf('Next Steps:\n');
fprintf('  1. Connect to actual hardware by specifying COM port\n');
fprintf('  2. Use RoarmM2_QuickReference.m for quick command reference\n');
fprintf('  3. Use RoarmM2_Demo.m for shorter examples\n');
fprintf('  4. Refer to https://github.com/waveshareteam/roarm_m2 for more info\n\n');

%% Helper function
function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end
