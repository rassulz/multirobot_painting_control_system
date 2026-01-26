%% RO-ARM M2 ROBOT - MOTION CONTROL DEMONSTRATION
%

clear all; close all; clc;

%% 1. ROBOT INITIALIZATION
% =========================================================================
disp('=== RO-ARM M2 Robot Motion Control ===');
disp('Initializing robot...');

% Create robot object (update COM port as needed)
% For simulation without hardware, comment out the serial port
robot = RoarmM2_MotionControl();  % No serial connection for simulation
% robot = RoarmM2_MotionControl('COM3');  % Use this with actual hardware

disp('Robot initialized successfully!');

%% 2. HOME POSITION
% =========================================================================
disp(' ');
disp('Moving to HOME position...');
robot.moveHome();
pause(1);
disp('Reached home position');

%% 3. JOINT SPACE MOTION - SINGLE AXIS
% =========================================================================
disp(' ');
disp('Joint Space Motion - Moving Joint 1');
angles = [45, 0, 0, 0, 0, 0];  % Move joint 1 to 45 degrees
robot.moveToJointAngles(angles);
disp(['Current angles: ' num2str(robot.getCurrentJointAngles())]);

%% 4. MULTI-JOINT COORDINATED MOTION
% =========================================================================
disp(' ');
disp('Multi-Joint Coordinated Motion');
angles = [30, 45, -30, 0, 45, 0];
robot.moveToJointAngles(angles);
disp(['Current angles: ' num2str(robot.getCurrentJointAngles())]);

%% 5. FORWARD KINEMATICS - GET END-EFFECTOR POSITION
% =========================================================================
disp(' ');
disp('Computing Forward Kinematics...');
currentPose = robot.getCurrentEndEffectorPose();
disp(' ');
disp('End-Effector Pose:');
disp(['X: ' num2str(currentPose(1), '%.4f') ' m']);
disp(['Y: ' num2str(currentPose(2), '%.4f') ' m']);
disp(['Z: ' num2str(currentPose(3), '%.4f') ' m']);
disp(['Roll: ' num2str(currentPose(4), '%.2f') ' deg']);
disp(['Pitch: ' num2str(currentPose(5), '%.2f') ' deg']);
disp(['Yaw: ' num2str(currentPose(6), '%.2f') ' deg']);

%% 6. CARTESIAN SPACE MOTION (Inverse Kinematics)
% =========================================================================
disp(' ');
disp('Cartesian Space Motion - Moving to XYZ position');
targetPosition = [0.3, 0.2, 0.25];  % [x, y, z] in meters
targetOrientation = [0, 0, 0];      % [roll, pitch, yaw] in degrees

disp(['Target Position: X=' num2str(targetPosition(1)) ...
      ' Y=' num2str(targetPosition(2)) ...
      ' Z=' num2str(targetPosition(3))]);

robot.moveToCartesian(targetPosition, targetOrientation);
pause(1);

% Verify end-effector reached target
actualPose = robot.getCurrentEndEffectorPose();
positionError = norm(actualPose(1:3) - targetPosition);
disp(['Position Error: ' num2str(positionError, '%.4f') ' m']);

%% 7. TRAJECTORY GENERATION AND EXECUTION
% =========================================================================
disp(' ');
disp('Generating and executing S-curve trajectory...');

% Define waypoints for the trajectory
startAngles = robot.getCurrentJointAngles();
midAngles = [20, 30, -20, 0, 30, 0];
endAngles = [0, 0, 0, 0, 0, 0];

% Generate complete trajectory through waypoints
trajectory1 = robot.generateSCurveTrajectory(startAngles, midAngles);
trajectory2 = robot.generateSCurveTrajectory(midAngles, endAngles);
completeTrajectory = [trajectory1; trajectory2];

% Execute trajectory
robot.executeTrajectory(completeTrajectory);
disp('Trajectory execution completed');

%% 8. CIRCULAR MOTION (DEMO)
% =========================================================================
disp(' ');
disp('Generating circular motion in XY plane...');

% Generate circular path
center = [0.3, 0.0, 0.25];
radius = 0.1;
numPoints = 50;
angles_circle = linspace(0, 2*pi, numPoints);

for i = 1:numPoints
    x = center(1) + radius * cos(angles_circle(i));
    y = center(2) + radius * sin(angles_circle(i));
    z = center(3);
    
    % Compute IK for each point (this will execute motion)
    % Uncomment below to enable actual circular motion:
    % robot.moveToCartesian([x, y, z], [0, 0, 0]);
    
    if i == 1 || mod(i, 10) == 0
        disp(['Point ' num2str(i) '/' num2str(numPoints) ...
              ' - Position: (' num2str(x, '%.3f') ', ' ...
              num2str(y, '%.3f') ', ' num2str(z, '%.3f') ')']);
    end
end

%% 9. MOTION CONSTRAINTS CHECK
% =========================================================================
disp(' ');
disp('Testing Motion Constraints...');

testAngles1 = [30, 45, -30, 0, 45, 0];
testAngles2 = [200, 45, -30, 0, 45, 0];  % Out of limits

isValid1 = robot.checkJointLimits(testAngles1);
isValid2 = robot.checkJointLimits(testAngles2);

disp(['Test angles 1 valid: ' num2str(isValid1)]);
disp(['Test angles 2 valid (out of limits): ' num2str(isValid2)]);

%% 10. FINAL STATUS
% =========================================================================
disp(' ');
disp('=== Motion Control Demonstration Complete ===');
disp('Final robot status:');
disp(['Current Joint Angles: ' num2str(robot.getCurrentJointAngles())]);
disp(['Current End-Effector Pose: ' num2str(robot.getCurrentEndEffectorPose())]);

% Return to home
disp(' ');
disp('Returning to home position...');
robot.moveHome();
disp('Done!');

%% CLEANUP
% =========================================================================
% Clear robot object and close serial connection
clear robot;
disp('Robot connection closed');
