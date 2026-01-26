%% RO-ARM M2 ROBOT MOTION CONTROL - QUICK REFERENCE GUIDE
% 
% This guide provides quick examples for common motion control tasks

%% ========================================================================
%  BASIC SETUP AND INITIALIZATION
% ========================================================================

% Create robot object (without serial connection - for simulation)
robot = RoarmM2_MotionControl();

% Create robot object (with serial connection - for actual robot)
% robot = RoarmM2_MotionControl('COM3');  % Replace COM3 with your port


%% ========================================================================
%  JOINT SPACE MOTION (Direct angle commands)
% ========================================================================

% Example 1: Move to home (all zeros)
robot.moveHome();

% Example 2: Move specific joint angles
robot.moveToJointAngles([45, 30, -20, 0, 45, 0]);
%                         J1  J2  J3   J4 J5  J6 (degrees)

% Example 3: Incremental motion - move from current position
currentAngles = robot.getCurrentJointAngles();
newAngles = currentAngles + [10, 5, -5, 0, 0, 0];  % Add 10° to J1, 5° to J2, etc.
robot.moveToJointAngles(newAngles);

% Example 4: Individual joint motion
angles = robot.getCurrentJointAngles();
angles(1) = angles(1) + 15;  % Rotate joint 1 by 15 degrees
robot.moveToJointAngles(angles);


%% ========================================================================
%  CARTESIAN SPACE MOTION (XYZ coordinates)
% ========================================================================

% Example 1: Move to specific cartesian position
position = [0.3, 0.2, 0.25];      % [x, y, z] in meters
orientation = [0, 0, 0];           % [roll, pitch, yaw] in degrees
robot.moveToCartesian(position, orientation);

% Example 2: Move with different orientations
robot.moveToCartesian([0.4, 0.0, 0.3], [0, 0, 90]);

% Example 3: Cartesian incremental motion
currentPose = robot.getCurrentEndEffectorPose();
newPose = currentPose + [0.05, 0.0, 0.0, 0, 0, 0];  % Move 5cm in X direction
robot.moveToCartesian(newPose(1:3), newPose(4:6));


%% ========================================================================
%  FORWARD & INVERSE KINEMATICS
% ========================================================================

% Example 1: Get end-effector position (Forward Kinematics)
angles = [30, 45, -30, 0, 45, 0];
endEffectorPose = robot.forwardKinematics(angles);
x = endEffectorPose(1);
y = endEffectorPose(2);
z = endEffectorPose(3);
disp(['Position: X=' num2str(x) ' Y=' num2str(y) ' Z=' num2str(z)]);

% Example 2: Get joint angles for a position (Inverse Kinematics)
targetPosition = [0.35, 0.15, 0.3];
targetOrientation = [0, 0, 0];
jointAngles = robot.inverseKinematics(targetPosition, targetOrientation);

% Example 3: Get current end-effector pose
currentPose = robot.getCurrentEndEffectorPose();
disp(currentPose);


%% ========================================================================
%  TRAJECTORY GENERATION & EXECUTION
% ========================================================================

% Example 1: Linear trajectory
startAngles = [0, 0, 0, 0, 0, 0];
endAngles = [45, 30, -20, 0, 30, 0];
trajectory = robot.generateLinearTrajectory(startAngles, endAngles);
robot.executeTrajectory(trajectory);

% Example 2: S-curve trajectory (smooth motion with accel/decel)
trajectory = robot.generateSCurveTrajectory(startAngles, endAngles);
robot.executeTrajectory(trajectory);

% Example 3: Multi-waypoint trajectory
waypoint1 = [0, 0, 0, 0, 0, 0];
waypoint2 = [30, 45, -30, 0, 45, 0];
waypoint3 = [0, 0, 0, 0, 0, 0];

trajectory1 = robot.generateSCurveTrajectory(waypoint1, waypoint2);
trajectory2 = robot.generateSCurveTrajectory(waypoint2, waypoint3);
fullTrajectory = [trajectory1; trajectory2];

robot.executeTrajectory(fullTrajectory);


%% ========================================================================
%  MOTION CONSTRAINTS & VALIDATION
% ========================================================================

% Example 1: Check if angles are within limits
angles = [30, 45, -30, 0, 45, 0];
isValid = robot.checkJointLimits(angles);
disp(['Angles valid: ' num2str(isValid)]);

% Example 2: Get robot limits
disp('Lower limits: ' num2str(robot.JointLimits_Lower));
disp('Upper limits: ' num2str(robot.JointLimits_Upper));
disp('Max velocities: ' num2str(robot.MaxVelocity));
disp('Max accelerations: ' num2str(robot.MaxAcceleration));


%% ========================================================================
%  STATUS & MONITORING
% ========================================================================

% Example 1: Get current joint angles
angles = robot.getCurrentJointAngles();
disp(angles);

% Example 2: Get current end-effector position
pose = robot.getCurrentEndEffectorPose();
disp(['X: ' num2str(pose(1)) ', Y: ' num2str(pose(2)) ', Z: ' num2str(pose(3))]);

% Example 3: Get link lengths and DOF
disp(['DOF: ' num2str(robot.DOF)]);
disp(['Link lengths: ' num2str(robot.LinkLengths)]);


%% ========================================================================
%  EMERGENCY OPERATIONS
% ========================================================================

% Example 1: Emergency stop
robot.emergencyStop();

% Example 2: Return to home position
robot.moveHome();


%% ========================================================================
%  CLEANUP
% ========================================================================

% Clear robot object and close serial connection
clear robot;


%% ========================================================================
%  PRACTICAL EXAMPLES
% ========================================================================

% EXAMPLE A: Pick and Place Motion
%--------------------------------------
% robot = RoarmM2_MotionControl('COM3');
% 
% % Move to pick location
% robot.moveToCartesian([0.3, 0.2, 0.25], [0, 0, 0]);
% pause(0.5);
% % (grasp object here)
% 
% % Move to place location
% robot.moveToCartesian([0.4, 0.2, 0.25], [0, 0, 0]);
% pause(0.5);
% % (release object here)
% 
% % Return home
% robot.moveHome();
% clear robot;


% EXAMPLE B: Circular Motion
%--------------------------------------
% robot = RoarmM2_MotionControl('COM3');
% center = [0.35, 0.0, 0.3];
% radius = 0.1;
% numPoints = 36;
% 
% for angle = linspace(0, 2*pi, numPoints)
%     x = center(1) + radius * cos(angle);
%     y = center(2) + radius * sin(angle);
%     z = center(3);
%     robot.moveToCartesian([x, y, z], [0, 0, 0]);
% end
% 
% robot.moveHome();
% clear robot;


% EXAMPLE C: Follow Predefined Path
%--------------------------------------
% robot = RoarmM2_MotionControl('COM3');
% 
% % Define waypoints
% waypoints = [
%     0,    0,    0,    0,    0,    0;      % Home
%     30,   45,  -30,   0,   45,   0;      % Point 1
%     60,   30,  -45,   0,   30,   0;      % Point 2
%     30,   45,  -30,   0,   45,   0;      % Point 3
%     0,    0,    0,    0,    0,    0;      % Home
% ];
% 
% % Execute trajectory through all waypoints
% for i = 1:size(waypoints, 1)-1
%     trajectory = robot.generateSCurveTrajectory(waypoints(i,:), waypoints(i+1,:));
%     robot.executeTrajectory(trajectory);
%     pause(0.5);
% end
% 
% clear robot;
