%% RO-ARM M2 ROBOT - QUICK REFERENCE GUIDE
% This guide shows quick examples for controlling the Waveshare RoArm M2
% Robot Specifications: 4-DOF, JSON protocol, serial communication (115200 baud)
% Reference: https://github.com/waveshareteam/roarm_m2

%% ========================================================================
%  BASIC SETUP
% ========================================================================

% Create robot object WITH hardware connection
robot = RoarmM2_MotionControl_v2('COM3');  % Change COM3 to your port

% Create robot object WITHOUT hardware (simulation)
robot = RoarmM2_MotionControl_v2();


%% ========================================================================
%  BASIC MOTION COMMANDS
% ========================================================================

% Move all joints together (Base, Shoulder, Elbow, EndEffector in degrees)
robot.moveToJointAngles([0, 0, 90, 180]);      % Home position
robot.moveToJointAngles([45, 30, 120, 135]);   % Custom angles
robot.moveToJointAngles([-90, -45, 60, 90]);   % Different position

% Move to home (initial position)
robot.moveHome();


%% ========================================================================
%  SINGLE JOINT CONTROL
% ========================================================================

% Move a single joint: moveSingleJoint(joint_number, angle_degrees, speed, accel)
% Joint 1: Base (-180 to 180)
% Joint 2: Shoulder (-90 to 90)
% Joint 3: Elbow (-45 to 180)
% Joint 4: EndEffector (45 to 315)

robot.moveSingleJoint(1, 45);           % Base to 45 degrees
robot.moveSingleJoint(2, 30, 50, 10);   % Shoulder to 30 degrees, speed=50, accel=10
robot.moveSingleJoint(3, 100);          % Elbow to 100 degrees
robot.moveSingleJoint(4, 90);           % EndEffector to 90 degrees


%% ========================================================================
%  CARTESIAN COORDINATES (XYZ POSITION)
% ========================================================================

% Move to XYZ position (in millimeters)
% X: forward/backward (-500 to 500 mm)
% Y: left/right (-500 to 500 mm)
% Z: up/down (0 to 500 mm)

robot.moveToCartesian([250, 0, 300]);           % Forward
robot.moveToCartesian([200, 150, 250]);         % Forward and left
robot.moveToCartesian([200, -150, 250]);        % Forward and right
robot.moveToCartesian([300, 0, 350]);           % Higher reach


%% ========================================================================
%  END EFFECTOR (GRIPPER) CONTROL
% ========================================================================

% Control gripper position in degrees
% Gripper mode: 180=closed, lower values=more open
%   45° = fully open
%   90° = semi-open
%   135° = semi-closed
%   180° = fully closed

robot.controlEndEffector(45);       % Open gripper
robot.controlEndEffector(120);       % Partially close
robot.controlEndEffector(180);       % Fully close


%% ========================================================================
%  SPEED AND ACCELERATION CONTROL
% ========================================================================

% Speed: 0-100 (0 or high values = fast, lower = slower)
% Acceleration: 0-254 (smaller = smoother, 0 = max accel)

robot.moveSingleJoint(1, 45, 10, 5);    % Slow and smooth
robot.moveSingleJoint(1, 45, 80, 20);   % Fast movement
robot.moveSingleJoint(1, 45, 50, 10);   % Medium speed


%% ========================================================================
%  CONTINUOUS MOVEMENT
% ========================================================================

% Continuous movement: moveContiguous(mode, axis, direction, speed)
% mode: 0=angle control, 1=coordinate control
% axis: 1-4 (which joint/axis)
% direction: 0=stop, 1=increase, 2=decrease
% speed: 0-20 (larger = faster)

robot.moveContiguous(0, 1, 1, 10);      % Rotate base left
robot.moveContiguous(0, 1, 2, 10);      % Rotate base right
robot.moveContiguous(0, 1, 0, 0);       % Stop base movement


%% ========================================================================
%  TORQUE CONTROL
% ========================================================================

% Enable/disable joint locking
robot.setTorque(1);   % Lock torque (prevent manual movement)
robot.setTorque(0);   % Unlock torque (allow manual movement)


%% ========================================================================
%  FEEDBACK AND STATUS
% ========================================================================

% Get current joint angles
angles = robot.getCurrentJointAngles();
disp(angles);

% Get robot status
status = robot.getStatus();
disp(status.IsConnected);
disp(status.CurrentAngles);

% Get feedback from robot (current position and torques)
feedback = robot.getFeedback();


%% ========================================================================
%  CONNECTION MANAGEMENT
% ========================================================================

% Connect to robot
robot.connect('COM3');

% Disconnect from robot
robot.disconnect();


%% ========================================================================
%  PRACTICAL EXAMPLES
% ========================================================================

% EXAMPLE 1: Simple reach forward
%--------------------------------------
robot.moveHome();
robot.moveToCartesian([300, 0, 300]);
pause(1);
robot.moveHome();

% EXAMPLE 2: Sweep left to right
%--------------------------------------
robot.moveHome();
for angle = -90:10:90
    robot.moveSingleJoint(1, angle);  % Base rotation
    pause(0.3);
end
robot.moveHome();

% EXAMPLE 3: Pick and place
%--------------------------------------
robot.moveHome();
pause(0.5);

% Move to pickup location
robot.moveToCartesian([250, 100, 350]);  % Above object
pause(0.5);
robot.moveToCartesian([250, 100, 200]);  % At object height
pause(0.3);

% Pick object
robot.controlEndEffector(180);  % Close gripper
pause(0.5);

% Lift and move
robot.moveToCartesian([250, 100, 350]);  % Lift
pause(0.5);
robot.moveToCartesian([-150, 0, 300]);   % Move to place location
pause(0.5);

% Place object
robot.moveToCartesian([-150, 0, 200]);   % Lower
pause(0.3);
robot.controlEndEffector(45);             % Open gripper
pause(0.3);

% Return home
robot.moveHome();

% EXAMPLE 4: Circular motion in XY plane
%--------------------------------------
robot.moveHome();
center = [300, 0, 300];
radius = 100;
for angle = 0:10:360
    x = center(1) + radius * cosd(angle);
    y = center(2) + radius * sind(angle);
    z = center(3);
    robot.moveToCartesian([x, y, z]);
    pause(0.2);
end
robot.moveHome();

% EXAMPLE 5: Vertical reach with varying height
%--------------------------------------
robot.moveHome();
for z = 200:50:350
    robot.moveToCartesian([300, 0, z]);
    pause(0.5);
end
robot.moveHome();


%% ========================================================================
%  JOINT ANGLE RANGES (DEGREES)
% ========================================================================

% Base (Joint 1): -180 to 180 degrees
%   - 0: center
%   - Positive: rotate left
%   - Negative: rotate right

% Shoulder (Joint 2): -90 to 90 degrees
%   - 0: center
%   - Positive: rotate forward
%   - Negative: rotate backward

% Elbow (Joint 3): -45 to 180 degrees
%   - 90: home position
%   - Higher values: elbow down
%   - Lower values: elbow up

% EndEffector (Joint 4): 45 to 315 degrees (Gripper mode)
%   - 180: fully closed
%   - 45: fully open


%% ========================================================================
%  CLEANUP
% ========================================================================

% Disconnect and clear
robot.disconnect();
clear robot;
