%% RO-ARM M2 WIFI/HTTP CONTROL - QUICK REFERENCE
% This guide shows how to control the RoArm M2 robot via WiFi/HTTP/IP address
% instead of serial communication
%
% WiFi Connection:
% - Default AP Mode: IP 192.168.4.1
% - WiFi Name: RoArm-M2
% - Password: 12345678
% - Or use STA mode with your local WiFi

%% ========================================================================
%  SETUP AND CONNECTION
% ========================================================================

% Connect via AP Mode (Default - 192.168.4.1)
robot = RoarmM2_MotionControl_WiFi('192.168.4.1');

% Connect via STA Mode (Your local network)
% First, connect robot to your WiFi via web interface
% Then find the IP address from robot's OLED display
robot = RoarmM2_MotionControl_WiFi('192.168.1.100');  % Example IP

% Simulation Mode (No WiFi needed)
robot = RoarmM2_MotionControl_WiFi();

% Check connection status
disp(robot.getStatus());

% Disconnect
robot.disconnect();


%% ========================================================================
%  BASIC MOTION COMMANDS
% ========================================================================

% Move to home position
robot.moveHome();

% Move all joints (Base, Shoulder, Elbow, EndEffector)
robot.moveToJointAngles([0, 0, 90, 180]);
robot.moveToJointAngles([45, 30, 120, 135]);
robot.moveToJointAngles([-90, -45, 60, 90]);


%% ========================================================================
%  SINGLE JOINT CONTROL
% ========================================================================

% Move a single joint: moveSingleJoint(joint_number, angle_degrees, speed, accel)
% Joint 1: Base (-180 to 180)
% Joint 2: Shoulder (-90 to 90)
% Joint 3: Elbow (-45 to 180)
% Joint 4: EndEffector (45 to 315)

robot.moveSingleJoint(1, 45);           % Base to 45 degrees
robot.moveSingleJoint(2, 30, 50, 10);   % Shoulder to 30 degrees
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
%  TORQUE CONTROL
% =========================================================================

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
disp(sprintf('Connected: %d', status.IsConnected));
disp(sprintf('IP: %s', status.IPAddress));

% Get feedback from robot (current position and torques)
feedback = robot.getFeedback();


%% ========================================================================
%  PRACTICAL EXAMPLES
% ========================================================================

% EXAMPLE 1: SIMPLE REACH FORWARD
%--------------------------------------
robot.moveHome();
robot.moveToCartesian([300, 0, 300]);
pause(1);
robot.moveHome();

% EXAMPLE 2: SWEEP BASE FROM LEFT TO RIGHT
%--------------------------------------
robot.moveHome();
for angle = -90:10:90
    robot.moveSingleJoint(1, angle);  % Base rotation
    pause(0.3);
end
robot.moveHome();

% EXAMPLE 3: PICK AND PLACE
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

% EXAMPLE 4: CIRCULAR MOTION IN XY PLANE
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

% EXAMPLE 5: VERTICAL REACH WITH VARYING HEIGHT
%--------------------------------------
robot.moveHome();
for z = 200:50:350
    robot.moveToCartesian([300, 0, z]);
    pause(0.5);
end
robot.moveHome();


%% ========================================================================
%  WIFI CONFIGURATION
% =========================================================================

% AP MODE (Default - Recommended for quick setup)
% --------------------------------
% 1. Power on the robot
% 2. Look for WiFi network "RoArm-M2"
% 3. Connect with password: 12345678
% 4. IP Address: 192.168.4.1
% 5. Use in MATLAB:
%    robot = RoarmM2_MotionControl_WiFi('192.168.4.1');

% STA MODE (Connect to Your Local WiFi)
% --------------------------------
% 1. Power on robot and wait for boot
% 2. Connect to WiFi network "RoArm-M2" (temporary)
% 3. Open browser: http://192.168.4.1
% 4. Go to WiFi settings in web interface
% 5. Select your local WiFi network and enter password
% 6. Robot will reconnect to your network
% 7. Find the IP address on robot's OLED display
% 8. Use in MATLAB:
%    robot = RoarmM2_MotionControl_WiFi('192.168.1.100');  % Your IP


%% ========================================================================
%  COORDINATE SYSTEM
% =========================================================================

% Right-hand rule convention:
%   X-axis: Positive = forward (away from base)
%   Y-axis: Positive = left (when facing forward)
%   Z-axis: Positive = up (vertically)
%
% Origin: Base of the robot
% Units: Millimeters (mm)


%% ========================================================================
%  JOINT ANGLE RANGES (DEGREES)
% =========================================================================

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
% =========================================================================

% Disconnect and clear
robot.disconnect();
clear robot;


%% ========================================================================
%  TROUBLESHOOTING
% =========================================================================

% PROBLEM: Cannot connect to robot
% SOLUTION:
%   1. Check robot is powered on
%   2. Check robot displays OLED information
%   3. Verify WiFi connection name and IP address
%   4. Try AP mode first (192.168.4.1)
%   5. Restart robot and MATLAB

% PROBLEM: Commands not received
% SOLUTION:
%   1. Check IsConnected status: robot.IsConnected
%   2. Verify IP address is correct
%   3. Check robot is still powered on
%   4. Try smaller movements first
%   5. Check robot OLED display for IP changes

% PROBLEM: Gripper not moving
% SOLUTION:
%   1. Check End Effector type (Gripper vs Wrist)
%   2. Verify angle is in range (45-315)
%   3. Check gripper mechanical connection
%   4. Try manual control via web interface first

%% ========================================================================
%  HTTP REQUEST FORMAT (FOR REFERENCE)
% =========================================================================

% MATLAB sends commands as HTTP GET requests:
% http://192.168.4.1/js?json={"T":122,"b":0,"s":30,"e":90,"h":180,"spd":50,"acc":10}
%
% Command Structure:
%   T: Command type ID
%   b: Base angle (degrees)
%   s: Shoulder angle (degrees)
%   e: Elbow angle (degrees)
%   h: Hand/EndEffector angle (degrees)
%   spd: Speed (0-100)
%   acc: Acceleration (0-254)
%
% Common Commands:
%   T=100: Home position
%   T=121: Single joint control
%   T=122: All joints control
%   T=106: EndEffector control
%   T=1041: Cartesian position control
%   T=210: Torque control
%   T=105: Get feedback
