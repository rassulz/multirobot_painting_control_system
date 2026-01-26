%% RO-ARM M2 WIFI/HTTP MOTION CONTROL DEMONSTRATION
% This script demonstrates WiFi-based control of the RoArm M2 robot
% using HTTP/IP address instead of serial communication
%
% WiFi Connection Modes:
% 1. AP Mode (Default): Connect directly to robot WiFi "RoArm-M2"
%    IP: 192.168.4.1
%    Password: 12345678
%
% 2. STA Mode: Connect to your local WiFi network
%    IP: Assigned by your router (check robot OLED display)

clear; clc; close all;

disp('========================================');
disp('   RoArm M2 WiFi Motion Control        ');
disp('========================================');
disp(' ');

%% 1. CONNECTION OPTIONS
% =========================================================================
disp('1. CONNECTION OPTIONS');
disp('-----------------------------------------');
disp(' ');
disp('Default AP Mode (recommended for quick setup):');
disp('  IP: 192.168.4.1');
disp('  WiFi: RoArm-M2');
disp('  Password: 12345678');
disp(' ');
disp('Alternative - STA Mode (local network):');
disp('  IP: Check robot OLED display after WiFi connection');
disp(' ');

%% 2. INITIALIZE ROBOT (CHOOSE ONE)
% =========================================================================
disp('2. INITIALIZING ROBOT');
disp('-----------------------------------------');

% OPTION A: AP Mode (Default - 192.168.4.1)
disp('Connecting via AP Mode (192.168.4.1)...');
robot = RoarmM2_MotionControl_WiFi('192.168.4.1');

% OPTION B: STA Mode (Uncomment if using local WiFi)
% disp('Connecting via STA Mode (192.168.1.100)...');
% robot = RoarmM2_MotionControl_WiFi('192.168.1.100');

% OPTION C: Simulation Mode (No WiFi connection)
% disp('Simulation mode (no WiFi)...');
% robot = RoarmM2_MotionControl_WiFi();

pause(1);
disp(' ');

%% 3. MOVE TO HOME POSITION
% =========================================================================
disp('3. MOVE TO HOME POSITION');
disp('-----------------------------------------');
disp('Moving to home: [0°, 0°, 90°, 180°]...');
robot.moveHome();
pause(1);
disp(' ');

%% 4. MOVE ALL JOINTS
% =========================================================================
disp('4. MOVE ALL JOINTS TOGETHER');
disp('-----------------------------------------');

disp('Example 1: Forward reach');
robot.moveToJointAngles([0, 30, 120, 135]);
pause(1);

disp('Example 2: Rotate base and move arm');
robot.moveToJointAngles([45, -20, 80, 180]);
pause(1);

disp('Example 3: Return to home');
robot.moveToJointAngles(robot.HomeAngles);
pause(1);
disp(' ');

%% 5. MOVE INDIVIDUAL JOINTS
% =========================================================================
disp('5. MOVE INDIVIDUAL JOINTS');
disp('-----------------------------------------');

disp('Rotating base left...');
robot.moveSingleJoint(1, 90);
pause(0.5);

disp('Rotating base right...');
robot.moveSingleJoint(1, -90);
pause(0.5);

disp('Centering base...');
robot.moveSingleJoint(1, 0);
pause(0.5);

disp('Moving shoulder...');
robot.moveSingleJoint(2, 45);
pause(0.3);
robot.moveSingleJoint(2, 0);
pause(0.5);
disp(' ');

%% 6. CARTESIAN POSITION CONTROL
% =========================================================================
disp('6. CARTESIAN POSITION CONTROL (XYZ)');
disp('-----------------------------------------');

disp('Moving forward (X=250mm, Y=0mm, Z=300mm)...');
robot.moveToCartesian([250, 0, 300]);
pause(1);

disp('Moving to left (X=200mm, Y=150mm, Z=250mm)...');
robot.moveToCartesian([200, 150, 250]);
pause(1);

disp('Moving to right (X=200mm, Y=-150mm, Z=250mm)...');
robot.moveToCartesian([200, -150, 250]);
pause(1);

disp('Higher reach (X=300mm, Y=0mm, Z=350mm)...');
robot.moveToCartesian([300, 0, 350]);
pause(1);

disp('Returning to home...');
robot.moveHome();
disp(' ');

%% 7. GRIPPER CONTROL
% =========================================================================
disp('7. GRIPPER CONTROL');
disp('-----------------------------------------');

disp('Opening gripper (45°)...');
robot.controlEndEffector(45);
pause(0.5);

disp('Partially closing (120°)...');
robot.controlEndEffector(120);
pause(0.5);

disp('Fully closing (180°)...');
robot.controlEndEffector(180);
pause(0.5);
disp(' ');

%% 8. PICK AND PLACE SEQUENCE
% =========================================================================
disp('8. PICK & PLACE SEQUENCE');
disp('-----------------------------------------');

disp('Step 1: Home position');
robot.moveHome();
pause(0.5);

disp('Step 2: Move above pickup location');
robot.moveToCartesian([250, 100, 350]);
pause(0.5);

disp('Step 3: Lower to pickup height');
robot.moveToCartesian([250, 100, 200]);
pause(0.3);

disp('Step 4: Close gripper (pick)');
robot.controlEndEffector(180);
pause(0.5);

disp('Step 5: Lift with object');
robot.moveToCartesian([250, 100, 350]);
pause(0.5);

disp('Step 6: Move to place location');
robot.moveToCartesian([-150, 0, 300]);
pause(0.5);

disp('Step 7: Lower to place height');
robot.moveToCartesian([-150, 0, 200]);
pause(0.3);

disp('Step 8: Open gripper (release)');
robot.controlEndEffector(45);
pause(0.5);

disp('Step 9: Return to home');
robot.moveHome();
disp(' ');

%% 9. ROBOT STATUS
% =========================================================================
disp('9. ROBOT STATUS');
disp('-----------------------------------------');

status = robot.getStatus();
disp(sprintf('Connected: %d', status.IsConnected));
if ~isempty(status.IPAddress)
    disp(sprintf('IP Address: %s', status.IPAddress));
end
disp('Current joint angles (degrees):');
for i = 1:4
    disp(sprintf('  %s: %.2f°', status.JointNames{i}, status.CurrentAngles(i)));
end
disp(' ');

%% 10. DEMONSTRATION COMPLETE
% =========================================================================
disp('DEMONSTRATION COMPLETE');
disp('========================================');
disp(' ');
disp('WiFi/HTTP Communication:');
disp('  • Protocol: HTTP GET requests');
disp('  • Default IP: 192.168.4.1 (AP mode)');
disp('  • WiFi Name: RoArm-M2');
disp('  • Password: 12345678');
disp(' ');
disp('Robot Specifications:');
disp('  • DOF: 4 (Base, Shoulder, Elbow, EndEffector)');
disp('  • Base: -180° to 180°');
disp('  • Shoulder: -90° to 90°');
disp('  • Elbow: -45° to 180°');
disp('  • EndEffector: 45° to 315°');
disp('  • Workspace: ~500mm radius');
disp(' ');

% Cleanup
robot.disconnect();
clear robot;
disp('Robot disconnected. Program complete.');
