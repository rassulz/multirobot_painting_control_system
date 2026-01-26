% RoArm M2 MATLAB Motion Control - README
% ===========================================================
%
% This package provides comprehensive MATLAB control for the Waveshare RoArm M2 robot
% Reference: https://github.com/waveshareteam/roarm_m2
%
% ===========================================================
% FILES INCLUDED
% ===========================================================
%
% 1. RoarmM2_MotionControl_v2.m
%    - Main robot control class
%    - All methods for motion control
%    - JSON command formatting and sending
%    - Serial communication handling
%
% 2. RoarmM2_Demo.m (old version - for reference)
%    - Original demo file
%
% 3. RoarmM2_CompleteExample.m (RECOMMENDED)
%    - Comprehensive step-by-step example
%    - Shows all features with detailed output
%    - Best for learning and understanding
%
% 4. RoarmM2_QuickReference.m
%    - Quick command reference
%    - Copy-paste ready examples
%    - Best for quick lookups
%
% 5. README.m (this file)
%    - Documentation and setup instructions
%
% ===========================================================
% GETTING STARTED
% ===========================================================
%
% SETUP:
% 1. Download files to your MATLAB working directory
% 2. Find your COM port (Device Manager on Windows)
% 3. Run examples
%
% STEP 1: Run Complete Example (RECOMMENDED)
%   >> RoarmM2_CompleteExample
%   This shows all features with detailed demonstrations
%
% STEP 2: Try Quick Examples
%   >> RoarmM2_QuickReference
%   Then copy and modify commands for your needs
%
% STEP 3: Create Your Own Script
%   >> robot = RoarmM2_MotionControl_v2('COM3');  % Your COM port
%   >> robot.moveHome();
%   >> robot.moveToJointAngles([0, 30, 90, 180]);
%   >> robot.disconnect();
%
% ===========================================================
% ROBOT SPECIFICATIONS
% ===========================================================
%
% DOF: 4 (Base, Shoulder, Elbow, End Effector)
%
% Joint Limits (degrees):
%   Base:         -180 to 180  (360° rotation)
%   Shoulder:      -90 to  90  (180° range)
%   Elbow:         -45 to 180  (225° range)
%   EndEffector:    45 to 315  (270° range)
%
% Workspace:
%   Radius: ~500 mm
%   Max reach: ~500 mm forward
%   Height range: 0-500 mm
%
% Communication:
%   Protocol: JSON over UART/USB
%   Baud Rate: 115200
%   Connector: USB Type-C
%
% ===========================================================
% QUICK START CODE
% ===========================================================
%
% EXAMPLE 1: CONNECT AND MOVE HOME
% ----------------------------------
% robot = RoarmM2_MotionControl_v2('COM3');
% robot.moveHome();
% robot.disconnect();
%
% EXAMPLE 2: MOVE ALL JOINTS
% ----------------------------------
% robot = RoarmM2_MotionControl_v2('COM3');
% robot.moveToJointAngles([0, 30, 90, 180]);
% pause(1);
% robot.moveHome();
% robot.disconnect();
%
% EXAMPLE 3: CARTESIAN CONTROL
% ----------------------------------
% robot = RoarmM2_MotionControl_v2('COM3');
% robot.moveToCartesian([250, 0, 300]);  % Forward
% pause(1);
% robot.moveToCartesian([250, 100, 250]); % Forward-left-lower
% pause(1);
% robot.moveHome();
% robot.disconnect();
%
% EXAMPLE 4: GRIPPER CONTROL
% ----------------------------------
% robot = RoarmM2_MotionControl_v2('COM3');
% robot.moveToCartesian([250, 0, 300]);
% robot.controlEndEffector(180);  % Close gripper
% pause(1);
% robot.controlEndEffector(45);   % Open gripper
% robot.moveHome();
% robot.disconnect();
%
% EXAMPLE 5: PICK AND PLACE
% ----------------------------------
% robot = RoarmM2_MotionControl_v2('COM3');
% 
% % Pick location
% robot.moveToCartesian([250, 100, 350]);  % Above
% robot.moveToCartesian([250, 100, 200]);  % At object
% robot.controlEndEffector(180);           % Close
% robot.moveToCartesian([250, 100, 350]);  % Lift
% 
% % Place location
% robot.moveToCartesian([-150, 0, 300]);   % Move
% robot.moveToCartesian([-150, 0, 200]);   % Lower
% robot.controlEndEffector(45);            % Open
% 
% robot.moveHome();
% robot.disconnect();
%
% ===========================================================
% MAIN METHODS
% ===========================================================
%
% INITIALIZATION & CONNECTION
% ---------------------------
% robot = RoarmM2_MotionControl_v2();           % Simulation mode
% robot = RoarmM2_MotionControl_v2('COM3');     % Connected mode
% robot.connect('COM3');                        % Manual connection
% robot.disconnect();                           % Disconnect
%
% BASIC MOTION
% ---------------------------
% robot.moveHome();                             % Move to home
% robot.moveToJointAngles([0, 30, 90, 180]);   % All joints
% robot.moveSingleJoint(1, 45);                % Single joint
% robot.moveSingleJoint(1, 45, 50, 10);        % With speed/accel
%
% CARTESIAN MOTION
% ---------------------------
% robot.moveToCartesian([250, 0, 300]);        % XYZ position
% robot.moveToCartesian([250, 0, 300], 135);   % XYZ + angle
%
% END EFFECTOR
% ---------------------------
% robot.controlEndEffector(180);                % Close gripper
% robot.controlEndEffector(45, 50, 10);        % With speed/accel
%
% CONTINUOUS MOVEMENT
% ---------------------------
% robot.moveContiguous(0, 1, 1, 10);           % Continuous
% robot.moveContiguous(0, 1, 0, 0);            % Stop
%
% FEEDBACK & STATUS
% ---------------------------
% angles = robot.getCurrentJointAngles();      % Get angles
% feedback = robot.getFeedback();              % Get position
% status = robot.getStatus();                  % Get status
%
% TORQUE CONTROL
% ---------------------------
% robot.setTorque(1);                          % Lock joints
% robot.setTorque(0);                          % Unlock joints
%
% ===========================================================
% TROUBLESHOOTING
% ===========================================================
%
% PROBLEM: Cannot connect to robot
% SOLUTION:
%   1. Check COM port in Device Manager
%   2. Verify USB cable is connected
%   3. Check robot is powered on
%   4. Verify baud rate is 115200
%
% PROBLEM: Robot not responding to commands
% SOLUTION:
%   1. Check connection status: robot.IsConnected
%   2. Try in simulation mode first
%   3. Check JSON command format
%   4. Verify robot power supply
%
% PROBLEM: Commands cause errors
% SOLUTION:
%   1. Check joint angles are within limits
%   2. Verify position is within workspace
%   3. Use RoarmM2_QuickReference.m for examples
%   4. Run RoarmM2_CompleteExample.m to test
%
% ===========================================================
% COORDINATE SYSTEM
% ===========================================================
%
% Right-hand rule convention:
%   X-axis: Positive = forward (away from base)
%   Y-axis: Positive = left (when facing forward)
%   Z-axis: Positive = up (vertically)
%
% Origin: Base of the robot
% Units: Millimeters (mm)
%
% ===========================================================
% SERIAL PROTOCOL
% ===========================================================
%
% Communication: JSON over UART/USB serial
% Baud Rate: 115200
% Format: {"T":command_id, "param1":value1, "param2":value2}
%
% Common Commands:
%   T=100: Home position
%   T=121: Single joint angle control
%   T=122: All joints angle control
%   T=106: End effector control
%   T=1041: Cartesian position control
%   T=210: Torque control
%
% Example Commands:
%   {"T":100}  % Move to home
%   {"T":121,"joint":1,"angle":45,"spd":50,"acc":10}  % Joint 1 to 45°
%   {"T":122,"b":0,"s":30,"e":90,"h":180,"spd":50,"acc":10}  % All joints
%   {"T":1041,"x":250,"y":0,"z":300,"t":0}  % Move to XYZ
%
% ===========================================================
% REFERENCES
% ===========================================================
%
% Official GitHub: https://github.com/waveshareteam/roarm_m2
% Official Wiki: https://www.waveshare.com/wiki/RoArm-M2-S
% JSON Commands: https://www.waveshare.com/wiki/RoArm-M2-S_JSON_Command_Meaning
% Control Guide: https://www.waveshare.com/wiki/RoArm-M2-S_Robotic_Arm_Control
%
% ===========================================================
% VERSION INFO
% ===========================================================
%
% RoarmM2_MotionControl_v2: v2.0 (Current)
% - 4-DOF support
% - JSON command formatting
% - Serial communication
% - Cartesian control
% - Gripper control
%
% ===========================================================
% NOTES
% ===========================================================
%
% 1. Simulation Mode
%    - Commands print to console instead of sending to hardware
%    - Useful for testing scripts without robot
%    - Remove serial port parameter to use
%
% 2. Speed and Acceleration
%    - Speed: 0-100 (0 or high = fast, lower = slower)
%    - Acceleration: 0-254 (smaller = smoother, 0 = max)
%    - Default: speed=50, accel=10
%
% 3. Joint Angles
%    - All angles in degrees
%    - Home position: [0, 0, 90, 180]
%    - Refer to joint limits for valid ranges
%
% 4. Cartesian Coordinates
%    - All coordinates in millimeters
%    - Uses right-hand rule
%    - Origin at robot base
%
% ===========================================================
% SUPPORT & ISSUES
% ===========================================================
%
% For issues or questions:
% 1. Check RoarmM2_CompleteExample.m for working examples
% 2. Verify robot specifications match your hardware
% 3. Test in simulation mode first
% 4. Refer to official Waveshare documentation
% 5. Check hardware connections and power supply
%
% ===========================================================
