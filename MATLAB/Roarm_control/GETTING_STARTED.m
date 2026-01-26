% ============================================================================
% RoArm M2 MATLAB MOTION CONTROL PACKAGE - INSTALLATION & USAGE GUIDE
% ============================================================================
%
% This MATLAB package provides complete motion control for the Waveshare RoArm M2
% 4-DOF robotic arm using JSON-based serial communication.
%
% ============================================================================
% WHAT YOU HAVE
% ============================================================================
%
% COMPLETE MOTION CONTROL SYSTEM including:
% ✓ 4-DOF joint control (Base, Shoulder, Elbow, End Effector)
% ✓ Cartesian XYZ position control with inverse kinematics
% ✓ Gripper/wrist end effector control
% ✓ Serial communication at 115200 baud
% ✓ JSON command formatting
% ✓ Speed and acceleration control
% ✓ Status feedback and monitoring
% ✓ Comprehensive examples and documentation
%
% ============================================================================
% FILES CREATED
% ============================================================================
%
% 1. RoarmM2_MotionControl_v2.m (14.45 KB)
%    └─ Main robot control class with all methods
%    └─ 100% compatible with Waveshare RoArm M2
%    └─ Fully object-oriented design
%    └─ Ready for production use
%
% 2. RoarmM2_CompleteExample.m (11.79 KB)
%    └─ RECOMMENDED - Start here!
%    └─ Step-by-step comprehensive demonstration
%    └─ 13 different examples showing all features
%    └─ Detailed console output
%    └─ Best for learning
%
% 3. RoarmM2_QuickReference.m (7.69 KB)
%    └─ Quick command reference guide
%    └─ Copy-paste ready code snippets
%    └─ 5 practical examples
%    └─ Best for quick lookups during development
%
% 4. README.m (9.61 KB)
%    └─ Complete documentation
%    └─ Setup instructions
%    └─ Troubleshooting guide
%    └─ API reference
%    └─ Coordinate system explanation
%
% 5. RoarmM2_MotionControl.m (14.90 KB)
%    └─ Original 6-DOF version (reference only)
%
% 6. RoarmM2_Demo.m (5.39 KB)
%    └─ Alternative demo file
%
% ============================================================================
% HOW TO START (3 EASY STEPS)
% ============================================================================
%
% STEP 1: RUN THE COMPLETE EXAMPLE
% ---------------------------------
% >> cd 'c:\Users\Collins\Downloads\sim-kuka-robot-master\sim-kuka-robot-master\roarm matlab program'
% >> RoarmM2_CompleteExample
%
% This will show you everything the robot can do with detailed output.
%
% STEP 2: TRY THE QUICK REFERENCE
% --------------------------------
% >> RoarmM2_QuickReference
%
% This shows quick examples you can copy and modify.
%
% STEP 3: CREATE YOUR OWN SCRIPT
% --------------------------------
% robot = RoarmM2_MotionControl_v2('COM3');  % Replace COM3 with your port
% robot.moveHome();
% robot.moveToJointAngles([0, 30, 90, 180]);
% robot.disconnect();
%
% ============================================================================
% KEY FEATURES & METHODS
% ============================================================================
%
% MOTION CONTROL:
%   moveHome()                          - Return to home position
%   moveToJointAngles([b,s,e,h])       - Move all joints
%   moveSingleJoint(joint, angle)       - Move one joint
%   moveToCartesian([x,y,z])            - Move to XYZ position
%   controlEndEffector(angle)           - Control gripper/wrist
%
% CONNECTION:
%   connect('COM3')                     - Connect to robot
%   disconnect()                        - Disconnect
%   isConnected                         - Check connection
%
% FEEDBACK:
%   getCurrentJointAngles()             - Get current angles
%   getStatus()                         - Get robot status
%   getFeedback()                       - Get position feedback
%
% VALIDATION:
%   checkJointLimits(angles)            - Verify angles are valid
%
% ============================================================================
% ROBOT SPECIFICATIONS
% ============================================================================
%
% MECHANICAL:
%   Type: 4-DOF Desktop Robotic Arm
%   Weight: < 850g (lightweight)
%   Payload: 0.5 kg @ 0.5m
%   Workspace: ~1 meter diameter
%   Joint Type: High-torque serial bus servo
%
% JOINT RANGES:
%   Base:          -180° to 180° (360° rotation)
%   Shoulder:       -90° to  90° (±90° movement)
%   Elbow:          -45° to 180° (225° range)
%   End Effector:    45° to 315° (Gripper mode)
%
% COMMUNICATION:
%   Interface: UART/USB (serial)
%   Baud Rate: 115200
%   Protocol: JSON commands
%   Connector: USB Type-C
%
% ELECTRONICS:
%   Main CPU: ESP32 (Arduino compatible)
%   Encoders: 12-bit magnetic encoders
%   Accuracy: ±0.088°
%   Control: Dual-drive shoulder joint
%
% ============================================================================
% QUICK EXAMPLES
% ============================================================================
%
% EXAMPLE 1: MOVE FORWARD
% >> robot = RoarmM2_MotionControl_v2('COM3');
% >> robot.moveToCartesian([250, 0, 300]);
% >> pause(1);
% >> robot.moveHome();
% >> robot.disconnect();
%
% EXAMPLE 2: SWEEP BASE FROM LEFT TO RIGHT
% >> robot = RoarmM2_MotionControl_v2('COM3');
% >> for angle = -90:10:90
% >>     robot.moveSingleJoint(1, angle);
% >>     pause(0.3);
% >> end
% >> robot.moveHome();
% >> robot.disconnect();
%
% EXAMPLE 3: OPEN AND CLOSE GRIPPER
% >> robot = RoarmM2_MotionControl_v2('COM3');
% >> robot.controlEndEffector(45);   % Open
% >> pause(1);
% >> robot.controlEndEffector(180);  % Close
% >> pause(1);
% >> robot.controlEndEffector(120);  % Partially open
% >> robot.disconnect();
%
% EXAMPLE 4: AUTOMATIC PICK & PLACE
% >> robot = RoarmM2_MotionControl_v2('COM3');
% >> robot.moveHome();
% >> robot.moveToCartesian([250, 100, 350]);  % Move above object
% >> robot.moveToCartesian([250, 100, 200]);  % Lower
% >> robot.controlEndEffector(180);            % Pick
% >> robot.moveToCartesian([-150, 0, 300]);   % Move to place location
% >> robot.moveToCartesian([-150, 0, 200]);   % Lower
% >> robot.controlEndEffector(45);             % Release
% >> robot.moveHome();
% >> robot.disconnect();
%
% ============================================================================
% COORDINATE SYSTEM
% ============================================================================
%
% The robot uses a RIGHT-HAND coordinate system:
%
%       Z-axis (UP)
%         |
%         |  Y-axis (LEFT)
%         | /
%         |/
%         +------- X-axis (FORWARD)
%        /
%
% When using moveToCartesian([x, y, z]):
%   X = 250:  250mm forward (away from robot)
%   Y = 100:  100mm to the left
%   Z = 300:  300mm up
%
% Origin is at the robot's base (joint 1)
% All measurements in millimeters
%
% ============================================================================
% HARDWARE SETUP
% ============================================================================
%
% POWER:
%   Voltage: 7-12.6V DC
%   Recommended: 12V 5A power supply
%   Alternative: 3S lithium battery
%
% CONNECTIONS:
%   Serial: Connect via USB Type-C to your computer
%   Power: Connect 12V DC power supply
%   Ground: Ensure common ground if using external power
%
% FIND COM PORT:
%   Windows: Device Manager → COM & LPT Ports
%   Linux: ls /dev/ttyUSB*
%   Mac: ls /dev/tty.usbserial*
%
% ============================================================================
% GETTING HELP
% ============================================================================
%
% 1. READ THE DOCUMENTATION:
%    >> README.m
%    Contains complete API reference and troubleshooting
%
% 2. VIEW QUICK REFERENCE:
%    >> RoarmM2_QuickReference.m
%    Copy-paste examples for your use case
%
% 3. RUN COMPLETE EXAMPLE:
%    >> RoarmM2_CompleteExample.m
%    See all features with detailed output
%
% 4. CHECK OFFICIAL RESOURCES:
%    GitHub: https://github.com/waveshareteam/roarm_m2
%    Wiki: https://www.waveshare.com/wiki/RoArm-M2-S
%
% ============================================================================
% TROUBLESHOOTING
% ============================================================================
%
% ISSUE: "Failed to connect to RoArm M2"
% SOLUTION:
%   1. Check COM port is correct (Device Manager)
%   2. Ensure USB cable is connected
%   3. Verify robot is powered on
%   4. Try a different USB cable or port
%
% ISSUE: "Target angles exceed joint limits"
% SOLUTION:
%   1. Check joint angle ranges (see above)
%   2. Use validateJointLimits before moving
%   3. Refer to RoarmM2_QuickReference.m for valid ranges
%
% ISSUE: Robot moves slowly or doesn't respond
% SOLUTION:
%   1. Check robot power supply (12V 5A)
%   2. Verify serial connection: robot.IsConnected
%   3. Try in simulation mode first to test code
%   4. Restart robot and MATLAB
%
% ISSUE: Gripper won't open/close
% SOLUTION:
%   1. Check end effector type (gripper vs wrist)
%   2. Verify angle range (45-315 degrees)
%   3. Check gripper power and mechanical connection
%
% ============================================================================
% SIMULATION MODE
% ============================================================================
%
% To test your code WITHOUT hardware:
%
% >> robot = RoarmM2_MotionControl_v2();  % No COM port parameter
% >> robot.moveHome();                     % Commands print to console
% >> robot.moveToJointAngles([0, 30, 90, 180]);
%
% Simulation mode:
% ✓ No hardware required
% ✓ Commands display instead of executing
% ✓ Perfect for testing code
% ✓ No errors from hardware
% ✓ Great for learning the API
%
% ============================================================================
% NEXT STEPS
% ============================================================================
%
% 1. Open RoarmM2_CompleteExample.m and run it
%    >> RoarmM2_CompleteExample
%
% 2. Find your COM port (Device Manager)
%
% 3. Modify the initialization:
%    robot = RoarmM2_MotionControl_v2('COM3');  % Your port
%
% 4. Create your own motion sequences
%
% 5. Refer to RoarmM2_QuickReference.m for command examples
%
% ============================================================================
% SUPPORT
% ============================================================================
%
% Official Resources:
% • GitHub Repository: https://github.com/waveshareteam/roarm_m2
% • Waveshare Wiki: https://www.waveshare.com/wiki/RoArm-M2-S
% • JSON Commands: https://www.waveshare.com/wiki/RoArm-M2-S_JSON_Command_Meaning
% • Control Guide: https://www.waveshare.com/wiki/RoArm-M2-S_Robotic_Arm_Control
%
% Quick Start:
% • RoarmM2_CompleteExample.m (shows everything)
% • RoarmM2_QuickReference.m (quick lookup)
% • README.m (full documentation)
%
% ============================================================================
% THANK YOU FOR USING RoArm M2 MATLAB CONTROL SYSTEM
% ============================================================================
%
% Created: December 2024
% Version: 2.0
% Robot: Waveshare RoArm M2 (4-DOF)
% Protocol: JSON over UART/USB
%
% Happy Robotics!
%
% ============================================================================
