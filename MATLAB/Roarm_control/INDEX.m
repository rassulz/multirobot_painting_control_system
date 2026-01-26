%% ROARM M2 MATLAB MOTION CONTROL - FILE INDEX
% ============================================================================
% Complete MATLAB control system for Waveshare RoArm M2 robotic arm
% GitHub: https://github.com/waveshareteam/roarm_m2
% ============================================================================
%
% This directory contains 8 MATLAB files totaling 82.4 KB of code,
% examples, and documentation for controlling the RoArm M2 robot.
%
% ============================================================================
% START HERE
% ============================================================================
%
% 1. GETTING_STARTED.m (11.32 KB)
%    └─ Installation & setup guide
%    └─ Hardware requirements
%    └─ Quick start examples
%    └─ Troubleshooting
%    └─ READ THIS FIRST!
%
%    >> open GETTING_STARTED.m
%
% ============================================================================
% MAIN CONTROL SYSTEM
% ============================================================================
%
% 2. RoarmM2_MotionControl_v2.m (14.45 KB)
%    └─ PRIMARY ROBOT CONTROL CLASS
%    └─ All motion methods
%    └─ Serial communication
%    └─ JSON command formatting
%    └─ Status feedback
%    └─ RECOMMENDED FOR PRODUCTION USE
%
%    >> robot = RoarmM2_MotionControl_v2('COM3');
%    >> robot.moveHome();
%    >> robot.disconnect();
%
% ============================================================================
% EXAMPLES & TUTORIALS
% ============================================================================
%
% 3. RoarmM2_CompleteExample.m (11.79 KB)
%    └─ COMPREHENSIVE STEP-BY-STEP TUTORIAL
%    └─ 13 different demonstration examples
%    └─ Shows all features in action
%    └─ Detailed console output
%    └─ BEST FOR LEARNING
%
%    >> RoarmM2_CompleteExample
%
% 4. RoarmM2_QuickReference.m (7.69 KB)
%    └─ Quick reference guide
%    └─ Copy-paste code snippets
%    └─ Common commands
%    └─ 5 practical examples
%    └─ BEST FOR QUICK LOOKUPS
%
%    >> open RoarmM2_QuickReference.m
%
% ============================================================================
% DOCUMENTATION
% ============================================================================
%
% 5. README.m (9.61 KB)
%    └─ Complete API documentation
%    └─ All methods explained
%    └─ Parameter descriptions
%    └─ Robot specifications
%    └─ Troubleshooting guide
%    └─ DETAILED REFERENCE
%
%    >> open README.m
%
% ============================================================================
% REFERENCE FILES
% ============================================================================
%
% 6. RoarmM2_MotionControl.m (14.90 KB)
%    └─ Original 6-DOF version
%    └─ For reference only
%    └─ Use v2 for RoArm M2
%
% 7. RoarmM2_Demo.m (5.39 KB)
%    └─ Alternative demo
%    └─ Simpler examples
%
% 8. QuickReference.m (7.25 KB)
%    └─ Original quick reference
%
% ============================================================================
% RECOMMENDED USAGE FLOW
% ============================================================================
%
% STEP 1: READ GETTING STARTED
% >> open GETTING_STARTED.m
% Takes 5 minutes, covers everything you need to know
%
% STEP 2: RUN COMPLETE EXAMPLE
% >> RoarmM2_CompleteExample
% Takes 2-3 minutes, shows all features in action
%
% STEP 3: TRY QUICK REFERENCE
% >> open RoarmM2_QuickReference.m
% Quick command lookups during development
%
% STEP 4: CREATE YOUR SCRIPT
% >> robot = RoarmM2_MotionControl_v2('COM3');
% >> % Your code here
% >> robot.disconnect();
%
% STEP 5: REFER TO README
% >> open README.m
% Complete API reference when needed
%
% ============================================================================
% KEY FILES AT A GLANCE
% ============================================================================
%
% FILE                              PURPOSE                    SIZE
% ────────────────────────────────────────────────────────────────────────
% RoarmM2_MotionControl_v2.m        Main control class        14.45 KB
% RoarmM2_CompleteExample.m         Full tutorial             11.79 KB
% GETTING_STARTED.m                 Setup guide               11.32 KB
% README.m                           Full documentation       9.61 KB
% RoarmM2_QuickReference.m          Quick reference           7.69 KB
% RoarmM2_Demo.m                    Alternative demo          5.39 KB
% QuickReference.m                  Original reference        7.25 KB
% RoarmM2_MotionControl.m           6-DOF reference           14.90 KB
% ────────────────────────────────────────────────────────────────────────
% TOTAL: 8 files, 82.4 KB
%
% ============================================================================
% ROBOT SPECIFICATIONS (QUICK REFERENCE)
% ============================================================================
%
% Type: 4-DOF Robotic Arm
% Weight: < 850g
% Payload: 0.5 kg @ 0.5m
% Communication: UART/USB @ 115200 baud
% Protocol: JSON commands
%
% JOINTS:
%   1. Base:          -180° to +180°
%   2. Shoulder:       -90° to  +90°
%   3. Elbow:          -45° to +180°
%   4. EndEffector:    +45° to +315°
%
% WORKSPACE:
%   Radius: ~500 mm
%   Height: 0-500 mm
%
% ============================================================================
% MAIN METHODS
% ============================================================================
%
% robot.moveHome()
%   └─ Return to home position [0, 0, 90, 180]
%
% robot.moveToJointAngles([b, s, e, h])
%   └─ Move all joints to specified angles
%   └─ b=base, s=shoulder, e=elbow, h=hand
%
% robot.moveSingleJoint(joint, angle, speed, accel)
%   └─ Move one joint
%   └─ joint: 1-4
%
% robot.moveToCartesian([x, y, z])
%   └─ Move to XYZ position
%   └─ Uses inverse kinematics
%
% robot.controlEndEffector(angle)
%   └─ Control gripper/wrist
%   └─ angle: 45-315 degrees
%
% robot.getFeedback()
%   └─ Get current position and torques
%
% robot.getStatus()
%   └─ Get robot status
%
% robot.connect('COM3')
%   └─ Connect to robot
%
% robot.disconnect()
%   └─ Disconnect from robot
%
% ============================================================================
% QUICK START CODE
% ============================================================================
%
% % Create robot object
% robot = RoarmM2_MotionControl_v2('COM3');  % Replace COM3 with your port
%
% % Move to home
% robot.moveHome();
% pause(1);
%
% % Move all joints
% robot.moveToJointAngles([0, 30, 90, 180]);
% pause(1);
%
% % Move to cartesian position
% robot.moveToCartesian([250, 0, 300]);
% pause(1);
%
% % Control gripper
% robot.controlEndEffector(180);  % Close
% pause(0.5);
% robot.controlEndEffector(45);   % Open
%
% % Clean up
% robot.moveHome();
% robot.disconnect();
%
% ============================================================================
% COMMON ERRORS & SOLUTIONS
% ============================================================================
%
% ERROR: "Failed to connect to RoArm M2"
% → Check COM port (Device Manager)
% → Ensure USB cable is connected
% → Verify robot is powered on
%
% ERROR: "Target angles exceed joint limits"
% → Check angle ranges (see above)
% → Verify angles are in degrees
% → Use checkJointLimits() to validate
%
% ERROR: "Robot not responding"
% → Check power supply (12V 5A)
% → Verify serial connection
% → Try robot.IsConnected
% → Restart robot and MATLAB
%
% ============================================================================
% SIMULATION MODE (NO HARDWARE NEEDED)
% ============================================================================
%
% To test code without hardware:
%
% % Create robot in simulation mode
% robot = RoarmM2_MotionControl_v2();  % No COM port
%
% % Commands print to console instead of executing
% robot.moveHome();
% robot.moveToJointAngles([0, 30, 90, 180]);
%
% Perfect for testing your code before using hardware!
%
% ============================================================================
% OFFICIAL RESOURCES
% ============================================================================
%
% GitHub Repository:
% https://github.com/waveshareteam/roarm_m2
%
% Waveshare Wiki:
% https://www.waveshare.com/wiki/RoArm-M2-S
%
% JSON Commands:
% https://www.waveshare.com/wiki/RoArm-M2-S_JSON_Command_Meaning
%
% Control Guide:
% https://www.waveshare.com/wiki/RoArm-M2-S_Robotic_Arm_Control
%
% ============================================================================
% NEXT STEPS
% ============================================================================
%
% 1. Read GETTING_STARTED.m (5 minutes)
%    >> open GETTING_STARTED.m
%
% 2. Run RoarmM2_CompleteExample (2 minutes)
%    >> RoarmM2_CompleteExample
%
% 3. Open RoarmM2_QuickReference.m for quick lookups
%    >> open RoarmM2_QuickReference.m
%
% 4. Refer to README.m for complete API documentation
%    >> open README.m
%
% 5. Create your own motion control scripts!
%
% ============================================================================
% PACKAGE INFORMATION
% ============================================================================
%
% Created: December 2024
% Version: 2.0
% Robot: Waveshare RoArm M2 (4-DOF)
% Protocol: JSON over UART/USB
% Baud Rate: 115200
%
% License: Compatible with RoArm M2 (AGPL-3.0)
%
% ============================================================================
% THANK YOU FOR USING ROARM M2 MATLAB CONTROL!
% ============================================================================
%
% For support and questions, refer to:
% • GETTING_STARTED.m
% • README.m
% • Official Waveshare documentation
%
% Happy robotics!
%
% ============================================================================
