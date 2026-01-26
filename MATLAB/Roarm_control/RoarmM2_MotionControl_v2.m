%% RO-ARM M2 ROBOT MOTION CONTROL SYSTEM
% This MATLAB code provides comprehensive motion control for the Waveshare RoArm M2 robot
% RoArm M2 is a 4-DOF robotic arm with JSON-based serial communication
% Reference: https://github.com/waveshareteam/roarm_m2
%
% Robot Specifications:
% - 4 DOF: Base (360° rotation), Shoulder, Elbow, End Effector (Gripper/Wrist)
% - Serial Communication: UART/USB at 115200 baud
% - Control Protocol: JSON commands
% - Workspace: ~1 meter diameter
% - Coordinate System: Right-hand rule (X forward, Y left, Z up)
% - Payload: 0.5 kg @ 0.5m

classdef RoarmM2_MotionControl_v2
    properties
        % Robot parameters
        DOF = 4;  % Degrees of freedom
        
        % Joint angle limits in degrees
        % 1: Base (360° rotation)
        % 2: Shoulder 
        % 3: Elbow
        % 4: End Effector (Gripper or Wrist)
        JointLimits_Lower = [-180, -90, -45, 45];
        JointLimits_Upper = [180, 90, 180, 315];
        JointNames = {'Base', 'Shoulder', 'Elbow', 'EndEffector'};
        
        % Home position in degrees
        HomeAngles = [0, 0, 90, 180];
        
        % Serial communication
        SerialPort;
        BaudRate = 115200;
        Timeout = 5;
        
        % Current state
        CurrentJointAngles;
        TargetJointAngles;
        IsConnected = false;
        
        % Workspace bounds
        WorkspaceRadius = 500;  % mm
        MaxXYZ = [500, 500, 500];  % mm
    end
    
    methods
        %% Constructor
        function obj = RoarmM2_MotionControl_v2(portName)
            % Initialize RoArm M2 with optional serial port connection
            % Usage: robot = RoarmM2_MotionControl_v2('COM3');
            %    or: robot = RoarmM2_MotionControl_v2();  % For simulation
            
            obj.CurrentJointAngles = obj.HomeAngles;
            obj.TargetJointAngles = obj.HomeAngles;
            obj.IsConnected = false;
            
            if nargin > 0 && ~isempty(portName)
                obj.connect(portName);
            end
        end
        
        %% Connect to robot
        function connect(obj, portName)
            % Connect to RoArm M2 via serial port
            try
                obj.SerialPort = serial(portName);
                obj.SerialPort.BaudRate = obj.BaudRate;
                obj.SerialPort.Timeout = obj.Timeout;
                fopen(obj.SerialPort);
                obj.IsConnected = true;
                disp(['Successfully connected to RoArm M2 on ' portName]);
                pause(1);  % Allow time for initialization
            catch ME
                disp(['Failed to connect: ' ME.message]);
                obj.IsConnected = false;
            end
        end
        
        %% Disconnect from robot
        function disconnect(obj)
            % Close serial connection
            if ~isempty(obj.SerialPort) && isvalid(obj.SerialPort)
                fclose(obj.SerialPort);
                delete(obj.SerialPort);
                obj.SerialPort = [];
                obj.IsConnected = false;
                disp('Disconnected from RoArm M2');
            end
        end
        
        %% Move to home position
        function moveHome(obj)
            % Move all joints to home position
            % Command T=100: CMD_MOVE_INIT
            cmdStr = '{"T":100}';
            obj.sendCommand(cmdStr);
            obj.CurrentJointAngles = obj.HomeAngles;
            disp('Robot moved to home position');
        end
        
        %% Move all joints to specified angles (in degrees)
        function moveToJointAngles(obj, targetAngles)
            % Move to target joint angles using CMD_JOINTS_ANGLE_CTRL (T=122)
            % 
            % Usage: moveToJointAngles(robot, [0, 30, 90, 180])
            %
            % Inputs:
            %   targetAngles - [base, shoulder, elbow, endeffector] in degrees
            %
            % Joint Angle Ranges:
            %   Base: -180 to 180 degrees (0 = center)
            %   Shoulder: -90 to 90 degrees (0 = center)
            %   Elbow: -45 to 180 degrees (90 = home)
            %   EndEffector: 45 to 315 degrees (180 = gripper closed, gripper mode)
            
            if length(targetAngles) ~= 4
                error('targetAngles must be a 4-element vector [base, shoulder, elbow, endeffector]');
            end
            
            % Validate limits
            for i = 1:4
                if targetAngles(i) < obj.JointLimits_Lower(i) || ...
                   targetAngles(i) > obj.JointLimits_Upper(i)
                    error(sprintf('%s angle %.1f exceeds limits [%.1f, %.1f]', ...
                        obj.JointNames{i}, targetAngles(i), ...
                        obj.JointLimits_Lower(i), obj.JointLimits_Upper(i)));
                end
            end
            
            % JSON command: T=122 CMD_JOINTS_ANGLE_CTRL
            % Parameters: b=base, s=shoulder, e=elbow, h=hand, spd=speed, acc=acceleration
            cmdStr = sprintf('{"T":122,"b":%.2f,"s":%.2f,"e":%.2f,"h":%.2f,"spd":50,"acc":10}', ...
                targetAngles(1), targetAngles(2), targetAngles(3), targetAngles(4));
            
            obj.sendCommand(cmdStr);
            obj.CurrentJointAngles = targetAngles;
            obj.TargetJointAngles = targetAngles;
        end
        
        %% Move single joint
        function moveSingleJoint(obj, joint, targetAngle, speed, accel)
            % Move a single joint to specified angle (degrees)
            %
            % Usage: moveSingleJoint(robot, 1, 45)
            %        moveSingleJoint(robot, 2, 30, 20, 5)
            %
            % Inputs:
            %   joint - joint number (1=Base, 2=Shoulder, 3=Elbow, 4=EndEffector)
            %   targetAngle - target angle in degrees
            %   speed - movement speed 0-100 (optional, default 50)
            %   accel - acceleration 0-254 (optional, default 10)
            
            if nargin < 4, speed = 50; end
            if nargin < 5, accel = 10; end
            
            if joint < 1 || joint > 4
                error('Joint number must be 1-4');
            end
            
            if targetAngle < obj.JointLimits_Lower(joint) || ...
               targetAngle > obj.JointLimits_Upper(joint)
                error(sprintf('%s angle %.1f exceeds limits [%.1f, %.1f]', ...
                    obj.JointNames{joint}, targetAngle, ...
                    obj.JointLimits_Lower(joint), obj.JointLimits_Upper(joint)));
            end
            
            % JSON command: T=121 CMD_SINGLE_JOINT_ANGLE
            % Parameters: joint (1-4), angle (degrees), spd (speed), acc (acceleration)
            cmdStr = sprintf('{"T":121,"joint":%d,"angle":%.2f,"spd":%d,"acc":%d}', ...
                joint, targetAngle, speed, accel);
            
            obj.sendCommand(cmdStr);
            obj.CurrentJointAngles(joint) = targetAngle;
        end
        
        %% Move to Cartesian position (XYZ coordinates)
        function moveToCartesian(obj, position, handAngle)
            % Move end-effector to XYZ position using inverse kinematics
            % This uses non-blocking direct control command
            %
            % Usage: moveToCartesian(robot, [250, 0, 300])
            %        moveToCartesian(robot, [250, 0, 300], 135)
            %
            % Inputs:
            %   position - [x, y, z] in millimeters
            %             X: forward/backward (-500 to 500)
            %             Y: left/right (-500 to 500)
            %             Z: up/down (0 to 500)
            %   handAngle - end effector rotation in degrees (optional)
            
            if nargin < 3
                handAngle = obj.CurrentJointAngles(4);  % Keep current end effector angle
            end
            
            if length(position) ~= 3
                error('Position must be [x, y, z] in millimeters');
            end
            
            % Convert hand angle to radians
            handRad = deg2rad(handAngle);
            
            % JSON command: T=1041 CMD_XYZT_DIRECT_CTRL (non-blocking)
            % Parameters: x, y, z (mm), t (end effector angle in radians)
            cmdStr = sprintf('{"T":1041,"x":%.2f,"y":%.2f,"z":%.2f,"t":%.4f}', ...
                position(1), position(2), position(3), handRad);
            
            obj.sendCommand(cmdStr);
            
            % Request feedback to update current angles
            obj.getFeedback();
        end
        
        %% Get robot feedback (current state)
        function feedback = getFeedback(obj)
            % Get current position, angles, and torques from robot
            % Command T=105: CMD_SERVO_RAD_FEEDBACK
            % Returns: x, y, z (mm), b, s, e, t (radians), torque values
            
            cmdStr = '{"T":105}';
            obj.sendCommand(cmdStr);
            
            % Read feedback (this needs to be parsed from the returned JSON)
            % The robot will respond with coordinates and angles
            feedback = [];
            if obj.IsConnected && isvalid(obj.SerialPort)
                try
                    pause(0.1);  % Wait for response
                    response = fscanf(obj.SerialPort, '%s');
                    feedback = response;
                    disp('Feedback received: ');
                    disp(response);
                catch
                    disp('No feedback received');
                end
            end
        end
        
        %% Move with continuous control
        function moveContiguous(obj, mode, axis, direction, speed)
            % Continuous movement control
            %
            % Usage: moveContiguous(robot, 0, 1, 1, 10)  % Move base left
            %        moveContiguous(robot, 1, 1, 1, 10)  % Move X axis
            %        moveContiguous(robot, 0, 1, 0, 0)   % Stop movement
            %
            % Inputs:
            %   mode - 0: angle control, 1: coordinate control
            %   axis - which axis to control (1-4)
            %   direction - 0: stop, 1: increase, 2: decrease
            %   speed - speed coefficient 0-20 (larger = faster)
            
            % JSON command: T=123 CMD_CONSTANT_CTRL
            cmdStr = sprintf('{"T":123,"m":%d,"axis":%d,"cmd":%d,"spd":%d}', ...
                mode, axis, direction, speed);
            
            obj.sendCommand(cmdStr);
        end
        
        %% Control end effector (gripper/wrist)
        function controlEndEffector(obj, angle, speed, accel)
            % Control gripper or wrist end effector
            %
            % Gripper mode:
            %   Angle range: 45 to 180 degrees
            %   180 = closed, lower values = more open
            %
            % Wrist mode:
            %   Angle range: 45 to 315 degrees
            %   Rotation capability
            
            if nargin < 3, speed = 50; end
            if nargin < 4, accel = 10; end
            
            % Command: T=106 CMD_EOAT_HAND_CTRL
            angleRad = deg2rad(angle);
            cmdStr = sprintf('{"T":106,"cmd":%.4f,"spd":%d,"acc":%d}', ...
                angleRad, speed, accel);
            
            obj.sendCommand(cmdStr);
            obj.CurrentJointAngles(4) = angle;
        end
        
        %% Torque control (lock/unlock joints)
        function setTorque(obj, state)
            % Control torque lock (enable/disable manual movement)
            %
            % Usage: setTorque(robot, 1)  % Lock torque (prevent manual movement)
            %        setTorque(robot, 0)  % Unlock torque (allow manual movement)
            
            % Command T=210: CMD_TORQUE_CTRL
            % cmd: 0=off (unlock), 1=on (lock)
            cmdStr = sprintf('{"T":210,"cmd":%d}', state);
            
            obj.sendCommand(cmdStr);
        end
        
        %% Send JSON command to robot
        function sendCommand(obj, cmdStr)
            % Send JSON command to robot
            % Input: cmdStr - JSON command string (without newline)
            
            if obj.IsConnected && isvalid(obj.SerialPort)
                % Add newline if not present
                if ~endsWith(cmdStr, newline)
                    cmdStr = [cmdStr newline];
                end
                
                try
                    fwrite(obj.SerialPort, cmdStr);
                    % Small delay for command processing
                    pause(0.05);
                catch ME
                    disp(['Error sending command: ' ME.message]);
                end
            else
                % Simulation mode - just display command
                disp(['[SIM] Command: ' cmdStr]);
            end
        end
        
        %% Check joint limits
        function isValid = checkJointLimits(obj, angles)
            % Verify all angles are within joint limits
            isValid = all(angles >= obj.JointLimits_Lower) && ...
                      all(angles <= obj.JointLimits_Upper);
        end
        
        %% Get current joint angles
        function angles = getCurrentJointAngles(obj)
            angles = obj.CurrentJointAngles;
        end
        
        %% Get status
        function status = getStatus(obj)
            % Get robot status
            status.IsConnected = obj.IsConnected;
            status.CurrentAngles = obj.CurrentJointAngles;
            status.TargetAngles = obj.TargetJointAngles;
            status.JointNames = obj.JointNames;
        end
        
        %% Cleanup
        function delete(obj)
            % Destructor - close serial connection
            if ~isempty(obj.SerialPort) && isvalid(obj.SerialPort)
                try
                    fclose(obj.SerialPort);
                    delete(obj.SerialPort);
                catch
                end
            end
        end
    end
end

%% Example Usage
% % Initialize robot (with serial connection)
% robot = RoarmM2_MotionControl_v2('COM3');
% 
% % Move to home position
% robot.moveHome();
% pause(2);
% 
% % Move all joints
% robot.moveToJointAngles([0, 30, 90, 180]);
% pause(2);
% 
% % Move individual joint
% robot.moveSingleJoint(1, 45);  % Base to 45 degrees
% pause(1);
% 
% % Move to Cartesian position (XYZ)
% robot.moveToCartesian([250, 0, 300], 135);
% pause(2);
% 
% % Get feedback
% robot.getFeedback();
% 
% % Return to home
% robot.moveHome();
% 
% % Cleanup
% robot.disconnect();
% clear robot;
