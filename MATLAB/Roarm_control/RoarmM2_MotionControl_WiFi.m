%% RO-ARM M2 ROBOT MOTION CONTROL SYSTEM (WiFi/HTTP Version)
% This MATLAB code provides comprehensive motion control for the Waveshare RoArm M2 robot
% using WiFi/HTTP communication via IP address
% 
% Robot Specifications:
% - 4 DOF (Base, Shoulder, Elbow, End Effector)
% - Communication: WiFi/HTTP at IP address (not serial)
% - Control Protocol: JSON commands over HTTP
% - Default IP: 192.168.4.1 (AP mode)
% - Coordinates: Right-hand rule (X forward, Y left, Z up)

classdef RoarmM2_MotionControl_WiFi
    properties
        % Robot parameters
        DOF = 4;  % Degrees of freedom
        
        % Joint angle limits in degrees
        JointLimits_Lower = [-180, -90, -45, 45];
        JointLimits_Upper = [180, 90, 180, 315];
        JointNames = {'Base', 'Shoulder', 'Elbow', 'EndEffector'};
        
        % Home position in degrees
        HomeAngles = [0, 0, 90, 180];
        
        % WiFi connection properties
        IPAddress;      % Robot IP address
        Port = 80;      % HTTP port
        Timeout = 5;    % Connection timeout in seconds
        
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
        function obj = RoarmM2_MotionControl_WiFi(ipAddress)
            % Initialize RoArm M2 with WiFi/HTTP connection via IP address
            % Usage: robot = RoarmM2_MotionControl_WiFi('192.168.4.1');
            %    or: robot = RoarmM2_MotionControl_WiFi('192.168.1.100');
            
            obj.CurrentJointAngles = obj.HomeAngles;
            obj.TargetJointAngles = obj.HomeAngles;
            obj.IsConnected = false;
            
            if nargin > 0 && ~isempty(ipAddress)
                obj.connect(ipAddress);
            end
        end
        
        %% Connect to robot via IP address
        function connect(obj, ipAddress)
            % Connect to RoArm M2 via HTTP/WiFi
            % Supports both AP mode (192.168.4.1) and STA mode
            obj.IPAddress = ipAddress;
            
            try
                % Test connection by sending a simple command
                testURL = sprintf('http://%s/js?json={"T":100}', obj.IPAddress);
                response = webread(testURL, 'Timeout', obj.Timeout);
                obj.IsConnected = true;
                disp(sprintf('Successfully connected to RoArm M2 at %s', obj.IPAddress));
            catch ME
                disp(sprintf('Failed to connect to %s: %s', obj.IPAddress, ME.message));
                obj.IsConnected = false;
                obj.IPAddress = ipAddress;  % Store IP anyway for later attempts
            end
        end
        
        %% Disconnect from robot
        function disconnect(obj)
            % Disconnect from robot
            obj.IsConnected = false;
            if ~isempty(obj.IPAddress)
                disp(sprintf('Disconnected from RoArm M2 at %s', obj.IPAddress));
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
            %   EndEffector: 45 to 315 degrees (180 = gripper closed)
            
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
            feedback = obj.sendCommand(cmdStr);
            
            if ~isempty(feedback)
                disp('Feedback received:');
                disp(feedback);
            else
                disp('No feedback received');
            end
        end
        
        %% Control end effector (gripper/wrist)
        function controlEndEffector(obj, angle, speed, accel)
            % Control gripper or wrist end effector
            %
            % Gripper mode:
            %   Angle range: 45 to 180 degrees
            %   180 = closed, lower values = more open
            
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
        
        %% Send JSON command to robot via HTTP/WiFi
        function response = sendCommand(obj, cmdStr)
            % Send JSON command to robot via HTTP GET request
            % Input: cmdStr - JSON command string
            % Output: response - HTTP response from robot
            
            response = '';
            
            if isempty(obj.IPAddress)
                disp('[SIM] No IP address configured. Operating in simulation mode.');
                disp(['[SIM] Command: ' cmdStr]);
                return;
            end
            
            if ~obj.IsConnected
                disp('[OFFLINE] Robot is not connected. Command not sent.');
                disp(['[OFFLINE] Command: ' cmdStr]);
                return;
            end
            
            try
                % Build HTTP GET request URL
                % Format: http://192.168.4.1/js?json={"T":122,...}
                encodedCmd = strrep(cmdStr, ' ', '');  % Remove spaces
                url = sprintf('http://%s/js?json=%s', obj.IPAddress, encodedCmd);
                
                % Send command via HTTP GET
                options = weboptions('Timeout', obj.Timeout);
                response = webread(url, options);
                
                % Small delay for command processing
                pause(0.05);
                
            catch ME
                disp(sprintf('Error sending command: %s', ME.message));
                % Try to reconnect
                obj.IsConnected = false;
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
            status.IPAddress = obj.IPAddress;
            status.CurrentAngles = obj.CurrentJointAngles;
            status.TargetAngles = obj.TargetJointAngles;
            status.JointNames = obj.JointNames;
        end
        
        %% Cleanup
        function delete(obj)
            % Destructor - disconnect from robot
            if obj.IsConnected
                obj.disconnect();
            end
        end
    end
end

%% Example Usage
% % Initialize robot via WiFi (Default AP mode IP)
% robot = RoarmM2_MotionControl_WiFi('192.168.4.1');
%
% % Or with custom IP (if using STA mode)
% robot = RoarmM2_MotionControl_WiFi('192.168.1.100');
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
