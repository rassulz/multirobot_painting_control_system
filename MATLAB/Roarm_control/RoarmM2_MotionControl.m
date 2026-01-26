%% RO-ARM M2 ROBOT MOTION CONTROL SYSTEM
% This MATLAB code provides comprehensive motion control for the Waveshare RoArm M2 robot
% RoArm M2 is a 4-DOF (4 Degrees of Freedom) robotic arm with JSON-based serial communication
% Includes initialization, joint control, cartesian motion, and trajectory planning
% 
% Robot Specifications:
% - 4 DOF (Base, Shoulder, Elbow, End Effector)
% - Serial Communication: UART/USB at 115200 baud
% - Control Protocol: JSON commands
% - Coordinates: Right-hand rule (X forward, Y left, Z up)

classdef RoarmM2_MotionControl
    properties
        % Robot parameters for RoArm M2
        DOF = 4;                          % Degrees of freedom
        % Joint names: 1=Base, 2=Shoulder, 3=Elbow, 4=End Effector (Gripper/Wrist)
        
        % Joint limits in degrees
        JointLimits_Lower = [-180, -90, -45, 45];      % Lower limits
        JointLimits_Upper = [180, 90, 180, 315];       % Upper limits
        
        % Default initial angles (home position)
        HomeAngles = [0, 0, 90, 180];   % Base, Shoulder, Elbow, End Effector
        
        % Serial communication properties
        SerialPort;
        BaudRate = 115200;
        Timeout = 5;
        CurrentJointAngles;
        TargetJointAngles;
        
        % RoArm M2 specific parameters
        WorkspaceRadius = 500;  % Workspace radius in mm
        MaxSpeed = 50;          % Speed coefficient (0-100)
        MaxAccel = 10;          % Acceleration coefficient (0-254)
    end
    
    methods
        %% Constructor - Initialize robot
        function obj = RoarmM2_MotionControl(portName)
            % Initialize RoArm M2 robot with serial port
            % Example: robot = RoarmM2_MotionControl('COM3')
            
            if nargin > 0
                obj.SerialPort = serial(portName);
                obj.SerialPort.BaudRate = obj.BaudRate;
                obj.SerialPort.Timeout = obj.Timeout;
                try
                    fopen(obj.SerialPort);
                    disp(['Connected to RoArm M2 on port: ' portName]);
                    pause(0.5);  % Wait for connection to stabilize
                catch
                    disp('Failed to connect to RoArm M2');
                    obj.SerialPort = [];
                end
            end
            
            % Initialize joint angles (home position in degrees)
            obj.CurrentJointAngles = obj.HomeAngles;
            obj.TargetJointAngles = obj.HomeAngles;
        end
        
        %% Move robot to joint angles (all joints in degrees)
        function moveToJointAngles(obj, targetAngles)
            % Move robot to specified joint angles using CMD_JOINTS_ANGLE_CTRL
            % Input: targetAngles - vector of 4 joint angles in degrees
            % Example: moveToJointAngles(robot, [0, 0, 90, 180])
            %   Base: -180 to 180 degrees
            %   Shoulder: -90 to 90 degrees
            %   Elbow: -45 to 180 degrees
            %   End Effector: 45 to 315 degrees (Gripper) or 45 to 315 (Wrist)
            
            % Validate input
            if length(targetAngles) ~= 4
                error('Target angles must be a 4-element vector [base, shoulder, elbow, endeffector]');
            end
            
            % Check joint limits
            if ~obj.checkJointLimits(targetAngles)
                error('Target angles exceed joint limits');
            end
            
            % Send JSON command to move all joints
            % Command T=122: CMD_JOINTS_ANGLE_CTRL
            % Parameters: b (base), s (shoulder), e (elbow), h (hand), spd (speed), acc (acceleration)
            cmdStr = sprintf('{"T":122,"b":%.1f,"s":%.1f,"e":%.1f,"h":%.1f,"spd":50,"acc":10}\n', ...
                targetAngles(1), targetAngles(2), targetAngles(3), targetAngles(4));
            
            obj.sendCommand(cmdStr);
            obj.CurrentJointAngles = targetAngles;
            obj.TargetJointAngles = targetAngles;
        end
        
        %% Move single joint (Base, Shoulder, Elbow, or EndEffector)
        function moveSingleJoint(obj, joint, angle)
            % Move a single joint by number
            % Input: joint - joint number (1=Base, 2=Shoulder, 3=Elbow, 4=EndEffector)
            %        angle - target angle in degrees
            
            if joint < 1 || joint > 4
                error('Joint number must be 1-4');
            end
            
            % Validate angle is within limits
            if angle < obj.JointLimits_Lower(joint) || angle > obj.JointLimits_Upper(joint)
                error(sprintf('Angle %f exceeds limits for joint %d: [%f, %f]', ...
                    angle, joint, obj.JointLimits_Lower(joint), obj.JointLimits_Upper(joint)));
            end
            
            % Send command: T=121 CMD_SINGLE_JOINT_ANGLE
            % Parameters: joint (1-4), angle (degrees), spd (speed), acc (acceleration)
            cmdStr = sprintf('{"T":121,"joint":%d,"angle":%.1f,"spd":50,"acc":10}\n', joint, angle);
            
            obj.sendCommand(cmdStr);
            
            % Update current position
            obj.CurrentJointAngles(joint) = angle;
        end
            % Move end-effector to cartesian position
            % Input: position - [x, y, z] in meters
            % Input: orientation - [roll, pitch, yaw] in degrees (optional)
            
            if nargin < 3
                orientation = [0, 0, 0];  % Default orientation
            end
            
            % Compute inverse kinematics
            targetAngles = obj.inverseKinematics(position, orientation);
            
            if isnan(targetAngles(1))
                warning('No IK solution found for target position');
                return;
            end
            
            % Move to computed angles
            obj.moveToJointAngles(targetAngles);
        end
        
        %% Forward Kinematics (DH Parameters)
        function endEffectorPose = forwardKinematics(obj, jointAngles)
            % Compute end-effector position and orientation using DH parameters
            % Input: jointAngles - joint angles in degrees
            % Output: endEffectorPose - [x, y, z, roll, pitch, yaw]
            
            % Convert to radians
            angles_rad = deg2rad(jointAngles);
            
            % DH Parameters for ro-arm M2 (example values)
            % [a, d, alpha, theta_offset]
            a = [0, obj.LinkLengths(1), obj.LinkLengths(2), 0, 0, 0];
            d = [0, 0, 0, obj.LinkLengths(4), 0.1, 0];
            alpha = [90, 0, 0, 90, 0, 0];
            theta_offset = [0, 0, 0, 0, 0, 0];
            
            % Compute transformation matrices
            T = eye(4);
            
            for i = 1:obj.DOF
                theta = angles_rad(i) + deg2rad(theta_offset(i));
                
                % DH Transformation matrix
                cth = cos(theta);
                sth = sin(theta);
                cal = cos(deg2rad(alpha(i)));
                sal = sin(deg2rad(alpha(i)));
                
                Ti = [cth, -sth*cal, sth*sal, a(i)*cth;
                      sth, cth*cal, -cth*sal, a(i)*sth;
                      0, sal, cal, d(i);
                      0, 0, 0, 1];
                
                T = T * Ti;
            end
            
            % Extract position
            position = T(1:3, 4)';
            
            % Extract orientation (Euler angles)
            R = T(1:3, 1:3);
            roll = atan2(R(3,2), R(3,3));
            pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
            yaw = atan2(R(2,1), R(1,1));
            
            endEffectorPose = [position, rad2deg([roll, pitch, yaw])];
        end
        
        %% Inverse Kinematics
        function jointAngles = inverseKinematics(obj, position, orientation)
            % Compute joint angles for given end-effector position
            % Using numerical method (Newton-Raphson)
            
            if nargin < 3
                orientation = [0, 0, 0];
            end
            
            % Initial guess (current position)
            q_init = obj.CurrentJointAngles;
            
            % Target pose
            targetPose = [position, orientation];
            
            % Optimization options
            options = optimoptions('fmincon', 'Display', 'off', 'TolFun', 1e-6);
            
            % Cost function (minimize position error)
            costFunc = @(q) norm(obj.forwardKinematics(q)(1:3) - position);
            
            % Non-linear constraint for joint limits
            nonlincon = @(q) deal([], []);  % No non-linear constraints
            
            % Solve IK
            try
                jointAngles = fmincon(costFunc, q_init, [], [], [], [], ...
                    obj.JointLimits_Lower, obj.JointLimits_Upper, nonlincon, options);
            catch
                jointAngles = nan(1, 6);  % Return NaN if IK fails
            end
        end
        
        %% Generate linear trajectory
        function trajectory = generateLinearTrajectory(obj, startAngles, endAngles)
            % Generate linear trajectory in joint space
            % Input: startAngles, endAngles - joint angle vectors
            % Output: trajectory - Nx6 matrix of joint angles
            
            % Time parameters
            samplingTime = 0.05;  % 50ms sampling
            
            % Compute time based on max velocity
            angles_diff = abs(endAngles - startAngles);
            max_angle_diff = max(angles_diff);
            velocities = obj.MaxVelocity ./ 60;  % Convert to deg/sample
            time_required = max(angles_diff ./ velocities) * samplingTime;
            
            numPoints = ceil(time_required / samplingTime);
            
            % Linear interpolation
            trajectory = zeros(numPoints, 6);
            for i = 1:6
                trajectory(:, i) = linspace(startAngles(i), endAngles(i), numPoints);
            end
        end
        
        %% Generate S-curve trajectory (smooth motion)
        function trajectory = generateSCurveTrajectory(obj, startAngles, endAngles)
            % Generate smooth S-curve trajectory
            
            samplingTime = 0.05;
            angles_diff = endAngles - startAngles;
            max_angle_diff = max(abs(angles_diff));
            
            % Duration based on max velocity and acceleration
            v_max = min(obj.MaxVelocity) / 60;  % deg/sample
            a_max = min(obj.MaxAcceleration) / 60;
            
            duration = max_angle_diff / v_max;
            numPoints = ceil(duration / samplingTime);
            
            trajectory = zeros(numPoints, 6);
            time_vector = (0:numPoints-1) * samplingTime;
            
            % S-curve for each joint
            for j = 1:6
                qi = startAngles(j);
                qf = endAngles(j);
                
                % Simple S-curve using quintic polynomial
                q = qi + (qf - qi) * (10*(time_vector/duration).^3 - 15*(time_vector/duration).^4 + 6*(time_vector/duration).^5);
                trajectory(:, j) = q;
            end
        end
        
        %% Execute trajectory
        function executeTrajectory(obj, trajectory)
            % Execute motion trajectory
            % Input: trajectory - Nx6 matrix of joint angles
            
            numPoints = size(trajectory, 1);
            samplingTime = 0.05;
            
            for i = 1:numPoints
                targetAngles = trajectory(i, :);
                
                % Send command to robot via serial port
                if ~isempty(obj.SerialPort) && isvalid(obj.SerialPort)
                    command = obj.formatMotionCommand(targetAngles);
                    fwrite(obj.SerialPort, command);
                end
                
                % Update current position
                obj.CurrentJointAngles = targetAngles;
                
                % Wait for next trajectory point
                pause(samplingTime);
            end
        end
        
        %% Format motion command for serial transmission
        function command = formatMotionCommand(obj, jointAngles)
            % Format joint angles into serial command
            % Example format: G0 Jx.xx Jx.xx ... (G-code style)
            
            angleStr = sprintf('G0');
            for i = 1:6
                angleStr = sprintf('%s J%d:%.2f', angleStr, i, jointAngles(i));
            end
            angleStr = sprintf('%s\r\n', angleStr);
            command = uint8(angleStr);
        end
        
        %% Check joint limits
        function isValid = checkJointLimits(obj, jointAngles)
            % Check if joint angles are within limits
            
            isValid = all(jointAngles >= obj.JointLimits_Lower) && ...
                      all(jointAngles <= obj.JointLimits_Upper);
        end
        
        %% Move to home position
        function moveHome(obj)
            % Move robot to home position (all joints at 0 degrees)
            homeAngles = [0, 0, 0, 0, 0, 0];
            obj.moveToJointAngles(homeAngles);
        end
        
        %% Emergency stop
        function emergencyStop(obj)
            % Stop all motion immediately
            if ~isempty(obj.SerialPort) && isvalid(obj.SerialPort)
                fwrite(obj.SerialPort, uint8('STOP\r\n'));
            end
            obj.CurrentJointAngles = obj.TargetJointAngles;
        end
        
        %% Get current joint angles
        function angles = getCurrentJointAngles(obj)
            angles = obj.CurrentJointAngles;
        end
        
        %% Get current end-effector pose
        function pose = getCurrentEndEffectorPose(obj)
            pose = obj.forwardKinematics(obj.CurrentJointAngles);
        end
        
        %% Cleanup - Close serial connection
        function delete(obj)
            if ~isempty(obj.SerialPort) && isvalid(obj.SerialPort)
                fclose(obj.SerialPort);
                delete(obj.SerialPort);
            end
        end
    end
end

%% Example Usage Script
% Uncomment and run this section to test the robot control

% % Initialize robot
% robot = RoarmM2_MotionControl('COM3');  % Replace 'COM3' with your actual port
% 
% % Move to home position
% robot.moveHome();
% 
% % Move to specific joint angles
% targetAngles = [30, 45, -30, 0, 45, 0];  % degrees
% robot.moveToJointAngles(targetAngles);
% 
% % Move to cartesian position
% targetPosition = [0.4, 0.0, 0.3];  % [x, y, z] in meters
% robot.moveToCartesian(targetPosition, [0, 0, 0]);
% 
% % Get current end-effector pose
% currentPose = robot.getCurrentEndEffectorPose();
% disp(['Current Position: X=' num2str(currentPose(1)) ...
%       ' Y=' num2str(currentPose(2)) ' Z=' num2str(currentPose(3))]);
% 
% % Clean up
% clear robot;
