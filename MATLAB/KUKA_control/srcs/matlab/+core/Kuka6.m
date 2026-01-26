classdef Kuka6 < matlab.System & matlab.system.mixin.Propagates
    % Kuka6 System Object for dynamic and kinematic inverse solver.
    %
    % Solve the inverse kinematic or inverse dynamic of a Kuka6.
    %
    %#codegen
    
    %tunable properties could require a longer access time, hence following
    %properties are defined as nontunable, as they do not change during the
    %runtime.
    properties (Nontunable)
        %Solver Method
        Solver = 'Inverse Kinematic';
        %DH Parameters
        DHStruct = struct( ...
            'd', zeros(6, 1), ...
            'a', zeros(6, 1), ...
            'alpha', zeros(6, 1), ...
            'offset', zeros(6, 1));
        %Tolerance
        Tolerance = 1e-6;
        %Mass
        M = zeros(6, 1);
        %Inertia
        I = zeros(3, 3, 6);
        %External forces
        Fext = zeros(6, 1);
    end
    
    properties(Constant, Hidden)
        % String set for SolverMethod
        SolverSet = matlab.system.StringSet({...
            'Forward Kinematic', ...
            'Inverse Kinematic', ...
            'Inverse Dynamic', ...
            })
    end
    
    properties(DiscreteState)
    end
    
    % Pre-computed constants
    properties(Access = private)
        N = 6;
        DH = table();
    end
    
    methods(Access = protected)
        
        function validatePropertiesImpl(obj)
            n = obj.N;
            % attribute validation
            validateattributes(obj.DHStruct, {'struct'}, {'nonempty'}, 'Kuka6', 'dhParameters');
            
            coder.internal.errorIf(length(obj.DHStruct.d) ~= n, 'core:kuka6:DHParameterDimensionMismatch');
            coder.internal.errorIf(length(obj.DHStruct.a) ~= n, 'core:kuka6:DHParameterDimensionMismatch');
            coder.internal.errorIf(length(obj.DHStruct.alpha) ~= n, 'core:kuka6:DHParameterDimensionMismatch');
            coder.internal.errorIf(length(obj.DHStruct.offset) ~= n, 'core:kuka6:DHParameterDimensionMismatch');
            
            % attribute validate for Inverse Dynamic
            if strcmp(obj.Solver, 'Inverse Dynamic')
                validateattributes(obj.M, {'numeric'}, {'2d', 'nonempty', 'real', 'finite'}, 'Kuka6', 'Mass');
                validateattributes(obj.I, {'numeric'}, {'3d', 'nonempty', 'real', 'finite'}, 'Kuka6', 'Inertia');
                validateattributes(obj.Fext, {'numeric'}, {'2d', 'nonempty', 'real', 'finite'}, 'Kuka6', 'ExternalForces');
                
                coder.internal.errorIf(length(obj.M) ~= n, 'core:kuka6:MassDimensionMismatch');
                coder.internal.errorIf(size(obj.I) ~= [3 3 n], 'core:kuka6:InertiaDimensionMismatch');
                coder.internal.errorIf(length(obj.Fext) ~= n, 'core:kuka6:ExternalForcesDimensionMismatch');
            end
        end
        
        function flag = isInactivePropertyImpl(obj,prop)
            %isInactivePropertyImpl Identify inactive properties
            %   Return false if property is visible based on object
            %   configuration, for the command line and System block dialog
            
            props = {'Solver', 'DHStruct'};
            if strcmp(obj.Solver, 'Inverse Dynamic')
                props = [props, {'M', 'I', 'Fext'}];
            elseif strcmp(obj.Solver, 'Inverse Kinematic')
                props = [props, {'Tolerance'}];
            end
            flag = ~ismember(prop, props);
        end
        
        function num = getNumInputsImpl(obj)
            %getNumInputsImpl Define total number of inputs for system with optional inputs
            solver = obj.Solver;
            if strcmp(solver, 'Inverse Dynamic')
                num = 3;
            elseif strcmp(solver, 'Inverse Kinematic')
                num = 1;
            else %Forward Kinematic
                num = 2;
            end
        end
        
        function varargout = getInputNamesImpl(obj)
            %getInputNamesImpl Return input port names for System block
            solver = obj.Solver;
            if strcmp(solver, 'Inverse Dynamic')
                varargout = {'q', 'qd', 'qdd'};
            elseif strcmp(solver, 'Inverse Kinematic')
                varargout = {'targetPose'};
            else %Forward Kinematic
                varargout = {'q', 'dt'};
            end
        end
        
        function num = getNumOutputsImpl(obj)
            %getNumOutputsImpl Define total number of outputs for system with optional outputs
            if strcmp(obj.Solver, 'Forward Kinematic')
                num = 3;
            else
                num = 2; % [result, status]
            end
        end
        
        function varargout = getOutputNamesImpl(obj)
            %getInputNamesImpl Return input port names for System block
            solver = obj.Solver;
            if strcmp(solver, 'Inverse Kinematic')
                varargout = {'q', 'status'};
            elseif strcmp(obj.Solver, 'Inverse Dynamic')
                varargout = {'tau', 'status'};
            else %Forward Kinematic
                varargout = {'q_out', 'qd_out', 'qdd_out'};
            end
        end
        
        function [out, out2, out3]  = getOutputDataTypeImpl(obj)
            %getOutputDataTypeImpl Return data type for each output port
            out = 'double';
            out3 = 'double';
            if strcmp(obj.Solver, 'Forward Kinematic')
                out2 = 'double';
            else
                out2 = 'int8';
            end
        end
        
        function [out, out2, out3] = getOutputSizeImpl(obj)
            %getOutputSizeImpl Return size for each output port
            out2 = [1 1];
            out3 = [1 1];
            n = obj.N;
            if strcmp(obj.Solver, 'Inverse Kinematic')
                out = [1, n];
            elseif strcmp(obj.Solver, 'Inverse Dynamic')
                out = [1, n];
            else %Forward Kinematic
                out = [3, n];
                out2 = [3, n];
                out3 = [3, n];
            end
        end
        
        function [out, out2, out3] = isOutputFixedSizeImpl(~)
            %isOutputFixedSizeImpl Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
        end
        
        function [out, out2, out3] = isOutputComplexImpl(~)
            %getOutputDataTypeImpl Return data type for each output port
            out = false;
            out2 = false;
            out3 = false;
        end
        
        function icon = getIconImpl(obj)
            %getIconImpl Define icon for System block
            solver = obj.Solver;
            if strcmp(solver, 'Inverse Dynamic')
                filepath = fullfile('srcs', 'simulink', 'blockicons', 'solver-id-icon.jpg');
            elseif strcmp(solver, 'Inverse Kinematic')
                filepath = fullfile('srcs', 'simulink', 'blockicons', 'solver-ik-icon.jpg');
            else %Forward Kinematic
                filepath = fullfile('srcs', 'simulink', 'blockicons', 'solver-fk-icon.jpg');
            end
            icon = matlab.system.display.Icon(filepath);
        end
        
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.DH = struct2table(obj.DHStruct);
        end
        
        function varargout = stepImpl(obj, varargin)
            if strcmp(obj.Solver, 'Inverse Kinematic')
                T = varargin{1};
                dim = size(T);
                if (dim(1) == 3 && dim(2) == 1) || ...
                        (dim(1) == 1 && dim(2) == 3)
                    TT = [1, 0, 0, T(1, 1);
                        0, -1, 0, T(2, 1);
                        0, 0, -1, T(3, 1);
                        0, 0, 0, 1];
                else
                    TT = T;
                end
                [q, status] = obj.solveInverseKinematics(TT);
                varargout{1} = q;
                varargout{2} = status;
            elseif strcmp(obj.Solver, 'Inverse Dynamic')
                [tau, status] = obj.solveInverseDynamic(varargin{:});
                varargout{1} = tau;
                varargout{2} = status;
            else
                [q, qd, qdd] = obj.solveForwardKinematics(varargin{:});
                varargout{1} = q;
                varargout{2} = qd;
                varargout{3} = qdd;
            end
        end
        
        function resetImpl(~)
        end
        
    end
    
    methods(Access = private)
        
        function [y_out, qd_out, qdd_out] = solveForwardKinematics(obj, q, dt)
            coder.internal.errorIf(nargin > 2 & nargout < 3, 'core:kuka6:solveForwardKinematics:NotEnoughInputArgument');
            n = obj.N;
            q_out = zeros(3, n);
            T = eye(4, 4); % initialize T for first computation
            for i=1:n
                T = T*obj.computeT(obj.DH(i, :), q(i));
                q_out(:, i) = T(1:3, 4);
            end
            if nargout > 1 && nargin > 2 % if time is given and gradient can be calculated
                y_out = q_out;
                qd_out = gradient(q_out, dt);
                qdd_out = gradient(qd_out, dt);
            else
                y_out = T;
                qd_out = zeros(3, n);
                qdd_out = zeros(3, n);
            end
        end
        
        function [q, status] = solveInverseKinematics(obj, T)
            maxIterations = 50;
            
            lambda  = 0.1;               %damping parameter.
            W       = eye(6, 6); %W=weighting matrix, diagonal.
            Id       = eye(6, 6); %Identity matrix
            
            q = zeros(1, 6);    %initialize solution q
            rejected = 0;           %initialize number of solutions
            status = cast(0, 'int8');
            
            for i=1:maxIterations
                % compute error
                yErr = obj.computeError(q, T);
                
                isSolution = obj.isSolutionFound(W, yErr);
                if isSolution
                    status = cast(1, 'int8');
                    break;
                end
                
                % compute the Jacobian
                J = obj.computeJacobian(q);
                
                %Gauss-Newton with Levenberg-Marquadt
                %source: https://people.duke.edu/~hpgavin/ce281/lm.pdf
                %"""
                %The Levenberg-Marquardt algorithm adaptively varies the
                %parameter updates between the gradient descent update and
                %the Gauss-Newton update:
                % [Jt*W*J + λ*I]*h = Jt*W(y-y^)
                % where h=parameter update.
                %- if λ small: Gauss-Newton update
                %- if λ large: gradient descent update
                %"""
                h = (J'*W*J + lambda*Id)\(J'*W*yErr);
                % compute q basing on the parameter update h
                qUpdate = q + transpose(h);
                % compute new error
                yErrNew = obj.computeError(qUpdate, T);
                
                if obj.calculateNormError(W, yErrNew) < obj.calculateNormError(W, yErr)
                    % as the solution improves, \lambda is decreased: the
                    % Levenberg-Marquardt method approaches the
                    % Gauss-Newthon method, and the solution typically
                    % accelerates to the local minimum.
                    q = qUpdate;
                    lambda = lambda/2;
                else
                    rejected = rejected + 1;
                    lambda = lambda*2;
                    if rejected >= maxIterations
                        status = cast(2, 'int8');
                    end
                end
            end
        end
        
        function [tau, status] = solveInverseDynamic(obj, q, qd, qdd)
            % calculate the joint forces and torques necessary to create
            % the desired joint accelerations at the current joint position
            % and velocities.
            
            Rall = ones(3, 3, 6);   % initialize R
            w = zeros(3, 1);        % initialize velocity
            wd = zeros(3, 1);       % initialize acceleration
            vd = [0; 0; -9.8];      % initialize linear acceleration with gravity
            z0 = [0; 0; 1];
            r = [-0.25; 0; 0];
            status = cast(0, 'int8');
            dhParams = obj.DH;
            Mass = obj.M;
            Inertia = obj.I;
            nn  = obj.Fext(1:3);
            f   = obj.Fext(4:6);
            
            % initizialize output of forward iterations
            Fm = zeros(3, 6);
            Nm = zeros(3, 6);
            
            % forward iteration
            for i=1:6
                dh = dhParams(i, :);
                pStar = [dh.a; dh.d*cos(dh.alpha); dh.d*sin(dh.alpha)]; % move from i-1 to i
                Ti = obj.computeT(dh, q(i));
                R = Ti(1:3, 1:3)';
                Rall(:, :, i) = R; % save R in order to reuse it in the next step
                
                % wd is the sum of wd(i-1) (expressed in frame i), and of a
                % term due to qdd(i) and of the velocity product term due
                % to wd and qd(i).
                wd = R*(wd+z0*qdd(i) + cross(w, z0*qd(i)));
                % w is the sum of w(i-1) (expressed in frame i), and of a
                % term due to qd(i).
                w = R*(w + z0*qd(i));
                vd = cross(wd, pStar) + cross(w, cross(w, pStar)) + R*vd;
                
                vHat = cross(wd, r) + cross(w, cross(w, r)) + vd;
                
                Fm(:, i) = Mass(i)*vHat;
                Nm(:, i) = Inertia(:, :, i)*wd + cross(w, Inertia(:, :, i)*w);
            end
            
            % backward iteration
            tau = zeros(1, 6); % initialize output of backward iteration
            for j=6:-1:1
                if j == 6
                    R = eye(3);
                else
                    R = Rall(:, :, j+1);
                end
                dh = dhParams(i, :);
                pStar = [dh.a; dh.d*cos(dh.alpha); dh.d*sin(dh.alpha)]; % move from i-1 to i
                %update nn
                nn = R*(nn + cross(R*pStar, f)) + cross(pStar + r, Fm(:, j)) + Nm(j);
                %upd
                f = R*f + Fm(:, j);
                tau(j) = nn'*(R*z0);
            end
            status = cast(1, 'int8');
        end
        
        function J = computeJacobian(obj, q)
            dhParams = obj.DH;
            J = zeros(6, 6); % initialize Jacobian
            U = eye(4, 4);
            for indx=6:-1:1
                T = obj.computeT(dhParams(indx, :), q(indx));
                U = T*U;
                d = [-U(1,1)*U(2,4) + U(2,1)*U(1,4)
                    U(1,2)*U(2,4) + U(2,2)*U(1,4)
                    -U(1,3)*U(2,4) + U(2,3)*U(1,4)];
                delta = U(3,1:3)';  % nz oz az
                J(:,indx) = [d; delta];
            end
        end
        
        function diffMotionMatrix = computeError(obj, q, T)
            y = obj.solveForwardKinematics(q);
            C = y\T;
            translMatrix = C(1:3, 4);
            rotMatrix = C(1:3, 1:3);
            R = rotMatrix - eye(3, 3);
            v = 0.5*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
            diffMotionMatrix = [translMatrix; v];
        end
        
        function status = isSolutionFound(obj, W, err)
            % check if the error is in the tolerance range
            status = obj.calculateNormError(W, err) < obj.Tolerance;
        end
        
    end
    
    methods (Static)
        
        function T = computeT(linkDH, q)
            T = [cos(q)     -sin(q)*cos(linkDH.alpha)   sin(q)*sin(linkDH.alpha)    linkDH.a*cos(q);
                sin(q)      cos(q)*cos(linkDH.alpha)    -cos(q)*sin(linkDH.alpha)   linkDH.a*sin(q);
                0           sin(linkDH.alpha)           cos(linkDH.alpha)           linkDH.d;
                0           0                           0                           1];
        end
        
        function normError = calculateNormError(W, err)
            normError = norm(W*err);
        end
    end
end
