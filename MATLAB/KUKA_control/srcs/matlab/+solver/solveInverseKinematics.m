function [q, status, yErr, i] = solveInverseKinematics(dhParameters, T, maxIterations, tolerance)
% Solve inverse kinematics using the Newton-Euler Method
% tau = M(q)qddot + C(q,qdot)qdot + g(q)

if isstruct(dhParameters)
    dhParameters = struct2table(dhParameters);
end
T = checkT(T);
if nargin < 3
    maxIterations = 500;
end
if nargin < 4
    tolerance = 1e-6;
end

W       = eye(6); %W=weighting matrix, diagonal.
Id      = eye(6); %Identity matrix

q = zeros(1, 6);    %initialize solution q
lambda   = 0.1;     %initialize damping parameter.
rejected = 0;       %initialize number of solutions
maxRejected = 100;
status = cast(-1, 'int8');

% compute initial error
yErr = computeError(dhParameters, q, T);

for i=1:maxIterations
    
    isSolution = isSolutionFound(W, yErr, tolerance);
    if isSolution
        status = cast(1, 'int8');
        break;
    end
    
    % compute the Jacobian
    J = computeJacobian(dhParameters, q);
    
    %Gauss-Newton with Levenberg-Marquadt
    %source: https://people.duke.edu/~hpgavin/ce281/lm.pdf
    % and 
    %"""
    %The Levenberg-Marquardt algorithm adaptively varies the
    %parameter updates between the gradient descent update and
    %the Gauss-Newton update:
    % [Jt*W*J + λ*I]*h = Jt*W*(y-y^)
    % where h=parameter update.
    %- if λ small: Gauss-Newton update
    %- if λ large: gradient descent update
    %"""
    h = (J'*W*J + lambda*Id)\(J'*W*yErr);
    % compute q basing on the parameter update h
    qUpdate = q + transpose(h);
    % compute new error
    yErrNew = computeError(dhParameters, qUpdate, T);
    
    if calculateNormError(W, yErrNew) < calculateNormError(W, yErr)
        % as the solution improves, \lambda is decreased: the
        % Levenberg-Marquardt method approaches the
        % Gauss-Newthon method, and the solution typically
        % accelerates to the local minimum.
        q = qUpdate;
        lambda = lambda/2;
        yErr = yErrNew;
        rejected = 0;
    else
        % do not update yErr and q
        rejected = rejected + 1;
        lambda = lambda*2;
        if rejected > maxRejected
            status = cast(0, 'int8');
            %q = [];
            break;
        end
    end
end
if status == -1
    status = cast(2, 'int8'); % solution not found -> return last iteration
end
end

function T = checkT(T)
dim = size(T);
if (dim(1) == 3 && dim(2) == 1) || ...
        (dim(1) == 1 && dim(2) == 3)
    % define homogeneous transform T from pose
    T = [eye(3), T; zeros(3, 1), 1];
end
end

function J = computeJacobian(dhParams, q)
J = zeros(6, 6); % initialize Jacobian
U = eye(4, 4);
for indx=6:-1:1
    T = solver.computeT(dhParams(indx, :), q(indx));
    U = T*U;
    d = [-U(1,1)*U(2,4) + U(2,1)*U(1,4)
        -U(1,2)*U(2,4) + U(2,2)*U(1,4)
        -U(1,3)*U(2,4) + U(2,3)*U(1,4)];
    delta = U(3,1:3)';  % nz oz az
    J(:,indx) = [d; delta];
end
end

function diffMotionMatrix = computeError(dhParams, q, T)
y = solver.solveForwardKinematics(dhParams, q);
% get transformation matrix from y to T
C = y\T;
translMatrix = C(1:3, 4);
rotMatrix = C(1:3, 1:3);
R = rotMatrix - eye(3, 3);
% convert from skew matrix [0 -v3 v2; v3 0 -v1; -v2 v1 0]
% to vector [v1; v2; v3].
v = 0.5*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
diffMotionMatrix = [translMatrix; v];
end

function status = isSolutionFound(W, err, tolerance)
% check if the error is in the tolerance range
status = calculateNormError(W, err) < tolerance;
end

function normError = calculateNormError(W, err)
normError = norm(W*err);
end