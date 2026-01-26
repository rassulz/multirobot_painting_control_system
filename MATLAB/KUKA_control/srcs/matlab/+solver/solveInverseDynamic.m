function [tau, status] = solveInverseDynamic(dhParameters, M, II, Fext, q, qd, qdd)
% calculate the joint forces and torques necessary to create
% the desired joint accelerations at the current joint position
% and velocities.
% Sources for the algorithm implementation and verification:
%  - https://github.com/petercorke/robotics-toolbox-matlab [@SerialLink]
%  - https://www.gamedeveloper.com/programming/create-your-own-inverse-dynamics-in-unity
%
% inputs:
%  dhParameters %DH parameters;
%  M            %mass vector;
%  II           %inertia matrix;
%  Fext         %external forces vector;
%  q            %joint position;
%  qd           %joint velocities;
%  qdd          %joint accelerations;
%
% return:
%  tau       %torque of the link (in reference to axes of rotation)
%
% further variable definitions:
%  G        %gravity vector
%  P        %cartesian coordinates of the next link's inboard joint with respect to
%           previous link reference frame
%  Pcm      %cartesian coordinates of the next link's center of mass with respect to
%           previous link reference fram
%  Z        %axis of rotation
% (forward iteration)
%  w        %angular velocity
%  wd       %angular acceleration
%  v        %linear acceleration
% (backward iteration)
%  linkTau  %torque of the link at the inboard joint
%  linkF    %force of the link at the inboard joint

G  = [-9.8; 0; 0];  % gravity vector
R_ = ones(3, 3, 6); % initialize R_ (3D matrix contianing all the R matrix from frame i-1 to frame i)
w  = zeros(3, 6);   % initialize angular velocity
wd = zeros(3, 6);   % initialize angular acceleration
vd = zeros(3, 6);   % initialize linear acceleration
vd(:, 1) = G;       % initialize linear acceleration with gravity (for the first iteration)

Z    = [0; 0; 1];    % set axis of rotation
Pcm  = [0; 0; 0];    % distance to the center of mass (assumed zeros, TDB)

linkTau = Fext(1:3); % initialize total torque link
linkF   = Fext(4:6); % initialize total force link

tau = zeros(1, 6);   % initialize output

for i=1:6
    dh = dhParameters(i, :);
    dh.a = dh.a/1000; % conversion mm -> m
    dh.d = dh.d/1000; % conversion mm -> m
    P = [dh.a; dh.d*sin(dh.alpha); dh.d*cos(dh.alpha)]; % move from i-1 to i
    Ti = solver.computeT(dh, q(i)+dh.offset);
    R = Ti(1:3, 1:3)';
    R_(:, :, i) = R; % save R in order to reuse it in the next step
    
    if i == 1
        wd(:, 1) = R*(Z*qdd(i));
        w(:, 1)  = R*(Z*qd(i));
        vd(:, i) = cross(wd(:, i), P) + ...
            cross(w(:, i), cross(w(:, i), P)) + ...
            R*G; % use G for the first iteration
    else
        % wd is the sum of wd(i-1) (expressed in frame i), and of a
        % term due to qdd(i) and of the velocity product term due
        % to wd and qd(i).
        wd(:, i) = R*(wd(:, i-1)+Z*qdd(i) + cross(w(:, i-1), Z*qd(i)));
        % w is the sum of w(i-1) (expressed in frame i), and of a
        % term due to qd(i).
        w(:, i)  = R*(w(:, i-1) + Z*qd(i));
        % calculate linear acceleration as the sum of 
        vd(:, i) = cross(wd(:, i), P) + ...
            cross(w(:, i), cross(w(:, i), P)) + ...
            R*vd(:, i-1);
    end
end

% backward iteration
for j=6:-1:1
    % calculate total force F on the center of mass (Newton)
    vHat = cross(wd(:, j), Pcm) + ...
        cross(w(:, j), cross(w(:, j), Pcm)) + vd(:, j);
    linkFtot = M(j)*vHat;
    % calculate total torque tau about the center of mass (Euler)
    linkTauTot = II(:, :, j)*wd(:, j) + cross(w(:, j), II(:, :, j)*w(:, j));
    
    if j == 6
        R = eye(3);
    else
        R = R_(:, :, j+1)';
    end
    dh = dhParameters(j, :);
    dh.a = dh.a/1000; % conversion mm -> m
    dh.d = dh.d/1000; % conversion mm -> m
    P = [dh.a; dh.d*sin(dh.alpha); dh.d*cos(dh.alpha)]; % move from i-1 to i
    % update the link torque for the next iteration (use previous computed
    % linkF)
    linkTau = R*(linkTau + cross(R'*P, linkF)) + ...
        cross(P + Pcm, linkFtot) + linkTauTot;
    % update f for the next iteration
    linkF = R*linkF + linkFtot;
    tau(j) = linkTau'*(R_(:, :, j)*Z);
end
status = cast(1, 'int8');
end
