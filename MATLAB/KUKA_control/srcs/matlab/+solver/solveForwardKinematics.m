function q_out = solveForwardKinematics(dhParameters, q, completeSolution)
if nargin < 3
    completeSolution = false;
end
n = 6;
y = zeros(3, n);
T = eye(4, 4); % initialize T for first computation
for i=1:n
    T = T*solver.computeT(dhParameters(i, :), q(i));
    y(:, i) = T(1:3, 4);
end
if completeSolution % if complete solution is required, then all the joint position are returned
    q_out = y;
else
    q_out = T;
end
end

