function dynamicParametersStruct = gen_defaultDynamicParams()
dynamicParametersStruct.I = zeros(3, 3, 6);
dynamicParametersStruct.M = ones(6, 1);

% generate Inertia matrix
dynamicParametersStruct.I(:, :, 1) = eye(3);
dynamicParametersStruct.I(:, :, 2) = eye(3);
dynamicParametersStruct.I(:, :, 3) = eye(3);
dynamicParametersStruct.I(:, :, 4) = eye(3);
dynamicParametersStruct.I(:, :, 5) = eye(3);
dynamicParametersStruct.I(:, :, 6) = eye(3);

% generate external forces
dynamicParametersStruct.Fext = zeros(6, 1);
end