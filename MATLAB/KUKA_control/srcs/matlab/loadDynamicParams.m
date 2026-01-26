function dynamicParametersStruct = loadDynamicParams()
%http://sksaha.com/sites/default/files/upload_data/documents/NaCoMM2011_2.pdf
dynamicParametersStruct.I = ones(3, 3, 6);
dynamicParametersStruct.M = ones(6, 1);

% generate Inertia matrix
dynamicParametersStruct.I(:, :, 1) = [0.321807, -0.0175473, -0.145272;
    -0.0175473, 0.467086, -0.0136195;
    -0.145272, -0.0136195, 0.477758];
dynamicParametersStruct.I(:, :, 2) = [0.54137, 0.000435555, -0.00455128;
    0.000435555, 0.55181, 0.0174856;
    -0.00455128, 0.0174856, 0.0436346 ];
dynamicParametersStruct.I(:, :, 3) = [0.775113, -0.00947011, 0.0247925;
    -0.00947011, 0.750399, 0.00744338;
    0.0247925, 0.00744338, 0.207558];
dynamicParametersStruct.I(:, :, 4) = [0.00950111, 0.00163357, -8.41256e-006;
    0.00163357, 0.0201731, 2.69102e-006;
    -8.41256e-006, 2.69102e-006, 0.0241324];
dynamicParametersStruct.I(:, :, 5) = [0.00151607, 0.000409305, 0;
    0.000409305, 0.00355535, 0;
    0.0, 0.0, 0.00358942];
dynamicParametersStruct.I(:, :, 6) = [6.20477e-006, 0.0, 0.0;
    0.0, 3.01352e-006, 0.0;
    0.0, 0.0, 3.29301e-006];

% generate mass vector
dynamicParametersStruct.M = [26.9797;
    15.9204;
    25.8524;
    4.08846;
    1.61544;
    0.0160103];

% generate external forces
dynamicParametersStruct.Fext = zeros(6, 1);
end