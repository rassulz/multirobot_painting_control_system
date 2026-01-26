classdef InverseKinematicTest < matlab.unittest.TestCase
    
    properties (Access=private)
        DHParams        table;
    end
    
    properties (Access=public) % set public just for development/debug purposes
        RT_SerialLink   SerialLink;
    end
    
    properties (TestParameter)
        targetPose = {
            [1150; -200; -365];
            [1280;  100; -395];
            [1140; -100; -255];
            [1270;  100; -245];
            [1150; -200; -365];
        }
        expectedQ = {
            [-0.1726,   0.0595,   -0.0226,    0.0881,   -0.0315,    0.0766];
            [ 0.0774,   0.0996,   -0.2041,   -0.0475,    0.0899,   -0.0266];
            [-0.0879,  -0.0687,    0.1320,    0.0472,   -0.0534,    0.0368];
            [ 0.0781,  -0.0784,   -0.0130,   -0.0455,    0.0816,   -0.0291];
            [-0.1726,   0.0595,   -0.0226,    0.0881,   -0.0315,    0.0766];
            }
    end
    
    methods (TestClassSetup)
        
        function loadParameters(testCase)
            % load parameters: dh-parameters
            testCase.DHParams = loadDHParams(true);
        end
        
    end
    
    methods (Test, ParameterCombination='sequential')
        
        function startInverseKinematicTest(testCase, targetPose, expectedQ)
            T = [eye(3), targetPose;
                zeros(1, 3), 1];
            q = solver.solveInverseKinematics(testCase.DHParams, T);
            testCase.verifyLessThan( ...
                abs(q - expectedQ), 1e-4); % allow tolerance
        end
        
    end
end
