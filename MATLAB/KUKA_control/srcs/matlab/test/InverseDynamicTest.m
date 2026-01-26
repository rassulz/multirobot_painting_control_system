classdef InverseDynamicTest < matlab.unittest.TestCase
    
    properties (Access=private)
        DHParams        table;
        DynParams       struct;
    end
    
    properties (Access=public) % set public just for development/debug purposes
        RT_SerialLink   SerialLink;
    end
    
    properties (ClassSetupParameter)
        dynamicPropertiesGenerator = {
            @gen_defaultDynamicParams;
            @gen_dynamicParams;
            }
    end
    
    properties (TestParameter)
        q = {
            [0.3709;  2.2249; -0.9558; -0.3391; -2.8008; -2.0288];
            [1.0230; -1.0629;  2.5038; -2.3992;  3.0688;  0.2512];
            [1.3001;  3.1384; -1.3330; -0.5371; -0.2209;  1.6585];
            }
            
        qd = {
            zeros(6, 1);
            [0.1; 0; 0.1; 1; 0.1; 0.001];
            [0; 0.15; 0; 0.15; 0; 0];
            }
    end
    
    methods (TestClassSetup)
        
        function loadParameters(testCase, dynamicPropertiesGenerator)
            % load parameters: dh-parameters and dynamic attributes
            testCase.DHParams = loadDHParams(true);
            testCase.DynParams = dynamicPropertiesGenerator();
        end
        
        function generateSerialLinkRT(testCase)
            % Generate SerialLink
            % create six links robot with Robotic Toolbox for validation
            link_RT = cell(size(testCase.DHParams, 1), 1);
            for i=1:size(testCase.DHParams, 1)
                dh = testCase.DHParams(i, :);
                link_RT{i} = Link('d', dh.d, 'a', dh.a, 'alpha', dh.alpha, 'offset', dh.offset, ...
                    'm', testCase.DynParams.M(i), 'I', testCase.DynParams.I(:, :, i));
            end
            testCase.RT_SerialLink = SerialLink([link_RT{:}]);
            disp('Created serialLink_RT:');
            disp(char(testCase.RT_SerialLink));
        end
        
    end
    
    
    methods (Test, ParameterCombination='pairwise')
        
        function startInverseDynamicTest(testCase, q, qd)
            qdd = zeros(6, 1);
            tau = solver.solveInverseDynamic(testCase.DHParams, ...
                testCase.DynParams.M, ...
                testCase.DynParams.I, ...
                testCase.DynParams.Fext, ...
                q, qd, qdd);
            tauRT = testCase.RT_SerialLink.rne(q', qd', qdd', [0, -9.8, 0]);
            testCase.verifyLessThan( ...
                abs(tau - tauRT), 1e-6); % allow tolerance
        end
        
    end
end
