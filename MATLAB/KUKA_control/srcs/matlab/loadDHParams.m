function dhParameters = loadDHParams(toTable)	
if nargin < 1
    toTable = false;
end
%source for dimensions:
%http://www.wtech.com.tw/public/download/manual/kuka/KR_60HA.pdf
d       = [675;     0;      0;      -820;   0;      -170];
a       = [350;     850;    0;      0;      0;      0];
alpha   = [-pi/2;   0;      pi/2;   -pi/2;  pi/2;   pi];
%offset  = [0;       pi/2;  0;      0;      0;      0];
offset  = [0;       0;  0;      0;      0;      0];
dhParameters = struct( ...
    'd', d, ...
    'a', a, ...
    'alpha', alpha, ...
    'offset', offset);

if toTable
    dhParameters = struct2table(dhParameters);
end
end

