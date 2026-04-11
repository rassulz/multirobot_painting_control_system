%% SINGLE ROARM CONTROL FROM TERMINAL XYZ INPUT
% Управление одним манипулятором RoArm через ввод координат в MATLAB-терминале.
% Формат ввода координат: [x y z] в миллиметрах.
%
% Команды в терминале:
%   - [x y z] : перейти в указанную точку
%   - home    : вернуть манипулятор в HOME
%   - exit    : завершить программу

clc; clear; close all;

%% ===================== 1) SETTINGS =====================
ARM_IP = "192.168.1.101";
BASE_URL = "/js?json=";

HTTP_TIMEOUT = 2.5;
MAX_SEND_RETRIES = 2;
SEND_RETRY_DELAY = 0.20;

DEFAULT_T = 0;      % ориентация инструмента (град)
DEFAULT_SPD = 0.30; % скорость для T=104

%% ===================== 2) STARTUP =====================
disp('========================================');
disp(' Single RoArm XYZ Terminal Control ');
disp('========================================');
disp(['ARM IP: ' char(ARM_IP)]);
disp('Команды: [x y z], home, exit');

% Инициализация в HOME
disp('Homing manipulator...');
send_json_command(ARM_IP, BASE_URL, '{"T":100}', HTTP_TIMEOUT, MAX_SEND_RETRIES, SEND_RETRY_DELAY);
pause(2.0);

%% ===================== 3) MAIN LOOP =====================
while true
    userInput = input('\nВведите [x y z] / home / exit: ', 's');
    cmd = strtrim(lower(userInput));

    if isempty(cmd)
        disp('Пустой ввод. Введите [x y z], home или exit.');
        continue;
    end

    if strcmp(cmd, 'exit')
        disp('Завершение программы.');
        break;
    end

    if strcmp(cmd, 'home')
        send_json_command(ARM_IP, BASE_URL, '{"T":100}', HTTP_TIMEOUT, MAX_SEND_RETRIES, SEND_RETRY_DELAY);
        pause(1.5);
        continue;
    end

    xyz = str2num(userInput); %#ok<ST2NM>
    if ~(isnumeric(xyz) && numel(xyz) == 3)
        disp('Неверный формат. Пример: [250 100 180]');
        continue;
    end

    x = xyz(1);
    y = xyz(2);
    z = xyz(3);

    json_cmd = sprintf('{"T":104,"x":%.2f,"y":%.2f,"z":%.2f,"t":%.2f,"spd":%.2f}', ...
        x, y, z, DEFAULT_T, DEFAULT_SPD);

    send_json_command(ARM_IP, BASE_URL, json_cmd, HTTP_TIMEOUT, MAX_SEND_RETRIES, SEND_RETRY_DELAY);
    fprintf('Отправлено: x=%.2f, y=%.2f, z=%.2f\n', x, y, z);
    pause(0.2);
end

%% ===================== LOCAL FUNCTION =====================
function response = send_json_command(ip, base_url, json_cmd, timeout_s, maxRetries, retryDelay)
    if nargin < 5 || isempty(maxRetries)
        maxRetries = 1;
    end
    if nargin < 6 || isempty(retryDelay)
        retryDelay = 0.15;
    end

    url = ['http://' char(ip) char(base_url) urlencode(json_cmd)];
    response = '';

    for attempt = 1:maxRetries
        try
            response = webread(url, weboptions('Timeout', timeout_s));
            return;
        catch ME
            if attempt < maxRetries
                pause(retryDelay);
            else
                warning(['HTTP send failed to %s after %d attempt(s): %s\n' ...
                         'Проверьте IP робота, WiFi сеть и доступность web API.'], ...
                        char(ip), maxRetries, ME.message);
            end
        end
    end
end
