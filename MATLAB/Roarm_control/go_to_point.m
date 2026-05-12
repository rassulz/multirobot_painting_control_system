%% SINGLE ROARM CONTROL FROM TERMINAL XYZ INPUT
% Управление одним манипулятором RoArm через ввод координат в MATLAB-терминале.
% Формат ввода координат: [x y z] или x y z, в миллиметрах.
%
% Команды в терминале:
%   - [x y z] : перейти в указанную точку
%   - home    : вернуть манипулятор в HOME
%   - exit    : завершить программу

clc; clear; close all;

%% ===================== 1) SETTINGS =====================
ARM_IP = "172.31.17.101";
BASE_URL = "/js?json=";

HTTP_TIMEOUT = 2.5;
MAX_SEND_RETRIES = 2;
SEND_RETRY_DELAY = 0.20;

PRIMARY_CARTESIAN_T = 104;
FALLBACK_CARTESIAN_T = 1041;
DEFAULT_T_RAD = 0.0;   % Ориентация инструмента в радианах.
DEFAULT_SPD = 0.30;    % Скорость Cartesian-команды.

LED_CMD_T = 114;
LED_ON = 255;
LED_OFF = 0;

XYZ_MIN = [-500, -500, 0];
XYZ_MAX = [500, 500, 500];
MAX_RADIUS_MM = 500;

%% ===================== 2) STARTUP =====================
disp('========================================');
disp(' Single RoArm XYZ Terminal Control ');
disp('========================================');
disp(['ARM IP: ' char(ARM_IP)]);
disp('Команды: [x y z], home, exit');
disp(sprintf('Cartesian command IDs: primary T=%d, fallback T=%d', ...
    PRIMARY_CARTESIAN_T, FALLBACK_CARTESIAN_T));

% Инициализация в HOME
disp('Homing manipulator...');
[ledOk, ledMsg] = send_led_command( ...
    ARM_IP, BASE_URL, LED_CMD_T, LED_OFF, HTTP_TIMEOUT, MAX_SEND_RETRIES, SEND_RETRY_DELAY);
if ~ledOk
    warning('LED command failed: %s', ledMsg);
end
[homeOk, ~, homeMsg] = send_json_command( ...
    ARM_IP, BASE_URL, '{"T":100}', HTTP_TIMEOUT, MAX_SEND_RETRIES, SEND_RETRY_DELAY);
if homeOk
    disp('HOME command accepted.');
else
    warning('HOME command failed: %s', homeMsg);
end
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
        [ledOk, ledMsg] = send_led_command( ...
            ARM_IP, BASE_URL, LED_CMD_T, LED_OFF, HTTP_TIMEOUT, MAX_SEND_RETRIES, SEND_RETRY_DELAY);
        if ~ledOk
            warning('LED command failed: %s', ledMsg);
        end
        [homeOk, ~, homeMsg] = send_json_command( ...
            ARM_IP, BASE_URL, '{"T":100}', HTTP_TIMEOUT, MAX_SEND_RETRIES, SEND_RETRY_DELAY);
        if homeOk
            disp('Manipulator moved to HOME.');
        else
            warning('HOME command failed: %s', homeMsg);
        end
        pause(1.5);
        continue;
    end

    [xyz, parseMsg] = parse_xyz_input(userInput);
    if isempty(xyz)
        fprintf('Неверный формат: %s\n', parseMsg);
        disp('Пример: [250 100 180] или 250 100 180');
        continue;
    end

    [isValid, validationMsg] = validate_xyz(xyz, XYZ_MIN, XYZ_MAX, MAX_RADIUS_MM);
    if ~isValid
        fprintf('Координаты отклонены: %s\n', validationMsg);
        continue;
    end

    x = xyz(1);
    y = xyz(2);
    z = xyz(3);

    [ledOk, ledMsg] = send_led_command( ...
        ARM_IP, BASE_URL, LED_CMD_T, LED_ON, HTTP_TIMEOUT, MAX_SEND_RETRIES, SEND_RETRY_DELAY);
    if ~ledOk
        warning('LED command failed: %s', ledMsg);
    end
    [moveOk, usedCmdT, moveResponse, moveMsg] = send_cartesian_command( ...
        ARM_IP, BASE_URL, [x, y, z], DEFAULT_T_RAD, DEFAULT_SPD, ...
        PRIMARY_CARTESIAN_T, FALLBACK_CARTESIAN_T, ...
        HTTP_TIMEOUT, MAX_SEND_RETRIES, SEND_RETRY_DELAY);

    if moveOk
        fprintf('Отправлено: T=%d, x=%.2f, y=%.2f, z=%.2f, t=%.4f rad, spd=%.2f\n', ...
            usedCmdT, x, y, z, DEFAULT_T_RAD, DEFAULT_SPD);
        if ~isempty(moveResponse)
            fprintf('Ответ робота: %s\n', stringify_response(moveResponse));
        end
    else
        fprintf('Команда движения не отправлена: %s\n', moveMsg);
    end

    pause(0.2);
end

%% ===================== LOCAL FUNCTIONS =====================
function [success, usedCmdT, response, message] = send_cartesian_command( ...
    ip, base_url, xyz, toolAngleRad, speed, primaryCmdT, fallbackCmdT, ...
    timeout_s, maxRetries, retryDelay)

    usedCmdT = primaryCmdT;
    primaryCmd = build_cartesian_json(primaryCmdT, xyz, toolAngleRad, speed);
    [success, response, primaryMsg] = send_json_command( ...
        ip, base_url, primaryCmd, timeout_s, maxRetries, retryDelay);

    if success && is_valid_command_response(response)
        message = sprintf('Primary Cartesian command T=%d accepted.', primaryCmdT);
        return;
    end

    primaryDetail = strtrim(primaryMsg);
    if isempty(primaryDetail)
        primaryDetail = 'empty or invalid response';
    end

    usedCmdT = fallbackCmdT;
    fallbackCmd = build_cartesian_json(fallbackCmdT, xyz, toolAngleRad, speed);
    [success, response, fallbackMsg] = send_json_command( ...
        ip, base_url, fallbackCmd, timeout_s, maxRetries, retryDelay);

    if success && is_valid_command_response(response)
        message = sprintf(['Primary Cartesian command T=%d failed (%s). ' ...
            'Fallback T=%d accepted.'], primaryCmdT, primaryDetail, fallbackCmdT);
        fprintf('%s\n', message);
        return;
    end

    fallbackDetail = strtrim(fallbackMsg);
    if isempty(fallbackDetail)
        fallbackDetail = 'empty or invalid response';
    end

    success = false;
    response = '';
    message = sprintf(['Primary Cartesian command T=%d failed (%s). ' ...
        'Fallback T=%d failed (%s).'], ...
        primaryCmdT, primaryDetail, fallbackCmdT, fallbackDetail);
end

function [success, message] = send_led_command( ...
    ip, base_url, cmdT, ledValue, timeout_s, maxRetries, retryDelay)

    json_cmd = sprintf('{"T":%d,"led":%d}', cmdT, ledValue);
    [success, ~, message] = send_json_command( ...
        ip, base_url, json_cmd, timeout_s, maxRetries, retryDelay);
end

function json_cmd = build_cartesian_json(cmdT, xyz, toolAngleRad, speed)
    json_cmd = sprintf( ...
        '{"T":%d,"x":%.2f,"y":%.2f,"z":%.2f,"t":%.4f,"spd":%.2f}', ...
        cmdT, xyz(1), xyz(2), xyz(3), toolAngleRad, speed);
end

function [xyz, message] = parse_xyz_input(userInput)
    xyz = [];
    message = '';

    cleaned = strtrim(userInput);
    if isempty(cleaned)
        message = 'пустой ввод';
        return;
    end

    hasOpenBracket = startsWith(cleaned, '[');
    hasCloseBracket = endsWith(cleaned, ']');
    if xor(hasOpenBracket, hasCloseBracket)
        message = 'скобки должны быть парными: [x y z]';
        return;
    end

    if hasOpenBracket && hasCloseBracket
        cleaned = cleaned(2:end-1);
    end

    cleaned = strrep(cleaned, ',', ' ');
    invalidChars = regexprep(cleaned, '[-+0-9eE\.\s]', '');
    if ~isempty(strtrim(invalidChars))
        message = 'допустимы только числа, пробелы, запятые и внешние скобки';
        return;
    end

    tokens = regexp(cleaned, '[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?', 'match');

    if numel(tokens) ~= 3
        message = 'ожидалось ровно 3 числа';
        return;
    end

    xyz = str2double(tokens);
    if any(isnan(xyz))
        xyz = [];
        message = 'не удалось распознать числа';
    end
end

function [isValid, message] = validate_xyz(xyz, xyzMin, xyzMax, maxRadiusMm)
    isValid = false;
    message = '';

    if ~isnumeric(xyz) || numel(xyz) ~= 3
        message = 'ожидался вектор из трех чисел';
        return;
    end

    if any(~isfinite(xyz))
        message = 'координаты должны быть конечными числами';
        return;
    end

    if any(xyz < xyzMin) || any(xyz > xyzMax)
        message = sprintf(['допустимый диапазон: x=[%.0f, %.0f], ' ...
            'y=[%.0f, %.0f], z=[%.0f, %.0f] мм'], ...
            xyzMin(1), xyzMax(1), xyzMin(2), xyzMax(2), xyzMin(3), xyzMax(3));
        return;
    end

    radialReach = hypot(xyz(1), xyz(2));
    if radialReach > maxRadiusMm
        message = sprintf('горизонтальный радиус %.1f мм превышает лимит %.1f мм', ...
            radialReach, maxRadiusMm);
        return;
    end

    isValid = true;
end

function tf = is_valid_command_response(response)
    tf = false;

    if isempty(response)
        return;
    end

    if isstring(response) || ischar(response)
        tf = ~isempty(strtrim(char(response)));
        return;
    end

    if isstruct(response)
        tf = ~isempty(fieldnames(response));
        return;
    end

    tf = true;
end

function [success, response, message] = send_json_command( ...
    ip, base_url, json_cmd, timeout_s, maxRetries, retryDelay)

    if nargin < 5 || isempty(maxRetries)
        maxRetries = 1;
    end
    if nargin < 6 || isempty(retryDelay)
        retryDelay = 0.15;
    end

    url = ['http://' char(ip) char(base_url) urlencode(json_cmd)];
    response = '';
    success = false;
    message = 'request not sent';

    for attempt = 1:maxRetries
        try
            response = webread(url, weboptions('Timeout', timeout_s));
            success = true;

            if is_valid_command_response(response)
                message = sprintf('HTTP request succeeded on attempt %d.', attempt);
            else
                success = false;
                message = sprintf('HTTP request returned empty/invalid response on attempt %d.', attempt);
            end

            return;
        catch ME
            message = sprintf('HTTP send failed on attempt %d/%d: %s', ...
                attempt, maxRetries, ME.message);
            if attempt < maxRetries
                pause(retryDelay);
            end
        end
    end
end

function out = stringify_response(response)
    if isstring(response)
        out = char(response);
    elseif ischar(response)
        out = response;
    elseif isstruct(response)
        try
            out = jsonencode(response);
        catch
            out = '[struct response]';
        end
    else
        try
            out = char(string(response));
        catch
            out = '[non-text response]';
        end
    end
end
