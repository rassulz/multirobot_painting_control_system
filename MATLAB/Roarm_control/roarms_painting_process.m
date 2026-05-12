%% ================================================================
%  roarms_painting_process_updated.m
%  1-Station Conveyor Painting System
%  Single station at conveyor center
%  Painting only side faces and top face
%% ================================================================

function roarms_painting_process_updated()
    clc;
    close all;
    cfg = buildConfig();
    buildMainGUI(cfg);
end

%% ================================================================
%  SECTION 1: CONFIGURATION
%% ================================================================
function cfg = buildConfig()
    cfg.TABLE_X   = 795;
    cfg.TABLE_Y   = 595;
    cfg.B1        = [80, 595/2, 60];
    cfg.B2        = [715, 595/2, 60];
    cfg.L0        = 123;
    cfg.L1        = 239;
    cfg.L2        = 280;
    cfg.L3        = 0;
    cfg.MAX_REACH = 480;
    cfg.MIN_REACH = 50;
    cfg.Z_MIN     = -50;
    cfg.Z_MAX     = 320;

    % Conveyor geometry
    cfg.CONV_DX = 105;
    cfg.CONV_DY = 595;
    cfg.CONV_DZ = 50;
    cfg.CONV_CX = 795/2;
    cfg.CONV_Y0 = 0;

    % Object geometry
    cfg.OBJ_DX = 90;
    cfg.OBJ_DY = 200;
    cfg.OBJ_DZ = 80;
    cfg.OBJ_CX = cfg.CONV_CX;
    cfg.OBJ_Z0 = cfg.CONV_DZ;

    % Single painting station: use the middle station from the GUI version
    cfg.STATION_Y = 297.5;

    cfg.STANDOFF  = 10;
    cfg.STRIPE_H  = 10;
    cfg.APPROACH  = 10;
    cfg.PAINT_SPD = 5.0;
    cfg.IP1       = '172.31.17.184';
    cfg.IP2       = '172.31.17.101';
    cfg.BASE_URL  = '/js?json=';
    cfg.HTTP_TO   = 0.8;
    cfg.READ_TO   = 0.3;
    cfg.MAX_RET   = 1;
    cfg.RET_DLY   = 0.05;

    % Keep the gripper slightly open during Cartesian motion.
    cfg.MOTION_GRIPPER_OPEN_OFFSET_DEG = 8;
    cfg.MOTION_GRIPPER_ANGLE_RAD = pi - deg2rad(cfg.MOTION_GRIPPER_OPEN_OFFSET_DEG);

    cfg.SETTLE_TOL     = 5.0;
    cfg.SETTLE_TIMEOUT = 1.0;
    cfg.SETTLE_POLL    = 0.25;
end

%% ================================================================
%  SECTION 2: GUI
%% ================================================================
function buildMainGUI(cfg)
    C.bg    = [0.95 0.95 0.95];
    C.panel = [1.00 1.00 1.00];
    C.acc1  = [0.20 0.50 0.90];
    C.acc2  = [0.90 0.45 0.10];
    C.grn   = [0.20 0.70 0.30];
    C.txt   = [0.15 0.15 0.15];
    C.dark  = [1.00 1.00 1.00];

    figW = 1750;
    figH = 750;
    topBarH = 28;

    fig = uifigure( ...
        'Name', 'RoArm-M2-S One-Station Painting Controller', ...
        'Position', [30 30 figW figH], ...
        'Color', C.bg);

    topBarY = figH - topBarH;
    topBar = uipanel(fig, ...
        'Position', [0 topBarY figW topBarH], ...
        'BackgroundColor', C.bg, ...
        'BorderType', 'none');

    titleW = 1180;
    uilabel(topBar, ...
        'Text', 'RoArm-M2-S  |  Dual-Arm Painting System  |  1-Station Conveyor  |  Session HMI', ...
        'Position', [0 0 titleW topBarH], ...
        'FontSize', 14, ...
        'FontWeight', 'bold', ...
        'FontColor', C.acc1, ...
        'BackgroundColor', C.bg, ...
        'HorizontalAlignment', 'center');

    statusX = titleW + 10;
    uilabel(topBar, ...
        'Text', 'Status:', ...
        'Position', [statusX 3 55 22], ...
        'FontColor', C.txt, ...
        'BackgroundColor', C.bg, ...
        'FontSize', 9, ...
        'HorizontalAlignment', 'left');
    h.progLbl = uilabel(topBar, ...
        'Text', 'Not Connected', ...
        'Position', [statusX + 58 3 figW - (statusX + 58) - 10 22], ...
        'FontColor', [0.7 0 0], ...
        'BackgroundColor', C.bg, ...
        'FontSize', 10, ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left');

    [h.e1x, h.e1y, h.e1z, h.btn1, h.lbl1, h.tel1] = ...
        makeArmPanel(fig, 'ARM 1 (Left)', [10 520 420 195], C, C.acc1, cfg, 1);
    [h.e2x, h.e2y, h.e2z, h.btn2, h.lbl2, h.tel2] = ...
        makeArmPanel(fig, 'ARM 2 (Right)', [10 320 420 195], C, C.acc2, cfg, 2);

    h = makeConfigPanel(fig, h, C, cfg, [10 10 420 305]);

    pnl3d = uipanel(fig, ...
        'Title', '3D Workspace Preview', ...
        'FontWeight', 'bold', ...
        'Position', [440 210 640 500], ...
        'BackgroundColor', C.panel, ...
        'ForegroundColor', C.txt, ...
        'BorderType', 'line');
    h.ax = uiaxes(pnl3d, 'Position', [5 5 628 490], ...
        'Color', [1 1 1], ...
        'XColor', [0.3 0.3 0.3], ...
        'YColor', [0.3 0.3 0.3], ...
        'ZColor', [0.3 0.3 0.3], ...
        'GridColor', [0.80 0.80 0.80], ...
        'FontSize', 8);
    hold(h.ax, 'on');
    grid(h.ax, 'on');
    xlabel(h.ax, 'X (mm)');
    ylabel(h.ax, 'Y (mm)');
    zlabel(h.ax, 'Z (mm)');
    title(h.ax, 'Workspace And Painting Paths', 'Color', C.txt, 'FontSize', 10);
    view(h.ax, 42, 26);
    drawWorkspace(h.ax, cfg, C);

    logPan = uipanel(fig, ...
        'Title', 'Live Event Log', ...
        'FontWeight', 'bold', ...
        'Position', [440 10 640 195], ...
        'BackgroundColor', C.panel, ...
        'ForegroundColor', C.txt, ...
        'FontSize', 9, ...
        'BorderType', 'line');
    h.log = uitextarea(logPan, ...
        'Position', [4 4 628 168], ...
        'BackgroundColor', [1 1 1], ...
        'FontColor', [0.1 0.1 0.1], ...
        'FontName', 'Courier New', ...
        'FontSize', 9, ...
        'Editable', 'off', ...
        'Value', { ...
            '[System] Ready. Click PING to test robot connections.'; ...
            'Conveyor: 105(X) x 595(Y) x 50(Z)mm - 1 Station'; ...
            'Station 1: Side faces + Top only'; ...
            'Front face and back face are disabled' ...
        });

    pnlR = uipanel(fig, ...
        'Title', 'Real-Time Telemetry', ...
        'FontWeight', 'bold', ...
        'Position', [1090 10 650 700], ...
        'BackgroundColor', C.panel, ...
        'ForegroundColor', C.txt, ...
        'BorderType', 'line');

    gw = 620;
    gh = 158;
    gx = 12;

    h.axPos1 = uiaxes(pnlR, 'Position', [gx 530 gw gh]);
    hold(h.axPos1, 'on');
    grid(h.axPos1, 'on');
    title(h.axPos1, 'ARM 1 - Cartesian Position', 'FontSize', 9);
    ylabel(h.axPos1, 'mm');
    xlabel(h.axPos1, 'Step');
    h.linePos1X = animatedline(h.axPos1, 'Color', 'r', 'LineWidth', 1.3, 'DisplayName', 'X');
    h.linePos1Y = animatedline(h.axPos1, 'Color', [0 0.7 0], 'LineWidth', 1.3, 'DisplayName', 'Y');
    h.linePos1Z = animatedline(h.axPos1, 'Color', 'b', 'LineWidth', 1.3, 'DisplayName', 'Z');
    legend(h.axPos1, 'Location', 'bestoutside', 'FontSize', 7);

    h.axTrq1 = uiaxes(pnlR, 'Position', [gx 362 gw gh]);
    hold(h.axTrq1, 'on');
    grid(h.axTrq1, 'on');
    title(h.axTrq1, 'ARM 1 - Estimated Joint Torque', 'FontSize', 9);
    ylabel(h.axTrq1, 'N*mm');
    xlabel(h.axTrq1, 'Step');
    h.lineTrq1_1 = animatedline(h.axTrq1, 'Color', 'r', 'LineWidth', 1.3, 'DisplayName', 'tau_1 Base');
    h.lineTrq1_2 = animatedline(h.axTrq1, 'Color', [0 0.7 0], 'LineWidth', 1.3, 'DisplayName', 'tau_2 Shoulder');
    h.lineTrq1_3 = animatedline(h.axTrq1, 'Color', 'b', 'LineWidth', 1.3, 'DisplayName', 'tau_3 Elbow');
    legend(h.axTrq1, 'Location', 'bestoutside', 'FontSize', 7);

    h.axPos2 = uiaxes(pnlR, 'Position', [gx 194 gw gh]);
    hold(h.axPos2, 'on');
    grid(h.axPos2, 'on');
    title(h.axPos2, 'ARM 2 - Cartesian Position', 'FontSize', 9);
    ylabel(h.axPos2, 'mm');
    xlabel(h.axPos2, 'Step');
    h.linePos2X = animatedline(h.axPos2, 'Color', 'r', 'LineWidth', 1.3, 'DisplayName', 'X');
    h.linePos2Y = animatedline(h.axPos2, 'Color', [0 0.7 0], 'LineWidth', 1.3, 'DisplayName', 'Y');
    h.linePos2Z = animatedline(h.axPos2, 'Color', 'b', 'LineWidth', 1.3, 'DisplayName', 'Z');
    legend(h.axPos2, 'Location', 'bestoutside', 'FontSize', 7);

    h.axTrq2 = uiaxes(pnlR, 'Position', [gx 26 gw gh]);
    hold(h.axTrq2, 'on');
    grid(h.axTrq2, 'on');
    title(h.axTrq2, 'ARM 2 - Estimated Joint Torque', 'FontSize', 9);
    ylabel(h.axTrq2, 'N*mm');
    xlabel(h.axTrq2, 'Step');
    h.lineTrq2_1 = animatedline(h.axTrq2, 'Color', 'r', 'LineWidth', 1.3, 'DisplayName', 'tau_1 Base');
    h.lineTrq2_2 = animatedline(h.axTrq2, 'Color', [0 0.7 0], 'LineWidth', 1.3, 'DisplayName', 'tau_2 Shoulder');
    h.lineTrq2_3 = animatedline(h.axTrq2, 'Color', 'b', 'LineWidth', 1.3, 'DisplayName', 'tau_3 Elbow');
    legend(h.axTrq2, 'Location', 'bestoutside', 'FontSize', 7);

    h.stopFlag = false;
    h.stepCount1 = 0;
    h.stepCount2 = 0;

    fig.UserData = h;
    h.btn1.ButtonPushedFcn = @(~,~) cbMove(fig, 1, cfg);
    h.btn2.ButtonPushedFcn = @(~,~) cbMove(fig, 2, cfg);
    rewireConfigButtons(fig, cfg);
    fig.UserData = h;
end

function [ex, ey, ez, btn, lblIK, lblTel] = makeArmPanel(fig, titleStr, pos, C, acc, cfg, armIdx)
    pan = uipanel(fig, ...
        'Title', titleStr, ...
        'Position', pos, ...
        'BackgroundColor', C.panel, ...
        'ForegroundColor', acc, ...
        'FontSize', 10, ...
        'FontWeight', 'bold', ...
        'BorderType', 'line');

    row = @(n) 148 - n * 27;
    lw = 95;
    ew = 110;

    defaultX = cfg.OBJ_CX;
    if armIdx == 1
        defaultY = cfg.STATION_Y - 40;
    else
        defaultY = cfg.STATION_Y + 40;
    end
    defaultZ = cfg.OBJ_Z0 + 50;

    uilabel(pan, 'Text', 'X global (mm):', ...
        'Position', [8 row(0) lw 20], ...
        'FontColor', C.txt, ...
        'BackgroundColor', C.panel, ...
        'FontSize', 9);
    ex = uieditfield(pan, 'numeric', ...
        'Position', [lw+8 row(0) ew 22], ...
        'Value', defaultX, ...
        'Limits', [-100 cfg.TABLE_X+100], ...
        'FontSize', 9);

    uilabel(pan, 'Text', 'Y global (mm):', ...
        'Position', [8 row(1) lw 20], ...
        'FontColor', C.txt, ...
        'BackgroundColor', C.panel, ...
        'FontSize', 9);
    ey = uieditfield(pan, 'numeric', ...
        'Position', [lw+8 row(1) ew 22], ...
        'Value', defaultY, ...
        'Limits', [-100 cfg.TABLE_Y+100], ...
        'FontSize', 9);

    uilabel(pan, 'Text', 'Z height (mm):', ...
        'Position', [8 row(2) lw 20], ...
        'FontColor', C.txt, ...
        'BackgroundColor', C.panel, ...
        'FontSize', 9);
    ez = uieditfield(pan, 'numeric', ...
        'Position', [lw+8 row(2) ew 22], ...
        'Value', defaultZ, ...
        'Limits', [0 350], ...
        'FontSize', 9);

    btn = uibutton(pan, 'Text', 'Move', ...
        'Position', [8 row(3) 210 26], ...
        'BackgroundColor', acc, ...
        'FontColor', 'w', ...
        'FontSize', 9, ...
        'FontWeight', 'bold');

    lblIK = uilabel(pan, 'Text', 'Ready', ...
        'Position', [8 row(4) 330 18], ...
        'FontColor', [0.0 0.6 0.0], ...
        'BackgroundColor', C.panel, ...
        'FontSize', 8, ...
        'WordWrap', 'on');

    uilabel(pan, 'Text', 'Live:', ...
        'Position', [8 row(5) 40 16], ...
        'FontColor', [0.4 0.4 0.5], ...
        'BackgroundColor', C.panel, ...
        'FontSize', 8);
    lblTel = uilabel(pan, 'Text', '-', ...
        'Position', [50 row(5) 288 16], ...
        'FontColor', [0.1 0.4 0.7], ...
        'BackgroundColor', C.panel, ...
        'FontSize', 8);
end

function h = makeConfigPanel(fig, h, C, cfg, pos)
    pan = uipanel(fig, ...
        'Title', 'Settings And Control', ...
        'Position', pos, ...
        'BackgroundColor', C.panel, ...
        'ForegroundColor', C.txt, ...
        'FontSize', 10, ...
        'FontWeight', 'bold', ...
        'BorderType', 'line');

    lw = 120;
    ew = 80;

    uilabel(pan, 'Text', 'Stripe (mm):', ...
        'Position', [8 250 lw 20], ...
        'FontColor', C.txt, ...
        'BackgroundColor', C.panel, ...
        'FontSize', 9);
    h.eStripe = uieditfield(pan, 'numeric', ...
        'Position', [lw+5 250 ew 22], ...
        'Value', cfg.STRIPE_H, ...
        'Limits', [5 50]);

    uilabel(pan, 'Text', 'Standoff (mm):', ...
        'Position', [8 224 lw 20], ...
        'FontColor', C.txt, ...
        'BackgroundColor', C.panel, ...
        'FontSize', 9);
    h.eStandoff = uieditfield(pan, 'numeric', ...
        'Position', [lw+5 224 ew 22], ...
        'Value', cfg.STANDOFF, ...
        'Limits', [10 100]);

    uilabel(pan, 'Text', 'Speed:', ...
        'Position', [8 198 lw 20], ...
        'FontColor', C.txt, ...
        'BackgroundColor', C.panel, ...
        'FontSize', 9);
    h.eSpeed = uieditfield(pan, 'numeric', ...
        'Position', [lw+5 198 ew 22], ...
        'Value', cfg.PAINT_SPD, ...
        'Limits', [1 20]);

    uilabel(pan, 'Text', 'ARM 1 IP:', ...
        'Position', [8 172 lw 20], ...
        'FontColor', C.txt, ...
        'BackgroundColor', C.panel, ...
        'FontSize', 9);
    h.eIP1 = uieditfield(pan, 'text', ...
        'Position', [lw+5 172 160 22], ...
        'Value', cfg.IP1);

    uilabel(pan, 'Text', 'ARM 2 IP:', ...
        'Position', [8 146 lw 20], ...
        'FontColor', C.txt, ...
        'BackgroundColor', C.panel, ...
        'FontSize', 9);
    h.eIP2 = uieditfield(pan, 'text', ...
        'Position', [lw+5 146 160 22], ...
        'Value', cfg.IP2);

    uibutton(pan, 'Text', 'Ping', ...
        'Position', [8 112 120 28], ...
        'BackgroundColor', [0.2 0.5 0.9], ...
        'FontColor', 'w', ...
        'FontSize', 9, ...
        'FontWeight', 'bold', ...
        'Tag', 'btnPing');
    uibutton(pan, 'Text', 'HOME', ...
        'Position', [138 112 120 28], ...
        'BackgroundColor', [0.5 0.3 0.7], ...
        'FontColor', 'w', ...
        'FontSize', 9, ...
        'FontWeight', 'bold', ...
        'Tag', 'btnHome');
    uibutton(pan, 'Text', 'Telemetry', ...
        'Position', [268 112 130 28], ...
        'BackgroundColor', [0.2 0.7 0.3], ...
        'FontColor', 'w', ...
        'FontSize', 9, ...
        'FontWeight', 'bold', ...
        'Tag', 'btnTel');

    uibutton(pan, 'Text', 'START PAINTING', ...
        'Position', [8 58 390 42], ...
        'BackgroundColor', [0.2 0.7 0.3], ...
        'FontColor', 'w', ...
        'FontSize', 13, ...
        'FontWeight', 'bold', ...
        'Tag', 'btnStart');

    uibutton(pan, 'Text', 'STOP', ...
        'Position', [8 8 390 42], ...
        'BackgroundColor', [0.85 0.1 0.1], ...
        'FontColor', 'w', ...
        'FontSize', 14, ...
        'FontWeight', 'bold', ...
        'Tag', 'btnStop');
end

function rewireConfigButtons(fig, cfg)
    btn = findobj(fig, 'Tag', 'btnPing');
    btn.ButtonPushedFcn = @(~,~) cbPing(fig, cfg);
    btn = findobj(fig, 'Tag', 'btnHome');
    btn.ButtonPushedFcn = @(~,~) cbHome(fig, cfg);
    btn = findobj(fig, 'Tag', 'btnTel');
    btn.ButtonPushedFcn = @(~,~) cbTel(fig, cfg);
    btn = findobj(fig, 'Tag', 'btnStart');
    btn.ButtonPushedFcn = @(~,~) cbStart(fig, cfg);
    btn = findobj(fig, 'Tag', 'btnStop');
    btn.ButtonPushedFcn = @(~,~) cbStop(fig, cfg);
end

%% ================================================================
%  SECTION 3: COORDINATE TRANSFORM AND REACHABILITY
%% ================================================================
function [lx, ly, lz] = globalToLocal(gx, gy, gz, armIdx, cfg)
    if armIdx == 1
        lx = gx - cfg.B1(1);
        ly = gy - cfg.B1(2);
    else
        lx = cfg.B2(1) - gx;
        ly = cfg.B2(2) - gy;
    end
    lz = gz - cfg.B1(3);
end

function [ok, msg] = checkReach(lx, ly, lz, cfg)
    r = sqrt(lx^2 + ly^2);
    zR = lz - cfg.L0;
    d3 = sqrt(r^2 + zR^2);

    if lx < 0
        ok = false;
        msg = sprintf('BEHIND BASE: lx=%.1f (must be >= 0)', lx);
        return
    end
    if d3 > (cfg.L1 + cfg.L2 + cfg.L3)
        ok = false;
        msg = sprintf('OUT OF REACH: %.1f mm (max %.0f)', d3, cfg.L1 + cfg.L2 + cfg.L3);
        return
    end
    if d3 > cfg.MAX_REACH
        ok = false;
        msg = sprintf('BEYOND MAX_REACH: %.1f mm (max %.0f)', d3, cfg.MAX_REACH);
        return
    end
    if r < cfg.MIN_REACH && lz < (cfg.L0 + 20)
        ok = false;
        msg = sprintf('TOO CLOSE: r=%.1f mm (min %d)', r, cfg.MIN_REACH);
        return
    end
    if lz < cfg.Z_MIN || lz > cfg.Z_MAX
        ok = false;
        msg = sprintf('Z=%.1f out of range [%d-%d]', lz, cfg.Z_MIN, cfg.Z_MAX);
        return
    end

    ok = true;
    msg = sprintf('OK  r=%.1f  d3D=%.1f  lx=%.0f ly=%.0f lz=%.0f', r, d3, lx, ly, lz);
end

function [theta1, theta2, theta3] = computeJointAngles(lx, ly, lz, cfg)
    theta1 = atan2d(ly, lx);

    r  = sqrt(lx^2 + ly^2);
    zs = lz - cfg.L0;
    d  = sqrt(r^2 + zs^2);

    L1 = cfg.L1;
    L2 = cfg.L2;

    cosQ3 = (d^2 - L1^2 - L2^2) / (2 * L1 * L2);
    cosQ3 = max(-1, min(1, cosQ3));
    theta3 = acosd(cosQ3);

    alpha = atan2d(zs, r);
    beta  = atan2d(L2 * sind(theta3), L1 + L2 * cosd(theta3));
    theta2 = alpha + beta;
end

function torques = computeStaticTorques(theta2, theta3, cfg)
    g = 9.81;

    m1 = 0.120;
    m2 = 0.085;
    m3 = 0.040;

    L1 = cfg.L1;
    L2 = cfg.L2;

    phi = theta2 - theta3;

    tau3 = (m2/2 + m3) * g * L2 * cosd(phi);
    tau2 = (m1/2) * g * L1 * cosd(theta2) + ...
           (m2 + m3) * g * L1 * cosd(theta2) + ...
           (m2/2 + m3) * g * L2 * cosd(phi);

    rHoriz = abs(L1 * cosd(theta2) + L2 * cosd(phi));
    tau1 = 0.008 * (m1 + m2 + m3) * g * rHoriz;

    torques = [tau1, tau2, tau3];
end

%% ================================================================
%  SECTION 4: HTTP COMMUNICATION
%% ================================================================
function [resp, success] = httpSend(ip, baseURL, jsonCmd, timeout, maxRet, retDly)
    url = ['http://' char(ip) char(baseURL) urlencode(jsonCmd)];
    resp = '';
    success = false;

    for k = 1:maxRet
        try
            resp = webread(url, weboptions('Timeout', timeout));
            success = true;
            return
        catch
            if k < maxRet
                pause(retDly);
            end
        end
    end
end

function data = httpReadStatus(ip, timeout)
    cmd = struct('T', 105);
    url = ['http://' char(ip) '/js?json=' urlencode(jsonencode(cmd))];
    data = [];

    try
        resp = webread(url, weboptions('Timeout', timeout));
        s = strfind(resp, '{');
        e = strfind(resp, '}');
        if ~isempty(s) && ~isempty(e)
            data = jsondecode(resp(s(1):e(end)));
        else
            data = jsondecode(resp);
        end
    catch
    end
end

function [x, y, z] = parseTel(data)
    x = getField(data, {'x', 'X', 'px', 'posx'});
    y = getField(data, {'y', 'Y', 'py', 'posy'});
    z = getField(data, {'z', 'Z', 'pz', 'posz'});
end

function v = getField(s, names)
    v = 0;
    if isempty(s) || ~isstruct(s)
        return
    end

    for k = 1:numel(names)
        if isfield(s, names{k})
            try
                v = double(s.(names{k}));
                return
            catch
            end
        end
    end
end

function [connected, arm1ok, arm2ok] = validateConnection(ip1, ip2, timeout)
    data1 = httpReadStatus(ip1, timeout);
    data2 = httpReadStatus(ip2, timeout);
    arm1ok = ~isempty(data1);
    arm2ok = ~isempty(data2);
    connected = arm1ok && arm2ok;
end

function tf = stopRequested(fig)
    tf = false;

    if isempty(fig) || ~isvalid(fig)
        return
    end

    h = fig.UserData;
    tf = isstruct(h) && isfield(h, 'stopFlag') && logical(h.stopFlag);
end

function arrived = waitForArrival(ip, targetLocal, cfg, fig)
    tStart = tic;
    arrived = false;

    while toc(tStart) < cfg.SETTLE_TIMEOUT
        drawnow limitrate;
        if stopRequested(fig)
            return
        end

        pause(cfg.SETTLE_POLL);
        drawnow limitrate;
        if stopRequested(fig)
            return
        end

        data = httpReadStatus(ip, cfg.READ_TO);
        if stopRequested(fig)
            return
        end

        if isempty(data)
            continue
        end

        [rx, ry, rz] = parseTel(data);
        err = sqrt((rx - targetLocal(1))^2 + ...
                   (ry - targetLocal(2))^2 + ...
                   (rz - targetLocal(3))^2);
        if err <= cfg.SETTLE_TOL
            arrived = true;
            return
        end
    end
end

%% ================================================================
%  SECTION 5: PATH GENERATION
%  One station only: side faces and top face
%% ================================================================
function paths = generatePaintingPaths(cfg)
    so  = cfg.STANDOFF;
    sH  = cfg.STRIPE_H;
    app = cfg.APPROACH;
    cx  = cfg.OBJ_CX;
    cy  = cfg.STATION_Y;
    dx  = cfg.OBJ_DX / 2;
    dy  = cfg.OBJ_DY / 2;
    dz  = cfg.OBJ_DZ;
    z0  = cfg.OBJ_Z0;

    zBot = z0 + 8;
    zTop = z0 + dz - 8;
    nStrZ = max(1, floor((zTop - zBot) / sH) + 1);
    zLevels = linspace(zBot, zTop, nStrZ);

    xArm1Side = cx - dx - so;
    xArm2Side = cx + dx + so;

    pathArm1Side = genFacePathX(xArm1Side, cy, dy, zLevels, app);
    pathArm2Side = genFacePathX(xArm2Side, cy, dy, zLevels, app);

    zTopFace = z0 + dz + so;
    xLeft  = cx - dx + 8;
    xRight = cx + dx - 8;
    xMid   = cx;

    nStrX1 = max(1, floor((xMid - xLeft) / sH) + 1);
    xLevels1 = linspace(xLeft, xMid, nStrX1);
    pathArm1Top = genTopPath(xLevels1, cy, dy, zTopFace, app);

    nStrX2 = max(1, floor((xRight - xMid) / sH) + 1);
    xLevels2 = linspace(xMid, xRight, nStrX2);
    pathArm2Top = genTopPath(xLevels2, cy, dy, zTopFace, app);

    paths = struct();
    paths.arm1 = [pathArm1Side; pathArm1Top];
    paths.arm2 = [pathArm2Side; pathArm2Top];
end

function path = genFacePathX(xNozzle, cy, dy, zLevels, app)
    path = zeros(0, 3);

    for i = 1:numel(zLevels)
        z = zLevels(i);
        if mod(i, 2) == 1
            path = [path; ...
                xNozzle, cy-dy-app, z; ...
                xNozzle, cy-dy,     z; ...
                xNozzle, cy+dy,     z; ...
                xNozzle, cy+dy+app, z]; %#ok<AGROW>
        else
            path = [path; ...
                xNozzle, cy+dy+app, z; ...
                xNozzle, cy+dy,     z; ...
                xNozzle, cy-dy,     z; ...
                xNozzle, cy-dy-app, z]; %#ok<AGROW>
        end
    end
end

function path = genTopPath(xLevels, cy, dy, zTop, app)
    path = zeros(0, 3);

    for i = 1:numel(xLevels)
        x = xLevels(i);
        if mod(i, 2) == 1
            path = [path; ...
                x, cy-dy-app, zTop; ...
                x, cy-dy,     zTop; ...
                x, cy+dy,     zTop; ...
                x, cy+dy+app, zTop]; %#ok<AGROW>
        else
            path = [path; ...
                x, cy+dy+app, zTop; ...
                x, cy+dy,     zTop; ...
                x, cy-dy,     zTop; ...
                x, cy-dy-app, zTop]; %#ok<AGROW>
        end
    end
end

function [validCount, totalCount, issues] = validatePaintingPaths(paths, cfg)
    fNames = fieldnames(paths);
    totalCount = 0;
    validCount = 0;
    issues = {};

    for f = 1:numel(fNames)
        name = fNames{f};
        pts = paths.(name);
        n = size(pts, 1);
        if n == 0
            continue
        end

        if contains(name, 'arm1')
            armIdx = 1;
        else
            armIdx = 2;
        end

        faceValid = 0;
        for i = 1:n
            [lx, ly, lz] = globalToLocal(pts(i,1), pts(i,2), pts(i,3), armIdx, cfg);
            [ok, ~] = checkReach(lx, ly, lz, cfg);
            if ok
                faceValid = faceValid + 1;
            end
        end

        totalCount = totalCount + n;
        validCount = validCount + faceValid;
        if faceValid < n
            issues{end+1} = sprintf('%s: %d/%d reachable', name, faceValid, n); %#ok<AGROW>
        end
    end
end

%% ================================================================
%  SECTION 6: PATH EXECUTION
%% ================================================================
function success = executePath(ip, pathG, armIdx, cfg, fig, label)
    n = size(pathG, 1);
    logMsg(fig, sprintf('[%s] Starting %d waypoints', label, n));
    success = false;

    if n == 0
        logMsg(fig, sprintf('[%s] No waypoints to execute', label));
        success = true;
        return
    end

    skipped = 0;
    failed = 0;

    for i = 1:n
        drawnow limitrate;
        h = fig.UserData;
        if stopRequested(fig)
            logMsg(fig, sprintf('[%s] STOPPED by user at waypoint %d', label, i));
            return
        end

        gx = pathG(i,1);
        gy = pathG(i,2);
        gz = pathG(i,3);

        [lx, ly, lz] = globalToLocal(gx, gy, gz, armIdx, cfg);
        [ok, wMsg] = checkReach(lx, ly, lz, cfg);
        if ~ok
            logMsg(fig, sprintf('  [skip %d/%d] %s', i, n, wMsg));
            skipped = skipped + 1;
            continue
        end

        spd = h.eSpeed.Value;
        cmd = sprintf('{"T":104,"x":%d,"y":%d,"z":%d,"t":%.4f,"spd":%.2f}', ...
            round(lx), round(ly), round(lz), cfg.MOTION_GRIPPER_ANGLE_RAD, spd);
        if i == 1
            logMsg(fig, sprintf('[%s] First motion cmd: %s', label, cmd));
        end

        [~, motionOk] = httpSend(ip, cfg.BASE_URL, cmd, cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);
        if ~motionOk
            logMsg(fig, sprintf('  [FAIL %d/%d] Robot not responding!', i, n));
            failed = failed + 1;
            if failed >= 5
                logMsg(fig, sprintf('[%s] Too many failures, aborting path', label));
                return
            end
            continue
        end

        if stopRequested(fig)
            logMsg(fig, sprintf('[%s] STOPPED by user at waypoint %d', label, i));
            return
        end

        arrived = waitForArrival(ip, [lx, ly, lz], cfg, fig);
        if stopRequested(fig)
            logMsg(fig, sprintf('[%s] STOPPED by user at waypoint %d', label, i));
            return
        end

        if ~arrived
            logMsg(fig, sprintf('  [TIMEOUT %d/%d] Robot did not reach target in time', i, n));
        end

        if isfield(h, 'ax') && isvalid(h.ax)
            if armIdx == 1
                plot3(h.ax, gx, gy, gz, '.', 'Color', [0.15 0.55 1.0], 'MarkerSize', 5);
            else
                plot3(h.ax, gx, gy, gz, '.', 'Color', [1.0 0.55 0.15], 'MarkerSize', 5);
            end
        end

        [~, th2, th3] = computeJointAngles(lx, ly, lz, cfg);
        trq = computeStaticTorques(th2, th3, cfg);
        if armIdx == 1
            h.stepCount1 = h.stepCount1 + 1;
            step = h.stepCount1;
            addpoints(h.linePos1X, step, gx);
            addpoints(h.linePos1Y, step, gy);
            addpoints(h.linePos1Z, step, gz);
            addpoints(h.lineTrq1_1, step, trq(1));
            addpoints(h.lineTrq1_2, step, trq(2));
            addpoints(h.lineTrq1_3, step, trq(3));
        else
            h.stepCount2 = h.stepCount2 + 1;
            step = h.stepCount2;
            addpoints(h.linePos2X, step, gx);
            addpoints(h.linePos2Y, step, gy);
            addpoints(h.linePos2Z, step, gz);
            addpoints(h.lineTrq2_1, step, trq(1));
            addpoints(h.lineTrq2_2, step, trq(2));
            addpoints(h.lineTrq2_3, step, trq(3));
        end

        pct = round(i / n * 100);
        h.progLbl.Text = sprintf('[%s] %d%%  (%d/%d)', label, pct, i, n);
        fig.UserData = h;
        drawnow limitrate;
    end

    logMsg(fig, sprintf('[%s] Complete - %d sent, %d skipped, %d failed', ...
        label, n - skipped - failed, skipped, failed));
    success = true;
end

%% ================================================================
%  SECTION 7: WORKSPACE VISUALIZATION
%% ================================================================
function drawWorkspace(ax, cfg, C)
    tx = [0 cfg.TABLE_X cfg.TABLE_X 0];
    ty = [0 0 cfg.TABLE_Y cfg.TABLE_Y];
    fill3(ax, tx, ty, zeros(1,4), [0.85 0.85 0.85], ...
        'FaceAlpha', 0.35, 'EdgeColor', 'none');
    plot3(ax, [tx tx(1)], [ty ty(1)], zeros(1,5), ...
        'Color', [0.55 0.55 0.60], 'LineWidth', 1.1);

    convX0 = cfg.CONV_CX - cfg.CONV_DX / 2;
    drawBox(ax, convX0, cfg.CONV_Y0, 0, ...
        cfg.CONV_DX, cfg.CONV_DY, cfg.CONV_DZ, [0.50 0.50 0.55], 0.35);
    text(ax, cfg.CONV_CX, cfg.CONV_DY / 2, cfg.CONV_DZ + 5, ...
        'CONVEYOR', 'Color', [0.35 0.35 0.40], ...
        'FontSize', 7, 'HorizontalAlignment', 'center');

    objX0 = cfg.OBJ_CX - cfg.OBJ_DX / 2;
    objY0 = cfg.STATION_Y - cfg.OBJ_DY / 2;
    drawBox(ax, objX0, objY0, cfg.OBJ_Z0, ...
        cfg.OBJ_DX, cfg.OBJ_DY, cfg.OBJ_DZ, [0.90 0.65 0.1], 0.45);
    text(ax, cfg.OBJ_CX, cfg.STATION_Y, cfg.OBJ_Z0 + cfg.OBJ_DZ + 15, ...
        'S1: Side + Top', ...
        'Color', [0.90 0.65 0.1], ...
        'FontSize', 8, ...
        'HorizontalAlignment', 'center', ...
        'FontWeight', 'bold');

    bz = cfg.B1(3);
    platW = 60;
    platH = 60;
    drawBox(ax, cfg.B1(1)-platW/2, cfg.B1(2)-platW/2, 0, ...
        platW, platW, platH, [0.60 0.60 0.65], 0.60);
    drawBox(ax, cfg.B2(1)-platW/2, cfg.B2(2)-platW/2, 0, ...
        platW, platW, platH, [0.60 0.60 0.65], 0.60);

    drawRobotArm(ax, cfg.B1(1), cfg.B1(2), bz, C.acc1, 0);
    drawRobotArm(ax, cfg.B2(1), cfg.B2(2), bz, C.acc2, 180);

    text(ax, cfg.B1(1)+12, cfg.B1(2)+25, bz+80, 'ARM 1', ...
        'Color', C.acc1, 'FontSize', 9, 'FontWeight', 'bold');
    text(ax, cfg.B2(1)+12, cfg.B2(2)-35, bz+80, 'ARM 2', ...
        'Color', C.acc2, 'FontSize', 9, 'FontWeight', 'bold');

    th = linspace(0, 2*pi, 64);
    r = cfg.MAX_REACH;
    plot3(ax, cfg.B1(1)+r*cos(th), cfg.B1(2)+r*sin(th), bz*ones(1,64), ...
        '--', 'Color', [C.acc1 0.40], 'LineWidth', 0.7);
    plot3(ax, cfg.B2(1)+r*cos(th), cfg.B2(2)+r*sin(th), bz*ones(1,64), ...
        '--', 'Color', [C.acc2 0.40], 'LineWidth', 0.7);

    plot3(ax, [cfg.CONV_CX-60 cfg.CONV_CX+60], ...
        [cfg.STATION_Y cfg.STATION_Y], [1 1], ...
        '-', 'Color', [0.8 0.2 0.2], 'LineWidth', 1.5);

    axis(ax, 'equal');
    xlim(ax, [-70 cfg.TABLE_X+70]);
    ylim(ax, [-70 cfg.TABLE_Y+70]);
    zlim(ax, [0 280]);
end

function drawRobotArm(ax, x0, y0, z0, color, rotation_deg)
    theta = deg2rad(rotation_deg);

    [xc, yc, zc] = cylinder(25, 20);
    zc = zc * 54;
    for i = 1:size(xc, 1)
        for j = 1:size(xc, 2)
            xr = xc(i,j) * cos(theta) - yc(i,j) * sin(theta);
            yr = xc(i,j) * sin(theta) + yc(i,j) * cos(theta);
            xc(i,j) = xr + x0;
            yc(i,j) = yr + y0;
            zc(i,j) = zc(i,j) + z0;
        end
    end
    surf(ax, xc, yc, zc, ...
        'FaceColor', color, 'FaceAlpha', 0.7, 'EdgeColor', 'none');

    l1_start = [x0, y0, z0 + 54];
    l1_end = [x0 + 30*cos(theta), y0 + 30*sin(theta), z0 + 100];
    plot3(ax, [l1_start(1) l1_end(1)], [l1_start(2) l1_end(2)], [l1_start(3) l1_end(3)], ...
        'LineWidth', 8, 'Color', color);

    l2_end = [l1_end(1) + 70*cos(theta), l1_end(2) + 70*sin(theta), l1_end(3) + 20];
    plot3(ax, [l1_end(1) l2_end(1)], [l1_end(2) l2_end(2)], [l1_end(3) l2_end(3)], ...
        'LineWidth', 7, 'Color', color);

    l3_end = [l2_end(1) + 50*cos(theta), l2_end(2) + 50*sin(theta), l2_end(3) - 10];
    plot3(ax, [l2_end(1) l3_end(1)], [l2_end(2) l3_end(2)], [l2_end(3) l3_end(3)], ...
        'LineWidth', 5, 'Color', color);

    [xg, yg, zg] = sphere(10);
    xg = xg * 12 + l3_end(1);
    yg = yg * 12 + l3_end(2);
    zg = zg * 12 + l3_end(3);
    surf(ax, xg, yg, zg, ...
        'FaceColor', [0.9 0.9 0.9], 'FaceAlpha', 0.8, 'EdgeColor', 'none');

    plot3(ax, l1_start(1), l1_start(2), l1_start(3), 'o', ...
        'MarkerSize', 8, 'MarkerFaceColor', [0.3 0.3 0.3], 'Color', 'k');
    plot3(ax, l1_end(1), l1_end(2), l1_end(3), 'o', ...
        'MarkerSize', 7, 'MarkerFaceColor', [0.3 0.3 0.3], 'Color', 'k');
    plot3(ax, l2_end(1), l2_end(2), l2_end(3), 'o', ...
        'MarkerSize', 6, 'MarkerFaceColor', [0.3 0.3 0.3], 'Color', 'k');
end

function drawBox(ax, x0, y0, z0, dx, dy, dz, col, alphaVal)
    V = [x0    y0    z0;
         x0+dx y0    z0;
         x0+dx y0+dy z0;
         x0    y0+dy z0;
         x0    y0    z0+dz;
         x0+dx y0    z0+dz;
         x0+dx y0+dy z0+dz;
         x0    y0+dy z0+dz];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch(ax, 'Vertices', V, 'Faces', F, ...
        'FaceColor', col, 'FaceAlpha', alphaVal, ...
        'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 0.65);
end

%% ================================================================
%  SECTION 8: CALLBACKS
%% ================================================================
function cbMove(fig, armIdx, cfg)
    h = fig.UserData;

    if armIdx == 1
        gx = h.e1x.Value;
        gy = h.e1y.Value;
        gz = h.e1z.Value;
        ip = h.eIP1.Value;
        lblIK = h.lbl1;
        lblTel = h.tel1;
    else
        gx = h.e2x.Value;
        gy = h.e2y.Value;
        gz = h.e2z.Value;
        ip = h.eIP2.Value;
        lblIK = h.lbl2;
        lblTel = h.tel2;
    end

    logMsg(fig, sprintf('[ARM%d] Move -> X=%.1f  Y=%.1f  Z=%.1f', armIdx, gx, gy, gz));

    [lx, ly, lz] = globalToLocal(gx, gy, gz, armIdx, cfg);
    [ok, wMsg] = checkReach(lx, ly, lz, cfg);
    lblIK.Text = wMsg;

    logMsg(fig, sprintf('  Local: lx=%.1f  ly=%.1f  lz=%.1f', lx, ly, lz));

    if ~ok
        lblIK.FontColor = [0.7 0.0 0.0];
        logMsg(fig, ['  X ' wMsg]);
        return
    end

    lblIK.FontColor = [0.0 0.6 0.0];
    spd = h.eSpeed.Value;
    cmd = sprintf('{"T":104,"x":%d,"y":%d,"z":%d,"t":%.4f,"spd":%.2f}', ...
        round(lx), round(ly), round(lz), cfg.MOTION_GRIPPER_ANGLE_RAD, spd);
    logMsg(fig, sprintf('[ARM%d] Motion cmd: %s', armIdx, cmd));

    [resp, success] = httpSend(ip, cfg.BASE_URL, cmd, cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);
    if ~success
        logMsg(fig, '  X Robot not responding!');
        lblIK.Text = 'NOT CONNECTED';
        lblIK.FontColor = [0.7 0.0 0.0];
        return
    end

    logMsg(fig, sprintf('  Sent. Response: %s', strtrim(resp)));

    pause(0.5);
    data = httpReadStatus(ip, cfg.READ_TO);
    if ~isempty(data)
        [rx, ry, rz] = parseTel(data);
        lblTel.Text = sprintf('X=%.1f  Y=%.1f  Z=%.1f', rx, ry, rz);
    end
end

function cbPing(fig, ~)
    h = fig.UserData;
    ips = {h.eIP1.Value, h.eIP2.Value};
    arm1ok = false;
    arm2ok = false;

    for k = 1:2
        data = httpReadStatus(ips{k}, 1.5);
        if ~isempty(data)
            [rx, ry, rz] = parseTel(data);
            logMsg(fig, sprintf('[ARM%d] OK ONLINE  X=%.1f  Y=%.1f  Z=%.1f', k, rx, ry, rz));
            if k == 1
                arm1ok = true;
            else
                arm2ok = true;
            end
        else
            logMsg(fig, sprintf('[ARM%d] X NO RESPONSE  (%s)', k, ips{k}));
        end
    end

    if arm1ok && arm2ok
        h.progLbl.Text = 'OK Both Arms Connected';
        h.progLbl.FontColor = [0.0 0.6 0.0];
    elseif arm1ok || arm2ok
        h.progLbl.Text = 'Partial Connection';
        h.progLbl.FontColor = [0.8 0.5 0.0];
    else
        h.progLbl.Text = 'Not Connected';
        h.progLbl.FontColor = [0.7 0.0 0.0];
    end
    fig.UserData = h;
end

function success = cbHome(fig, cfg)
    h = fig.UserData;
    success = true;

    [~, arm1home] = httpSend(h.eIP1.Value, cfg.BASE_URL, '{"T":100}', ...
        cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);
    [~, arm2home] = httpSend(h.eIP2.Value, cfg.BASE_URL, '{"T":100}', ...
        cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);

    logMsg(fig, '[System] HOME command sent to both arms');
    if ~arm1home
        logMsg(fig, '[ARM1 Home] No HTTP ack for HOME; continuing');
    end
    if ~arm2home
        logMsg(fig, '[ARM2 Home] No HTTP ack for HOME; continuing');
    end
end

function success = homeOneArm(ip, cfg, fig, label)
    [~, homeSuccess] = httpSend(ip, cfg.BASE_URL, '{"T":100}', ...
        cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);

    if ~homeSuccess
        logMsg(fig, sprintf('[%s] No HTTP ack for HOME; continuing', label));
    else
        logMsg(fig, sprintf('[%s] HOME command sent', label));
    end

    success = true;
end

function cbTel(fig, cfg)
    h = fig.UserData;

    for k = 1:2
        if k == 1
            ip = h.eIP1.Value;
            lbl = h.tel1;
        else
            ip = h.eIP2.Value;
            lbl = h.tel2;
        end

        data = httpReadStatus(ip, cfg.READ_TO);
        if ~isempty(data)
            [rx, ry, rz] = parseTel(data);
            lbl.Text = sprintf('X=%.1f  Y=%.1f  Z=%.1f', rx, ry, rz);
            logMsg(fig, sprintf('[ARM%d] X=%.1f  Y=%.1f  Z=%.1f', k, rx, ry, rz));
        else
            logMsg(fig, sprintf('[ARM%d] Read failed', k));
        end
    end
end

function cbStart(fig, cfg)
    h = fig.UserData;
    ip1 = h.eIP1.Value;
    ip2 = h.eIP2.Value;

    h.stopFlag = false;
    h.stepCount1 = 0;
    h.stepCount2 = 0;
    clearpoints(h.linePos1X);
    clearpoints(h.linePos1Y);
    clearpoints(h.linePos1Z);
    clearpoints(h.lineTrq1_1);
    clearpoints(h.lineTrq1_2);
    clearpoints(h.lineTrq1_3);
    clearpoints(h.linePos2X);
    clearpoints(h.linePos2Y);
    clearpoints(h.linePos2Z);
    clearpoints(h.lineTrq2_1);
    clearpoints(h.lineTrq2_2);
    clearpoints(h.lineTrq2_3);
    fig.UserData = h;

    logMsg(fig, '========================================');
    logMsg(fig, '  VALIDATING ROBOT CONNECTION...');

    [connected, arm1ok, arm2ok] = validateConnection(ip1, ip2, 2.0);
    if ~connected
        logMsg(fig, 'X CONNECTION FAILED:');
        if ~arm1ok
            logMsg(fig, sprintf('  ARM1 (%s) not responding', ip1));
        end
        if ~arm2ok
            logMsg(fig, sprintf('  ARM2 (%s) not responding', ip2));
        end
        logMsg(fig, 'PAINTING ABORTED - Robots not connected!');
        logMsg(fig, '========================================');

        uialert(fig, ...
            sprintf('Cannot start painting:\n\nARM1: %s\nARM2: %s\n\nClick "Ping" to test connection.', ...
            ternary(arm1ok, 'OK Connected', 'Offline'), ...
            ternary(arm2ok, 'OK Connected', 'Offline')), ...
            'Connection Required', 'Icon', 'error');
        return
    end

    logMsg(fig, 'OK Both robots connected');

    cfg.STRIPE_H  = h.eStripe.Value;
    cfg.STANDOFF  = h.eStandoff.Value;
    cfg.PAINT_SPD = h.eSpeed.Value;

    logMsg(fig, sprintf('  Stripe=%.0fmm  Standoff=%.0fmm  Speed=%.1f', ...
        cfg.STRIPE_H, cfg.STANDOFF, cfg.PAINT_SPD));

    paths = generatePaintingPaths(cfg);
    [validCount, totalCount, issues] = validatePaintingPaths(paths, cfg);
    logMsg(fig, sprintf('  Path validation: %d/%d waypoints reachable', validCount, totalCount));
    for k = 1:numel(issues)
        logMsg(fig, sprintf('  WARNING: %s', issues{k}));
    end

    if validCount == 0
        logMsg(fig, 'ABORT: No reachable waypoints! Check object placement.');
        uialert(fig, ...
            'No waypoints are reachable. The object may be too far from the robot bases.', ...
            'Path Error', 'Icon', 'error');
        return
    end

    logMsg(fig, '  1-STATION PAINTING SEQUENCE');
    logMsg(fig, '  Station 1: Side faces + Top only');
    logMsg(fig, '  Front and back faces are disabled in this version');
    logMsg(fig, '========================================');

    cbHome(fig, cfg);
    pause(2.0);

    if stopRequested(fig)
        logMsg(fig, 'STOPPED by user');
        return
    end

    logMsg(fig, '--- STATION 1: Side Faces + Top ---');
    h.progLbl.Text = 'Station 1: Sides + Top';
    fig.UserData = h;

    executePath(ip1, paths.arm1, 1, cfg, fig, 'S1-ARM1-LeftSide+Top');
    if stopRequested(fig)
        return
    end

    logMsg(fig, '  ARM1 done -> HOME');
    homeOneArm(ip1, cfg, fig, 'S1-ARM1-Home');
    pause(1.0);

    if stopRequested(fig)
        logMsg(fig, 'STOPPED by user');
        return
    end

    executePath(ip2, paths.arm2, 2, cfg, fig, 'S1-ARM2-RightSide+Top');
    if stopRequested(fig)
        return
    end

    logMsg(fig, '  ARM2 done -> HOME');
    homeOneArm(ip2, cfg, fig, 'S1-ARM2-Home');
    pause(1.0);

    if stopRequested(fig)
        logMsg(fig, 'STOPPED by user');
        return
    end

    h = fig.UserData;
    h.progLbl.Text = 'OK COMPLETE  100%';
    fig.UserData = h;

    logMsg(fig, '========================================');
    logMsg(fig, '  PAINTING COMPLETE');
    logMsg(fig, '  1 Station | Side faces + Top only');
    logMsg(fig, '========================================');
end

function cbStop(fig, cfg)
    h = fig.UserData;
    alreadyStopped = stopRequested(fig);
    h.stopFlag = true;
    if isfield(h, 'progLbl') && isvalid(h.progLbl)
        h.progLbl.Text = 'STOPPED';
    end
    fig.UserData = h;
    httpSend(h.eIP1.Value, cfg.BASE_URL, '{"T":0}', cfg.HTTP_TO, 1, 0.05);
    httpSend(h.eIP2.Value, cfg.BASE_URL, '{"T":0}', cfg.HTTP_TO, 1, 0.05);
    if ~alreadyStopped
        logMsg(fig, 'EMERGENCY STOP sent');
    end
end

function logMsg(fig, msg)
    ts = char(datetime('now', 'Format', 'HH:mm:ss'));
    line = ['[' ts ']  ' msg];
    fprintf('%s\n', line);

    h = fig.UserData;
    if ~isfield(h, 'log') || ~isvalid(h.log)
        return
    end

    cur = h.log.Value;
    if ischar(cur)
        cur = {cur};
    elseif isstring(cur)
        cur = cellstr(cur);
    end
    h.log.Value = [cur; {line}];

    try
        scroll(h.log, 'bottom');
    catch
    end
    drawnow limitrate;
end

function result = ternary(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
