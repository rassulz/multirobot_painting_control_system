%% ================================================================
%  RoArm_Painting_System_FINAL.m
%  3-Station Conveyor Belt Painting System
%  Station 1: Front face (both arms)
%  Station 2: Side faces + Top (each arm its side)
%  Station 3: Back face (both arms)
%% ================================================================

function RoArm_Painting_System_FINAL()
    clc; close all;
    cfg = buildConfig();
    buildMainGUI(cfg);
end

%% ================================================================
%  SECTION 1: CONFIGURATION
%% ================================================================
function cfg = buildConfig()
    cfg.TABLE_X   = 795;
    cfg.TABLE_Y   = 595;
    cfg.B1        = [80,   595/2,   60];
    cfg.B2        = [715, 595/2,   60];
    cfg.L0        = 123;
    cfg.L1        = 239;
    cfg.L2        = 280;
    cfg.L3        = 0;
    cfg.MAX_REACH = 480;
    cfg.MIN_REACH = 50;
    cfg.Z_MIN     = -50;
    cfg.Z_MAX     = 320;
    
    % ============ CONVEYOR BELT ============
    cfg.CONV_DX   = 105;            % conveyor width (X direction)
    cfg.CONV_DY   = 595;            % conveyor length (Y direction)
    cfg.CONV_DZ   = 50;             % conveyor height (Z)
    cfg.CONV_CX   = 795/2;          % centered on table X
    cfg.CONV_Y0   = 0;              % conveyor starts at Y=0

    % ============ OBJECT DIMENSIONS ============
    % Physical object: 10.5cm x 21cm x 7.5cm
    cfg.OBJ_DX    = 105;   % Width along X (fits conveyor width)
    cfg.OBJ_DY    = 210;   % Length along Y (along conveyor direction)
    cfg.OBJ_DZ    = 75;    % Height (Z)

    cfg.OBJ_CX    = cfg.CONV_CX;    % centered on conveyor X
    cfg.OBJ_Z0    = cfg.CONV_DZ;    % sits on top of conveyor

    % ============ 3 STATION POSITIONS (Y center of object) ============
    cfg.STATION_Y    = [105, 297.5, 490];
    cfg.STATION_WAIT = 10;           % seconds between stations

    cfg.STANDOFF  = 10;
    cfg.STRIPE_H  = 10;
    cfg.APPROACH  = 10;
    cfg.PAINT_SPD = 5.0;
    cfg.IP1       = '192.168.1.192';
    cfg.IP2       = '192.168.1.101';
    cfg.BASE_URL  = '/js?json=';
    cfg.HTTP_TO   = 0.8;
    cfg.READ_TO   = 0.3;
    cfg.MAX_RET   = 1;
    cfg.RET_DLY   = 0.05;
    cfg.POLL_INT  = 0.15;

    % --- NEW: motion settling parameters ---
    cfg.SETTLE_TOL     = 5.0;    % mm tolerance to consider "arrived"
    cfg.SETTLE_TIMEOUT = 1.0;    % max seconds to wait per waypoint
    cfg.SETTLE_POLL    = 0.25;   % seconds between position polls
end

%% ================================================================
%  SECTION 2: GUI BUILDER
%% ================================================================
function buildMainGUI(cfg)
    C.bg    = [0.95 0.95 0.95];
    C.panel = [1.00 1.00 1.00];
    C.acc1  = [0.20 0.50 0.90];
    C.acc2  = [0.90 0.45 0.10];
    C.grn   = [0.20 0.70 0.30];
    C.txt   = [0.15 0.15 0.15];
    C.dark  = [1.00 1.00 1.00];

    fig = uifigure('Name','RoArm-M2-S Dual-Arm Painting Controller', ...
        'Position',[30 30 1750 750], ...
        'Color',C.bg);

    uilabel(fig,'Text','RoArm-M2-S  |  Dual-Arm Painting System  |  3-Station Conveyor  |  Session HMI', ...
        'Position',[0 720 1750 28], ...
        'FontSize',14,'FontWeight','bold', ...
        'FontColor',C.acc1,'BackgroundColor',C.bg, ...
        'HorizontalAlignment','center');

    % ===== LEFT COLUMN: ARM panels + Config =====
    [h.e1x, h.e1y, h.e1z, h.btn1, h.lbl1, h.tel1] = ...
        makeArmPanel(fig, 'ARM 1 (Front)', [10 520 420 195], C, C.acc1, cfg, 1);
    [h.e2x, h.e2y, h.e2z, h.btn2, h.lbl2, h.tel2] = ...
        makeArmPanel(fig, 'ARM 2 (Rear)', [10 320 420 195], C, C.acc2, cfg, 2);

    h = makeConfigPanel(fig, h, C, cfg, [10 10 420 305]);

    % ===== CENTER: 3D Workspace Preview =====
    pnl3d = uipanel(fig,'Title','3D Workspace Preview','FontWeight','bold', ...
        'Position',[440 210 640 530], ...
        'BackgroundColor',C.panel,'ForegroundColor',C.txt,'BorderType','line');
    h.ax = uiaxes(pnl3d,'Position',[5 5 628 490], ...
        'Color',[1 1 1],'XColor',[0.3 0.3 0.3],'YColor',[0.3 0.3 0.3],'ZColor',[0.3 0.3 0.3], ...
        'GridColor',[0.80 0.80 0.80],'FontSize',8);
    hold(h.ax,'on'); grid(h.ax,'on');
    xlabel(h.ax,'X (mm)'); ylabel(h.ax,'Y (mm)'); zlabel(h.ax,'Z (mm)');
    title(h.ax,'Workspace & Paths','Color',C.txt,'FontSize',10);
    view(h.ax,42,26);
    drawWorkspace(h.ax, cfg, C);

    % ===== CENTER BOTTOM: Event Log =====
    logPan = uipanel(fig,'Title','Live Event Log','FontWeight','bold', ...
        'Position',[440 10 640 195], ...
        'BackgroundColor',C.panel,'ForegroundColor',C.txt, ...
        'FontSize',9,'FontWeight','bold','BorderType','line');
    h.log = uitextarea(logPan,'Position',[4 4 628 168], ...
        'BackgroundColor',[1 1 1],'FontColor',[0.1 0.1 0.1], ...
        'FontName','Courier New','FontSize',9,'Editable','off', ...
        'Value',{'[System] Ready. Click PING to test robot connections.', ...
                 'Conveyor: 105(X) x 595(Y) x 50(Z)mm - 3 Stations', ...
                 'Station 1: Front face | Station 2: Sides + Top', ...
                 'Station 3: Back face | Wait: 10s between stations'});

    % ===== RIGHT PANEL: Real-Time Telemetry (4 Graphs) =====
    pnlR = uipanel(fig,'Title','Real-Time Telemetry','FontWeight','bold', ...
        'Position',[1090 10 650 730], ...
        'BackgroundColor',C.panel,'ForegroundColor',C.txt,'BorderType','line');

    gw = 620; gh = 158; gx = 12;

    % ARM 1 — Cartesian Position (top)
    h.axPos1 = uiaxes(pnlR,'Position',[gx 530 gw gh]);
    hold(h.axPos1,'on'); grid(h.axPos1,'on');
    title(h.axPos1,'ARM 1 — Cartesian Position','FontSize',9);
    ylabel(h.axPos1,'mm'); xlabel(h.axPos1,'Step');
    h.linePos1X = animatedline(h.axPos1,'Color','r','LineWidth',1.3,'DisplayName','X');
    h.linePos1Y = animatedline(h.axPos1,'Color',[0 0.7 0],'LineWidth',1.3,'DisplayName','Y');
    h.linePos1Z = animatedline(h.axPos1,'Color','b','LineWidth',1.3,'DisplayName','Z');
    legend(h.axPos1,'Location','bestoutside','FontSize',7);

    % ARM 1 — Estimated Joint Torque
    h.axTrq1 = uiaxes(pnlR,'Position',[gx 362 gw gh]);
    hold(h.axTrq1,'on'); grid(h.axTrq1,'on');
    title(h.axTrq1,'ARM 1 — Estimated Joint Torque','FontSize',9);
    ylabel(h.axTrq1,'N{\cdot}mm'); xlabel(h.axTrq1,'Step');
    h.lineTrq1_1 = animatedline(h.axTrq1,'Color','r','LineWidth',1.3,'DisplayName','\tau_1 Base');
    h.lineTrq1_2 = animatedline(h.axTrq1,'Color',[0 0.7 0],'LineWidth',1.3,'DisplayName','\tau_2 Shoulder');
    h.lineTrq1_3 = animatedline(h.axTrq1,'Color','b','LineWidth',1.3,'DisplayName','\tau_3 Elbow');
    legend(h.axTrq1,'Location','bestoutside','FontSize',7);

    % ARM 2 — Cartesian Position
    h.axPos2 = uiaxes(pnlR,'Position',[gx 194 gw gh]);
    hold(h.axPos2,'on'); grid(h.axPos2,'on');
    title(h.axPos2,'ARM 2 — Cartesian Position','FontSize',9);
    ylabel(h.axPos2,'mm'); xlabel(h.axPos2,'Step');
    h.linePos2X = animatedline(h.axPos2,'Color','r','LineWidth',1.3,'DisplayName','X');
    h.linePos2Y = animatedline(h.axPos2,'Color',[0 0.7 0],'LineWidth',1.3,'DisplayName','Y');
    h.linePos2Z = animatedline(h.axPos2,'Color','b','LineWidth',1.3,'DisplayName','Z');
    legend(h.axPos2,'Location','bestoutside','FontSize',7);

    % ARM 2 — Estimated Joint Torque (bottom)
    h.axTrq2 = uiaxes(pnlR,'Position',[gx 26 gw gh]);
    hold(h.axTrq2,'on'); grid(h.axTrq2,'on');
    title(h.axTrq2,'ARM 2 — Estimated Joint Torque','FontSize',9);
    ylabel(h.axTrq2,'N{\cdot}mm'); xlabel(h.axTrq2,'Step');
    h.lineTrq2_1 = animatedline(h.axTrq2,'Color','r','LineWidth',1.3,'DisplayName','\tau_1 Base');
    h.lineTrq2_2 = animatedline(h.axTrq2,'Color',[0 0.7 0],'LineWidth',1.3,'DisplayName','\tau_2 Shoulder');
    h.lineTrq2_3 = animatedline(h.axTrq2,'Color','b','LineWidth',1.3,'DisplayName','\tau_3 Elbow');
    legend(h.axTrq2,'Location','bestoutside','FontSize',7);

    % Status label
    uilabel(fig,'Text','Status:', ...
        'Position',[445 722 50 22], ...
        'FontColor',C.txt,'BackgroundColor',C.bg,'FontSize',9);
    h.progLbl = uilabel(fig,'Text','Not Connected', ...
        'Position',[498 722 260 22], ...
        'FontColor',[0.7 0 0],'BackgroundColor',C.bg, ...
        'FontSize',10,'FontWeight','bold');

    % Stop flag + graph step counters
    h.stopFlag = false;
    h.stepCount1 = 0;
    h.stepCount2 = 0;

    fig.UserData = h;
    h.btn1.ButtonPushedFcn = @(~,~) cbMove(fig, 1, cfg);
    h.btn2.ButtonPushedFcn = @(~,~) cbMove(fig, 2, cfg);
    rewireConfigButtons(fig, cfg);
    fig.UserData = h;
end

%% ================================================================
%  ARM PANEL
%% ================================================================
function [ex,ey,ez,btn,lblIK,lblTel] = makeArmPanel(fig, titleStr, pos, C, acc, cfg, armIdx)
    pan = uipanel(fig,'Title',titleStr, ...
        'Position',pos,'BackgroundColor',C.panel,'ForegroundColor',acc, ...
        'FontSize',10,'FontWeight','bold','BorderType','line');

    row = @(n) 148 - n*27;
    lw = 95; ew = 110;

    if armIdx == 1
        defaultX = cfg.OBJ_CX;
        defaultY = cfg.STATION_Y(2) - 40;
        defaultZ = cfg.OBJ_Z0 + 50;
    else
        defaultX = cfg.OBJ_CX;
        defaultY = cfg.STATION_Y(2) + 40;
        defaultZ = cfg.OBJ_Z0 + 50;
    end

    uilabel(pan,'Text','X global (mm):', ...
        'Position',[8 row(0) lw 20],'FontColor',C.txt, ...
        'BackgroundColor',C.panel,'FontSize',9);
    ex = uieditfield(pan,'numeric','Position',[lw+8 row(0) ew 22], ...
        'Value',defaultX,'Limits',[-100 cfg.TABLE_X+100], ...
        'FontSize',9);

    uilabel(pan,'Text','Y global (mm):', ...
        'Position',[8 row(1) lw 20],'FontColor',C.txt, ...
        'BackgroundColor',C.panel,'FontSize',9);
    ey = uieditfield(pan,'numeric','Position',[lw+8 row(1) ew 22], ...
        'Value',defaultY,'Limits',[-100 cfg.TABLE_Y+100], ...
        'FontSize',9);

    uilabel(pan,'Text','Z height (mm):', ...
        'Position',[8 row(2) lw 20],'FontColor',C.txt, ...
        'BackgroundColor',C.panel,'FontSize',9);
    ez = uieditfield(pan,'numeric','Position',[lw+8 row(2) ew 22], ...
        'Value',defaultZ,'Limits',[0 350], ...
        'FontSize',9);

    btn = uibutton(pan,'Text','Move', ...
        'Position',[8 row(3) 210 26], ...
        'BackgroundColor',acc,'FontColor','w', ...
        'FontSize',9,'FontWeight','bold');

    lblIK = uilabel(pan,'Text','Ready', ...
        'Position',[8 row(4) 330 18],'FontColor',[0.0 0.6 0.0], ...
        'BackgroundColor',C.panel,'FontSize',8,'WordWrap','on');

    uilabel(pan,'Text','Live:', ...
        'Position',[8 row(5) 40 16],'FontColor',[0.4 0.4 0.5], ...
        'BackgroundColor',C.panel,'FontSize',8);
    lblTel = uilabel(pan,'Text','\u2014', ...
        'Position',[50 row(5) 288 16],'FontColor',[0.1 0.4 0.7], ...
        'BackgroundColor',C.panel,'FontSize',8);
end

function h = makeConfigPanel(fig, h, C, cfg, pos)
    pan = uipanel(fig,'Title','Settings & Control', ...
        'Position',pos,'BackgroundColor',C.panel,'ForegroundColor',C.txt, ...
        'FontSize',10,'FontWeight','bold','BorderType','line','Tag','cfgPanel');

    lw = 120; ew = 80;

    uilabel(pan,'Text','Stripe (mm):', ...
        'Position',[8 250 lw 20],'FontColor',C.txt, ...
        'BackgroundColor',C.panel,'FontSize',9);
    h.eStripe = uieditfield(pan,'numeric', ...
        'Position',[lw+5 250 ew 22],'Value',cfg.STRIPE_H, ...
        'Limits',[5 50]);

    uilabel(pan,'Text','Standoff (mm):', ...
        'Position',[8 224 lw 20],'FontColor',C.txt, ...
        'BackgroundColor',C.panel,'FontSize',9);
    h.eStandoff = uieditfield(pan,'numeric', ...
        'Position',[lw+5 224 ew 22],'Value',cfg.STANDOFF, ...
        'Limits',[10 100]);

    uilabel(pan,'Text','Speed:', ...
        'Position',[8 198 lw 20],'FontColor',C.txt, ...
        'BackgroundColor',C.panel,'FontSize',9);
    h.eSpeed = uieditfield(pan,'numeric', ...
        'Position',[lw+5 198 ew 22],'Value',cfg.PAINT_SPD, ...
        'Limits',[1 20]);

    uilabel(pan,'Text','ARM 1 IP:', ...
        'Position',[8 172 lw 20],'FontColor',C.txt, ...
        'BackgroundColor',C.panel,'FontSize',9);
    h.eIP1 = uieditfield(pan,'text', ...
        'Position',[lw+5 172 160 22],'Value',cfg.IP1);

    uilabel(pan,'Text','ARM 2 IP:', ...
        'Position',[8 146 lw 20],'FontColor',C.txt, ...
        'BackgroundColor',C.panel,'FontSize',9);
    h.eIP2 = uieditfield(pan,'text', ...
        'Position',[lw+5 146 160 22],'Value',cfg.IP2);

    uibutton(pan,'Text','Ping',...
        'Position',[8 112 120 28],...
        'BackgroundColor',[0.2 0.5 0.9],'FontColor','w','FontSize',9,'FontWeight','bold',...
        'Tag','btnPing');
    uibutton(pan,'Text','HOME',...
        'Position',[138 112 120 28],...
        'BackgroundColor',[0.5 0.3 0.7],'FontColor','w','FontSize',9,'FontWeight','bold',...
        'Tag','btnHome');
    uibutton(pan,'Text','Telemetry',...
        'Position',[268 112 130 28],...
        'BackgroundColor',[0.2 0.7 0.3],'FontColor','w','FontSize',9,'FontWeight','bold',...
        'Tag','btnTel');

    uibutton(pan,'Text','START PAINTING',...
        'Position',[8 58 390 42],...
        'BackgroundColor',[0.2 0.7 0.3],...
        'FontColor','w','FontSize',13,'FontWeight','bold',...
        'Tag','btnStart');

    uibutton(pan,'Text','STOP',...
        'Position',[8 8 390 42],...
        'BackgroundColor',[0.85 0.1 0.1],...
        'FontColor','w','FontSize',14,'FontWeight','bold',...
        'Tag','btnStop');
end

function rewireConfigButtons(fig, cfg)
    btn = findobj(fig,'Tag','btnPing');
    btn.ButtonPushedFcn = @(~,~) cbPing(fig, cfg);
    btn = findobj(fig,'Tag','btnHome');
    btn.ButtonPushedFcn = @(~,~) cbHome(fig, cfg);
    btn = findobj(fig,'Tag','btnTel');
    btn.ButtonPushedFcn = @(~,~) cbTel(fig, cfg);
    btn = findobj(fig,'Tag','btnStart');
    btn.ButtonPushedFcn = @(~,~) cbStart(fig, cfg);
    btn = findobj(fig,'Tag','btnStop');
    btn.ButtonPushedFcn = @(~,~) cbStop(fig, cfg);
end

%% ================================================================
%  SECTION: COORDINATE TRANSFORMATION (FIXED)
%
%  ARM1 base is at B1=[50, 297.5, 0] — left side, facing +X.
%    Robot local frame: local_x = forward (+X global)
%                       local_y = left    (+Y global, right-hand rule)
%  ARM2 base is at B2=[745, 297.5, 0] — right side, facing -X.
%    Robot local frame: local_x = forward (-X global)
%                       local_y = left    (-Y global, right-hand rule)
%
%  The RoArm-M2 T:104 command expects (x, y, z) in the robot's own
%  local frame where x = forward reach, y = lateral, z = height.
%% ================================================================
function [lx, ly, lz] = globalToLocal(gx, gy, gz, armIdx, cfg)
    if armIdx == 1
        % ARM1 at left side, facing +X into the table
        lx = gx - cfg.B1(1);          % forward = global +X from base
        ly = gy - cfg.B1(2);          % lateral left = global +Y
    else
        % ARM2 at right side, facing -X into the table
        lx = cfg.B2(1) - gx;          % forward = global -X from base
        ly = cfg.B2(2) - gy;          % lateral left = global -Y
    end
    lz = gz - cfg.B1(3);   % Z relative to base height (125mm above table)
end

%% ================================================================
%  SECTION: REACHABILITY CHECK (IMPROVED)
%% ================================================================
function [ok, msg] = checkReach(lx, ly, lz, cfg)
    r = sqrt(lx^2 + ly^2);       % horizontal reach
    zR = lz - cfg.L0;            % height above shoulder
    d3 = sqrt(r^2 + zR^2);       % 3D distance from shoulder

    if lx < 0
        ok = false;
        msg = sprintf('BEHIND BASE: lx=%.1f (must be >= 0)', lx);
        return
    end
    if d3 > (cfg.L1 + cfg.L2 + cfg.L3)
        ok = false;
        msg = sprintf('OUT OF REACH: %.1f mm (max %.0f)', d3, cfg.L1+cfg.L2+cfg.L3);
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

%% ================================================================
%  SECTION: JOINT ANGLE COMPUTATION (3-DOF Inverse Kinematics)
%  Computes joint angles from local Cartesian coordinates.
%  Joint 1: Base rotation (about Z)
%  Joint 2: Shoulder (about lateral axis)
%  Joint 3: Elbow (about lateral axis)
%% ================================================================
function [theta1, theta2, theta3] = computeJointAngles(lx, ly, lz, cfg)
    % Base rotation
    theta1 = atan2d(ly, lx);

    % Planar reach and height above shoulder
    r  = sqrt(lx^2 + ly^2);        % horizontal distance
    zs = lz - cfg.L0;              % height above shoulder pivot
    d  = sqrt(r^2 + zs^2);         % distance from shoulder to target

    L1 = cfg.L1;  % upper arm
    L2 = cfg.L2;  % forearm

    % Elbow angle (law of cosines)
    cosQ3 = (d^2 - L1^2 - L2^2) / (2 * L1 * L2);
    cosQ3 = max(-1, min(1, cosQ3));   % clamp for numerical safety
    theta3 = acosd(cosQ3);

    % Shoulder angle
    alpha = atan2d(zs, r);
    beta  = atan2d(L2 * sind(theta3), L1 + L2 * cosd(theta3));
    theta2 = alpha + beta;
end

%% ================================================================
%  SECTION: STATIC TORQUE ESTIMATION
%  Computes gravity-induced static joint torques from joint angles.
%  Uses estimated link masses for RoArm-M2-S.
%  Returns torques in N*mm.
%% ================================================================
function torques = computeStaticTorques(theta2, theta3, cfg)
    % Gravity constant (m/s^2)
    g = 9.81;

    % Estimated link masses (kg) for RoArm-M2-S
    m1 = 0.120;   % upper arm
    m2 = 0.085;   % forearm
    m3 = 0.040;   % end effector / gripper

    L1 = cfg.L1;  % mm
    L2 = cfg.L2;  % mm

    % Absolute angle of forearm from horizontal (degrees)
    phi = theta2 - theta3;

    % Torque at elbow (joint 3): supports forearm COM + end effector
    %   tau3 = (m2/2 + m3) * g * L2 * cos(phi)
    tau3 = (m2/2 + m3) * g * L2 * cosd(phi);

    % Torque at shoulder (joint 2): supports entire arm
    %   tau2 = m1*g*(L1/2)*cos(theta2) + (m2+m3)*g*L1*cos(theta2) + tau3_contribution
    tau2 = (m1/2) * g * L1 * cosd(theta2) + ...
           (m2 + m3) * g * L1 * cosd(theta2) + ...
           (m2/2 + m3) * g * L2 * cosd(phi);

    % Torque at base (joint 1): gravity has no direct effect on
    % vertical rotation axis. Approximate as friction/inertial loading
    % proportional to horizontal arm extension.
    rHoriz = abs(L1 * cosd(theta2) + L2 * cosd(phi));
    tau1 = 0.008 * (m1 + m2 + m3) * g * rHoriz;

    torques = [tau1, tau2, tau3];
end

%% ================================================================
%  SECTION: HTTP COMMUNICATION (UNCHANGED)
%% ================================================================
function [resp, success] = httpSend(ip, baseURL, jsonCmd, timeout, maxRet, retDly)
    url = ['http://' char(ip) char(baseURL) urlencode(jsonCmd)];
    resp = '';
    success = false;
    for k = 1:maxRet
        try
            resp = webread(url, weboptions('Timeout', timeout));
            success = true;
            return;
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
    x = getField(data, {'x','X','px','posx'});
    y = getField(data, {'y','Y','py','posy'});
    z = getField(data, {'z','Z','pz','posz'});
end

function v = getField(s, names)
    v = 0;
    if isempty(s) || ~isstruct(s), return; end
    for k = 1:numel(names)
        if isfield(s, names{k})
            try
                v = double(s.(names{k}));
                return;
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

%% ================================================================
%  SECTION: WAIT FOR MOTION COMPLETION (NEW)
%  Polls the robot until it reaches within tolerance of target,
%  or until timeout expires.
%% ================================================================
function arrived = waitForArrival(ip, targetLocal, cfg)
    % targetLocal = [lx, ly, lz] in robot local frame
    tStart = tic;
    arrived = false;

    while toc(tStart) < cfg.SETTLE_TIMEOUT
        pause(cfg.SETTLE_POLL);
        data = httpReadStatus(ip, cfg.READ_TO);
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
%  SECTION: PATH GENERATION (3-STATION CONVEYOR SYSTEM)
%
%  Station 1 (beginning): Front face (+Y perpendicular, facing robots)
%  Station 2 (middle):    Side faces (X-perpendicular) + Top
%  Station 3 (end):       Back face (-Y perpendicular, facing robots)
%
%  Conveyor moves object along Y: Station1 -> Station2 -> Station3
%  Both arms collaborate at each station
%% ================================================================
function paths = generateStationPaths(cfg, stationIdx)
    so  = cfg.STANDOFF;
    sH  = cfg.STRIPE_H;
    app = cfg.APPROACH;
    cx  = cfg.OBJ_CX;
    cy  = cfg.STATION_Y(stationIdx);   % object center Y at this station
    dx  = cfg.OBJ_DX / 2;   % half-width in X
    dy  = cfg.OBJ_DY / 2;   % half-width in Y
    dz  = cfg.OBJ_DZ;
    z0  = cfg.OBJ_Z0;

    % Z stripe positions for vertical faces
    zBot = z0 + 8;
    zTop = z0 + dz - 8;
    nStrZ = max(1, floor((zTop - zBot) / sH) + 1);
    zLevels = linspace(zBot, zTop, nStrZ);

    paths = struct();
    midZ = ceil(nStrZ / 2);

    switch stationIdx
        case 1
            % STATION 1: Front face only (+Y perpendicular)
            % Face at Y = cy + dy, facing towards robots at Y=297.5
            yFront = cy + dy + so;
            % ARM1 paints lower Z stripes, ARM2 paints upper Z stripes
            if midZ >= 1
                paths.arm1 = genFacePathY(yFront, cx, dx, zLevels(1:midZ), app);
            else
                paths.arm1 = zeros(0,3);
            end
            if midZ < nStrZ
                paths.arm2 = genFacePathY(yFront, cx, dx, zLevels(midZ+1:end), app);
            else
                paths.arm2 = genFacePathY(yFront, cx, dx, zLevels(end), app);
            end

        case 2
            % STATION 2: Side faces + Top
            % ARM1 side: -X face at X = cx - dx (facing ARM1)
            xArm1Side = cx - dx - so;
            pathArm1Side = genFacePathX(xArm1Side, cy, dy, zLevels, app);

            % ARM2 side: +X face at X = cx + dx (facing ARM2)
            xArm2Side = cx + dx + so;
            pathArm2Side = genFacePathX(xArm2Side, cy, dy, zLevels, app);

            % Top face: Z = z0 + dz + so, split between arms
            zTopFace = z0 + dz + so;
            xLeft  = cx - dx + 8;
            xRight = cx + dx - 8;
            xMid   = cx;

            % ARM1 does left half of top (lower X, closer to ARM1)
            nStrX1 = max(1, floor((xMid - xLeft) / sH) + 1);
            xLevels1 = linspace(xLeft, xMid, nStrX1);
            pathArm1Top = genTopPath(xLevels1, cy, dy, zTopFace, app);

            % ARM2 does right half of top (higher X, closer to ARM2)
            nStrX2 = max(1, floor((xRight - xMid) / sH) + 1);
            xLevels2 = linspace(xMid, xRight, nStrX2);
            pathArm2Top = genTopPath(xLevels2, cy, dy, zTopFace, app);

            % Combine: side face first, then top
            paths.arm1 = [pathArm1Side; pathArm1Top];
            paths.arm2 = [pathArm2Side; pathArm2Top];

        case 3
            % STATION 3: Back face only (-Y perpendicular)
            % Face at Y = cy - dy, facing back towards robots at Y=297.5
            yBack = cy - dy - so;
            % ARM1 paints lower Z stripes, ARM2 paints upper Z stripes
            if midZ >= 1
                paths.arm1 = genFacePathY(yBack, cx, dx, zLevels(1:midZ), app);
            else
                paths.arm1 = zeros(0,3);
            end
            if midZ < nStrZ
                paths.arm2 = genFacePathY(yBack, cx, dx, zLevels(midZ+1:end), app);
            else
                paths.arm2 = genFacePathY(yBack, cx, dx, zLevels(end), app);
            end
    end
end

%% --- Path helper: Y-perpendicular face (sweeps along X) ---
function path = genFacePathY(yNozzle, cx, dx, zLevels, app)
    path = [];
    for i = 1:numel(zLevels)
        z = zLevels(i);
        if mod(i,2) == 1
            path = [path;
                cx-dx-app, yNozzle, z;
                cx-dx,     yNozzle, z;
                cx+dx,     yNozzle, z;
                cx+dx+app, yNozzle, z];
        else
            path = [path;
                cx+dx+app, yNozzle, z;
                cx+dx,     yNozzle, z;
                cx-dx,     yNozzle, z;
                cx-dx-app, yNozzle, z];
        end
    end
end

%% --- Path helper: X-perpendicular face (sweeps along Y) ---
function path = genFacePathX(xNozzle, cy, dy, zLevels, app)
    path = [];
    for i = 1:numel(zLevels)
        z = zLevels(i);
        if mod(i,2) == 1
            path = [path;
                xNozzle, cy-dy-app, z;
                xNozzle, cy-dy,     z;
                xNozzle, cy+dy,     z;
                xNozzle, cy+dy+app, z];
        else
            path = [path;
                xNozzle, cy+dy+app, z;
                xNozzle, cy+dy,     z;
                xNozzle, cy-dy,     z;
                xNozzle, cy-dy-app, z];
        end
    end
end

%% --- Path helper: Top face (sweeps along Y at each X) ---
function path = genTopPath(xLevels, cy, dy, zTop, app)
    path = [];
    for i = 1:numel(xLevels)
        x = xLevels(i);
        if mod(i,2) == 1
            path = [path;
                x, cy-dy-app, zTop;
                x, cy-dy,     zTop;
                x, cy+dy,     zTop;
                x, cy+dy+app, zTop];
        else
            path = [path;
                x, cy+dy+app, zTop;
                x, cy+dy,     zTop;
                x, cy-dy,     zTop;
                x, cy-dy-app, zTop];
        end
    end
end

%% ================================================================
%  SECTION: PATH VALIDATION
%  Pre-checks all waypoints for a station
%% ================================================================
function [validCount, totalCount, issues] = validateStationPaths(paths, cfg)
    fNames  = fieldnames(paths);
    totalCount = 0;
    validCount = 0;
    issues = {};

    for f = 1:numel(fNames)
        name = fNames{f};
        pts  = paths.(name);
        n    = size(pts, 1);
        if n == 0, continue; end

        % Determine arm index from field name
        if contains(name, 'arm1')
            aIdx = 1;
        else
            aIdx = 2;
        end

        faceValid = 0;
        for i = 1:n
            [lx, ly, lz] = globalToLocal(pts(i,1), pts(i,2), pts(i,3), aIdx, cfg);
            [ok, ~] = checkReach(lx, ly, lz, cfg);
            if ok
                faceValid = faceValid + 1;
            end
        end
        totalCount = totalCount + n;
        validCount = validCount + faceValid;
        if faceValid < n
            issues{end+1} = sprintf('%s: %d/%d reachable', ...
                name, faceValid, n); %#ok<AGROW>
        end
    end
end

%% ================================================================
%  SECTION: EXECUTE PATH (IMPROVED)
%  Now waits for each waypoint arrival before proceeding
%% ================================================================
function executePath(ip, pathG, armIdx, cfg, fig, label)
    n = size(pathG, 1);
    logMsg(fig, sprintf('[%s] Starting %d waypoints', label, n));
    h = fig.UserData;

    skipped = 0;
    failed  = 0;

    for i = 1:n
        h = fig.UserData;

        % Check stop flag
        if isfield(h, 'stopFlag') && h.stopFlag
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
        cmd = sprintf('{"T":104,"x":%d,"y":%d,"z":%d,"t":0,"spd":%.2f}', ...
            round(lx), round(ly), round(lz), spd);
        [~, success] = httpSend(ip, cfg.BASE_URL, cmd, cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);

        if ~success
            logMsg(fig, sprintf('  [FAIL %d/%d] Robot not responding!', i, n));
            failed = failed + 1;
            if failed >= 5
                logMsg(fig, sprintf('[%s] Too many failures — aborting face', label));
                return
            end
            continue
        end

        % Wait for the robot to actually reach the target
        arrived = waitForArrival(ip, [lx, ly, lz], cfg);
        if ~arrived
            logMsg(fig, sprintf('  [TIMEOUT %d/%d] Robot did not reach target in time', i, n));
        end

        % Plot waypoint
        if armIdx == 1
            plot3(h.ax, gx, gy, gz, '.', 'Color',[0.15 0.55 1.0], 'MarkerSize',5);
        else
            plot3(h.ax, gx, gy, gz, '.', 'Color',[1.0 0.55 0.15], 'MarkerSize',5);
        end

        % --- Update telemetry graphs ---
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
        fig.UserData = h;
        drawnow limitrate;

        pct = round(i/n * 100);
        h.progLbl.Text = sprintf('[%s] %d%%  (%d/%d)', label, pct, i, n);
    end

    logMsg(fig, sprintf('[%s] Complete — %d sent, %d skipped, %d failed', ...
        label, n - skipped - failed, skipped, failed));
end

%% ================================================================
%  SECTION: 3D VISUALIZATION WITH ROBOT IMAGES
%% ================================================================
function drawWorkspace(ax, cfg, C)
    % Table (light gray)
    tx = [0 cfg.TABLE_X cfg.TABLE_X 0];
    ty = [0 0 cfg.TABLE_Y cfg.TABLE_Y];
    fill3(ax, tx, ty, zeros(1,4), [0.85 0.85 0.85], ...
        'FaceAlpha',0.35,'EdgeColor','none');
    plot3(ax, [tx tx(1)], [ty ty(1)], zeros(1,5), ...
        'Color',[0.55 0.55 0.60],'LineWidth',1.1);

    % Conveyor Belt
    convX0 = cfg.CONV_CX - cfg.CONV_DX/2;
    drawBox(ax, convX0, cfg.CONV_Y0, 0, ...
        cfg.CONV_DX, cfg.CONV_DY, cfg.CONV_DZ, ...
        [0.50 0.50 0.55], 0.35);
    text(ax, cfg.CONV_CX, cfg.CONV_DY/2, cfg.CONV_DZ+5, ...
        'CONVEYOR', 'Color',[0.35 0.35 0.40], ...
        'FontSize',7,'HorizontalAlignment','center');

    % Draw objects at 3 station positions
    stColors = {[0.2 0.75 0.3], [0.90 0.65 0.1], [0.3 0.5 0.85]};
    stLabels = {'S1:Front','S2:Side+Top','S3:Back'};
    for s = 1:3
        objX0 = cfg.OBJ_CX - cfg.OBJ_DX/2;
        objY0 = cfg.STATION_Y(s) - cfg.OBJ_DY/2;
        drawBox(ax, objX0, objY0, cfg.OBJ_Z0, ...
            cfg.OBJ_DX, cfg.OBJ_DY, cfg.OBJ_DZ, ...
            stColors{s}, 0.45);
        text(ax, cfg.OBJ_CX, cfg.STATION_Y(s), ...
            cfg.OBJ_Z0 + cfg.OBJ_DZ + 15, stLabels{s}, ...
            'Color', stColors{s}, 'FontSize',7, ...
            'HorizontalAlignment','center','FontWeight','bold');
    end

    % Robot platform boxes (60x60x60)
    bz = cfg.B1(3);  % base height
    platW = 60; platH = 60;
    % ARM1 platform
    drawBox(ax, cfg.B1(1)-platW/2, cfg.B1(2)-platW/2, 0, ...
        platW, platW, platH, [0.60 0.60 0.65], 0.60);
    % ARM2 platform
    drawBox(ax, cfg.B2(1)-platW/2, cfg.B2(2)-platW/2, 0, ...
        platW, platW, platH, [0.60 0.60 0.65], 0.60);

    % Draw robots as 3D models
    drawRobotArm(ax, cfg.B1(1), cfg.B1(2), bz, C.acc1, 0);     % ARM1 facing +X
    drawRobotArm(ax, cfg.B2(1), cfg.B2(2), bz, C.acc2, 180);   % ARM2 facing -X

    % Labels
    text(ax, cfg.B1(1)+12, cfg.B1(2)+25, bz+80, 'ARM 1', ...
        'Color',C.acc1,'FontSize',9,'FontWeight','bold');
    text(ax, cfg.B2(1)+12, cfg.B2(2)-35, bz+80, 'ARM 2', ...
        'Color',C.acc2,'FontSize',9,'FontWeight','bold');

    % Workspace circles (at base height)
    th = linspace(0, 2*pi, 64);
    r = cfg.MAX_REACH;
    plot3(ax, cfg.B1(1)+r*cos(th), cfg.B1(2)+r*sin(th), bz*ones(1,64), ...
        '--','Color',[C.acc1 0.40],'LineWidth',0.7);
    plot3(ax, cfg.B2(1)+r*cos(th), cfg.B2(2)+r*sin(th), bz*ones(1,64), ...
        '--','Color',[C.acc2 0.40],'LineWidth',0.7);

    % Station markers on conveyor
    for s = 1:3
        plot3(ax, [cfg.CONV_CX-60 cfg.CONV_CX+60], ...
            [cfg.STATION_Y(s) cfg.STATION_Y(s)], [1 1], ...
            '-','Color',[0.8 0.2 0.2],'LineWidth',1.5);
    end

    axis(ax,'equal');
    xlim(ax,[-70 cfg.TABLE_X+70]);
    ylim(ax,[-70 cfg.TABLE_Y+70]);
    zlim(ax,[0 280]);
end

function drawRobotArm(ax, x0, y0, z0, color, rotation_deg)
    theta = deg2rad(rotation_deg);

    % Base cylinder
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
        'FaceColor', color, 'FaceAlpha', 0.7, ...
        'EdgeColor', 'none');

    % Link 1
    l1_start = [x0, y0, z0 + 54];
    l1_end   = [x0 + 30*cos(theta), y0 + 30*sin(theta), z0 + 100];
    plot3(ax, [l1_start(1) l1_end(1)], ...
              [l1_start(2) l1_end(2)], ...
              [l1_start(3) l1_end(3)], ...
        'LineWidth', 8, 'Color', color);

    % Link 2
    l2_end = [l1_end(1) + 70*cos(theta), ...
              l1_end(2) + 70*sin(theta), ...
              l1_end(3) + 20];
    plot3(ax, [l1_end(1) l2_end(1)], ...
              [l1_end(2) l2_end(2)], ...
              [l1_end(3) l2_end(3)], ...
        'LineWidth', 7, 'Color', color);

    % Link 3
    l3_end = [l2_end(1) + 50*cos(theta), ...
              l2_end(2) + 50*sin(theta), ...
              l2_end(3) - 10];
    plot3(ax, [l2_end(1) l3_end(1)], ...
              [l2_end(2) l3_end(2)], ...
              [l2_end(3) l3_end(3)], ...
        'LineWidth', 5, 'Color', color);

    % End effector
    [xg, yg, zg] = sphere(10);
    xg = xg * 12 + l3_end(1);
    yg = yg * 12 + l3_end(2);
    zg = zg * 12 + l3_end(3);
    surf(ax, xg, yg, zg, ...
        'FaceColor', [0.9 0.9 0.9], 'FaceAlpha', 0.8, ...
        'EdgeColor', 'none');

    % Joint markers
    plot3(ax, l1_start(1), l1_start(2), l1_start(3), 'o', ...
        'MarkerSize', 8, 'MarkerFaceColor', [0.3 0.3 0.3], 'Color', 'k');
    plot3(ax, l1_end(1), l1_end(2), l1_end(3), 'o', ...
        'MarkerSize', 7, 'MarkerFaceColor', [0.3 0.3 0.3], 'Color', 'k');
    plot3(ax, l2_end(1), l2_end(2), l2_end(3), 'o', ...
        'MarkerSize', 6, 'MarkerFaceColor', [0.3 0.3 0.3], 'Color', 'k');
end

function drawBox(ax, x0, y0, z0, dx, dy, dz, col, alpha)
    V = [x0    y0    z0;
         x0+dx y0    z0;
         x0+dx y0+dy z0;
         x0    y0+dy z0;
         x0    y0    z0+dz;
         x0+dx y0    z0+dz;
         x0+dx y0+dy z0+dz;
         x0    y0+dy z0+dz];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch(ax,'Vertices',V,'Faces',F, ...
        'FaceColor',col,'FaceAlpha',alpha, ...
        'EdgeColor',[0.4 0.4 0.4],'LineWidth',0.65);
end

%% ================================================================
%  SECTION: CALLBACKS (IMPROVED)
%% ================================================================
function cbMove(fig, armIdx, cfg)
    h = fig.UserData;

    if armIdx == 1
        gx = h.e1x.Value; gy = h.e1y.Value; gz = h.e1z.Value;
        ip = h.eIP1.Value; lblIK = h.lbl1; lblTel = h.tel1;
    else
        gx = h.e2x.Value; gy = h.e2y.Value; gz = h.e2z.Value;
        ip = h.eIP2.Value; lblIK = h.lbl2; lblTel = h.tel2;
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
    cmd = sprintf('{"T":104,"x":%d,"y":%d,"z":%d,"t":0,"spd":%.2f}', ...
        round(lx), round(ly), round(lz), spd);
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

function cbPing(fig, cfg)
    h = fig.UserData;
    ips = {h.eIP1.Value, h.eIP2.Value};

    arm1ok = false;
    arm2ok = false;

    for k = 1:2
        data = httpReadStatus(ips{k}, 1.5);
        if ~isempty(data)
            [rx, ry, rz] = parseTel(data);
            logMsg(fig, sprintf('[ARM%d] OK ONLINE  X=%.1f  Y=%.1f  Z=%.1f', k, rx, ry, rz));
            if k==1, arm1ok=true; else, arm2ok=true; end
        else
            logMsg(fig, sprintf('[ARM%d] X NO RESPONSE  (%s)', k, ips{k}));
        end
    end

    if arm1ok && arm2ok
        h.progLbl.Text = 'OK Both Arms Connected';
        h.progLbl.FontColor = [0.0 0.6 0.0];
    elseif arm1ok || arm2ok
        h.progLbl.Text = '! Partial Connection';
        h.progLbl.FontColor = [0.8 0.5 0.0];
    else
        h.progLbl.Text = 'X Not Connected';
        h.progLbl.FontColor = [0.7 0.0 0.0];
    end
    fig.UserData = h;
end

function cbHome(fig, cfg)
    h = fig.UserData;
    httpSend(h.eIP1.Value, cfg.BASE_URL, '{"T":100}', ...
        cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);
    httpSend(h.eIP2.Value, cfg.BASE_URL, '{"T":100}', ...
        cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);
    logMsg(fig, '[System] HOME command sent to both arms');
end

function homeOneArm(ip, cfg)
    httpSend(ip, cfg.BASE_URL, '{"T":100}', ...
        cfg.HTTP_TO, cfg.MAX_RET, cfg.RET_DLY);
end

function cbTel(fig, cfg)
    h = fig.UserData;

    for k = 1:2
        if k == 1
            ip = h.eIP1.Value; lbl = h.tel1;
        else
            ip = h.eIP2.Value; lbl = h.tel2;
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

    % Reset stop flag
    h.stopFlag = false;
    fig.UserData = h;

    % --- Clear telemetry graphs for new run ---
    h.stepCount1 = 0;
    h.stepCount2 = 0;
    clearpoints(h.linePos1X); clearpoints(h.linePos1Y); clearpoints(h.linePos1Z);
    clearpoints(h.lineTrq1_1); clearpoints(h.lineTrq1_2); clearpoints(h.lineTrq1_3);
    clearpoints(h.linePos2X); clearpoints(h.linePos2Y); clearpoints(h.linePos2Z);
    clearpoints(h.lineTrq2_1); clearpoints(h.lineTrq2_2); clearpoints(h.lineTrq2_3);
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
            ternary(arm1ok,'OK Connected','X Offline'), ...
            ternary(arm2ok,'OK Connected','X Offline')), ...
            'Connection Required', 'Icon','error');
        return
    end

    logMsg(fig, 'OK Both robots connected');

    cfg.STRIPE_H  = h.eStripe.Value;
    cfg.STANDOFF  = h.eStandoff.Value;
    cfg.PAINT_SPD = h.eSpeed.Value;

    logMsg(fig, sprintf('  Stripe=%.0fmm  Standoff=%.0fmm  Speed=%.1f', ...
        cfg.STRIPE_H, cfg.STANDOFF, cfg.PAINT_SPD));

    % Pre-validate all 3 stations
    totalValid = 0; totalAll = 0; allIssues = {};
    for s = 1:3
        sp = generateStationPaths(cfg, s);
        [v, t, iss] = validateStationPaths(sp, cfg);
        totalValid = totalValid + v;
        totalAll   = totalAll + t;
        allIssues  = [allIssues, iss]; %#ok<AGROW>
    end
    logMsg(fig, sprintf('  Path validation: %d/%d waypoints reachable', totalValid, totalAll));
    for k = 1:numel(allIssues)
        logMsg(fig, sprintf('  WARNING: %s', allIssues{k}));
    end

    if totalValid == 0
        logMsg(fig, 'ABORT: No reachable waypoints! Check object placement.');
        uialert(fig, 'No waypoints are reachable. The object may be too far from the robot bases.', ...
            'Path Error', 'Icon', 'error');
        return
    end

    logMsg(fig, '  3-STATION CONVEYOR PAINTING SEQUENCE');
    logMsg(fig, '  Station 1: Front | Station 2: Sides+Top | Station 3: Back');
    logMsg(fig, sprintf('  Conveyor wait between stations: %ds', cfg.STATION_WAIT));
    logMsg(fig, '========================================');

    cbHome(fig, cfg);
    pause(2.0);

    % === STATION 1: Front Face (both arms) ===
    h = fig.UserData;
    if isfield(h, 'stopFlag') && h.stopFlag
        logMsg(fig, 'STOPPED by user'); cbHome(fig, cfg); return
    end
    logMsg(fig, '--- STATION 1: Front Face (Y+ perpendicular) ---');
    h.progLbl.Text = 'Station 1: Front Face';
    fig.UserData = h;
    paths1 = generateStationPaths(cfg, 1);
    executePath(ip1, paths1.arm1, 1, cfg, fig, 'S1-ARM1-Front');
    logMsg(fig, '  ARM1 done -> HOME (collision avoidance)');
    homeOneArm(ip1, cfg);
    pause(1.0);
    executePath(ip2, paths1.arm2, 2, cfg, fig, 'S1-ARM2-Front');
    logMsg(fig, '  ARM2 done -> HOME (collision avoidance)');
    homeOneArm(ip2, cfg);
    pause(1.0);

    logMsg(fig, sprintf('--- Station 1 done. Conveyor moving to Station 2 (%ds) ---', cfg.STATION_WAIT));
    h = fig.UserData;
    h.progLbl.Text = 'Conveyor -> Station 2...';
    fig.UserData = h;
    pause(cfg.STATION_WAIT);

    % === STATION 2: Side Faces + Top (each arm its side) ===
    h = fig.UserData;
    if isfield(h, 'stopFlag') && h.stopFlag
        logMsg(fig, 'STOPPED by user'); cbHome(fig, cfg); return
    end
    logMsg(fig, '--- STATION 2: Side Faces + Top ---');
    h.progLbl.Text = 'Station 2: Sides + Top';
    fig.UserData = h;
    paths2 = generateStationPaths(cfg, 2);
    executePath(ip1, paths2.arm1, 1, cfg, fig, 'S2-ARM1-Side+Top');
    logMsg(fig, '  ARM1 done -> HOME (collision avoidance)');
    homeOneArm(ip1, cfg);
    pause(1.0);
    executePath(ip2, paths2.arm2, 2, cfg, fig, 'S2-ARM2-Side+Top');
    logMsg(fig, '  ARM2 done -> HOME (collision avoidance)');
    homeOneArm(ip2, cfg);
    pause(1.0);

    logMsg(fig, sprintf('--- Station 2 done. Conveyor moving to Station 3 (%ds) ---', cfg.STATION_WAIT));
    h = fig.UserData;
    h.progLbl.Text = 'Conveyor -> Station 3...';
    fig.UserData = h;
    pause(cfg.STATION_WAIT);

    % === STATION 3: Back Face (both arms) ===
    h = fig.UserData;
    if isfield(h, 'stopFlag') && h.stopFlag
        logMsg(fig, 'STOPPED by user'); cbHome(fig, cfg); return
    end
    logMsg(fig, '--- STATION 3: Back Face (Y- perpendicular) ---');
    h.progLbl.Text = 'Station 3: Back Face';
    fig.UserData = h;
    paths3 = generateStationPaths(cfg, 3);
    executePath(ip1, paths3.arm1, 1, cfg, fig, 'S3-ARM1-Back');
    logMsg(fig, '  ARM1 done -> HOME (collision avoidance)');
    homeOneArm(ip1, cfg);
    pause(1.0);
    executePath(ip2, paths3.arm2, 2, cfg, fig, 'S3-ARM2-Back');
    logMsg(fig, '  ARM2 done -> HOME (collision avoidance)');
    homeOneArm(ip2, cfg);
    pause(1.0);

    h = fig.UserData;
    h.progLbl.Text = 'OK COMPLETE  100%';
    fig.UserData = h;

    logMsg(fig, '========================================');
    logMsg(fig, '  PAINTING COMPLETE - ALL FACES DONE');
    logMsg(fig, '  3 Stations | Front + Sides + Top + Back');
    logMsg(fig, '========================================');
end

function cbStop(fig, cfg)
    h = fig.UserData;
    h.stopFlag = true;
    fig.UserData = h;
    httpSend(h.eIP1.Value, cfg.BASE_URL, '{"T":0}', cfg.HTTP_TO, 1, 0.05);
    httpSend(h.eIP2.Value, cfg.BASE_URL, '{"T":0}', cfg.HTTP_TO, 1, 0.05);
    logMsg(fig, 'EMERGENCY STOP sent');
end

function logMsg(fig, msg)
    h = fig.UserData;
    if ~isfield(h,'log') || ~isvalid(h.log), return; end

    ts = datestr(now, 'HH:MM:SS'); %#ok<TNOW1,DATST>
    cur = h.log.Value;
    if ischar(cur), cur = {cur}; end
    h.log.Value = [cur; {['[' ts ']  ' msg]}];

    try; scroll(h.log, 'bottom'); catch; end
    drawnow limitrate;
end

function result = ternary(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end