function kuka_pick_and_place_gui_magnetic()
%% KUKA Pick-and-Place GUI with MAGNETIC END EFFECTOR (magnetic_cmd -> $OUT[25])
%  - Uses a REAL trigger pulse width (12 ms) so KRL can detect move_trigger
%  - Uses smooth quintic interpolation between A4-safe key poses
%  - Adds GUI for Home / Pick / Place + Connect / Preview / Execute / STOP
%  - Magnetic control writes magnetic_cmd (BOOL) via KUKAVARPROXY
%  - MatlabControl.src mirrors magnetic_cmd to physical $OUT[25]
%  - KUKAVARPROXY running
%  - MatlabControl.src running and reading:
%       target_pos   (POS)
%       move_trigger (BOOL)
%       magnetic_cmd   (BOOL)
%  - Robot in T1 and safety cleared
%  - $OUT[25] wired to the magnetic end effector

    %% ===== UI FIGURE =====
    fig = uifigure('Name','KUKA Pick-and-Place (A4-safe Smooth GUI + Magnetic)', ...
        'Position',[40 30 1750 700], 'Color',[0.95 0.95 0.95], ...
        'CloseRequestFcn',@onClose);

    %% ===== STATE =====
    S = struct();
    S.robot_model = 'KR10_R1100';
    S.max_reach = 1100;
    S.min_reach = 300;

    S.z_table = 0;
    S.z_clearance = 300;

    % Default poses (mm / deg) - A4/A5-safe orientation
    S.home  = [550  0   650   -180  0.5  -179.90];
    S.pick  = [608  117 263   -180  0.5  -179.90];
    S.place = [45   204 204   -180  0.5  -179.90];

    % Connection
    S.robot_ip = '172.31.17.101';
    S.port = 7000;
    S.tcp = [];
    S.connected = false;

    % Controller-chosen S/T tracking
    S.robot_s = 2;
    S.robot_t = 35;

    % Trajectory
    S.key_poses = [];
    S.waypoints = [];

    % Execution
    S.running = false;
    S.stopRequested = false;

    % Magnetic state (NEW)
    S.magnetic_enabled = true;
    S.magnetic_engage_delay = 0.8;
    S.magnetic_release_delay = 0.5;
    S.magnetic_is_on = false;

    %% ===== LEFT PANEL =====
    pnlL = uipanel(fig,'Title','Setup','FontWeight','bold', ...
        'Position',[10 10 420 680]);

    % Robot model
    uilabel(pnlL,'Text','Robot Model','Position',[10 640 120 22],'FontWeight','bold');
    ddModel = uidropdown(pnlL,'Position',[140 640 260 22], ...
        'Items',{'KR 6 R900 (900mm)','KR 10 R1100 (1100mm)','KR 16 R1610 (1600mm)','Custom'}, ...
        'Value','KR 10 R1100 (1100mm)', 'ValueChangedFcn',@onModelChanged);

    lblReach = uilabel(pnlL,'Text','Reach: 1100 mm','Position',[140 618 260 20], ...
        'FontColor',[0.3 0.3 0.3]);

    lblCustomReach = uilabel(pnlL,'Text','Custom reach (mm)','Position',[10 596 120 20], ...
        'Visible','off');
    efCustomReach = uieditfield(pnlL,'numeric','Position',[140 596 90 22], ...
        'Value',1100,'Visible','off','ValueChangedFcn',@(~,~)refreshPreview());

    % Environment
    uilabel(pnlL,'Text','Environment','Position',[10 566 300 22],'FontWeight','bold');
    uilabel(pnlL,'Text','Table Z (mm)','Position',[10 544 120 22]);
    efZTable = uieditfield(pnlL,'numeric','Position',[140 544 90 22], ...
        'Value',S.z_table,'ValueChangedFcn',@(~,~)refreshPreview());
    uilabel(pnlL,'Text','Clearance (mm)','Position',[10 522 120 22]);
    efZClear = uieditfield(pnlL,'numeric','Position',[140 522 90 22], ...
        'Value',S.z_clearance,'ValueChangedFcn',@(~,~)refreshPreview());

    % Motion
    uilabel(pnlL,'Text','Motion','Position',[10 492 300 22],'FontWeight','bold');
    uilabel(pnlL,'Text','Points/segment','Position',[10 470 120 22]);
    efPtsSeg = uieditfield(pnlL,'numeric','Position',[140 470 90 22], ...
        'Value',40,'Limits',[10 200],'RoundFractionalValues','on');

    uilabel(pnlL,'Text','Control dt (s)','Position',[10 448 120 22]);
    efDT = uieditfield(pnlL,'numeric','Position',[140 448 90 22], ...
        'Value',0.03,'Limits',[0.01 0.1]);

    uilabel(pnlL,'Text','Trigger pulse (ms)','Position',[10 426 120 22]);
    efPulseMs = uieditfield(pnlL,'numeric','Position',[140 426 90 22], ...
        'Value',12,'Limits',[3 50],'RoundFractionalValues','on');

    % ===== MAGNETIC END EFFECTOR SECTION (NEW) =====
    uilabel(pnlL,'Text','Magnetic End Effector','Position',[10 396 300 22],'FontWeight','bold');

    cbMagneticEnable = uicheckbox(pnlL,'Text','Enable magnetic control ($OUT[25])', ...
        'Position',[10 374 250 22],'Value',true, ...
        'ValueChangedFcn',@onMagneticEnableChanged);

    uilabel(pnlL,'Text','Engage delay (s)','Position',[10 352 100 22]);
    efEngageDelay = uieditfield(pnlL,'numeric','Position',[140 352 90 22], ...
        'Value',S.magnetic_engage_delay,'Limits',[0.1 5.0]);

    uilabel(pnlL,'Text','Release delay (s)','Position',[10 330 120 22]);
    efReleaseDelay = uieditfield(pnlL,'numeric','Position',[140 330 90 22], ...
        'Value',S.magnetic_release_delay,'Limits',[0.1 5.0]);

    btnTestMagnetic = uibutton(pnlL,'push','Text','Test Magnetic','Position',[250 352 150 28], ...
        'BackgroundColor',[0.4 0.7 0.9],'FontColor','w','FontWeight','bold', ...
        'Enable','off','ButtonPushedFcn',@(~,~)doTestMagnetic());

    lblMagneticStatus = uilabel(pnlL,'Text','MAGNET OFF', ...
        'Position',[250 330 150 22],'FontWeight','bold','FontColor',[0.7 0 0], ...
        'HorizontalAlignment','center');

    % Connection
    uilabel(pnlL,'Text','Connection','Position',[10 300 300 22],'FontWeight','bold');
    uilabel(pnlL,'Text','IP','Position',[10 278 40 22]);
    efIP = uieditfield(pnlL,'text','Position',[50 278 160 22],'Value',S.robot_ip);
    uilabel(pnlL,'Text','Port','Position',[220 278 40 22]);
    efPort = uieditfield(pnlL,'numeric','Position',[265 278 80 22],'Value',S.port);

    btnConnect = uibutton(pnlL,'push','Text','Connect','Position',[10 246 190 28], ...
        'BackgroundColor',[0.2 0.7 0.3],'FontColor','w','FontWeight','bold', ...
        'ButtonPushedFcn',@onConnect);

    btnDisconnect = uibutton(pnlL,'push','Text','Disconnect','Position',[210 246 190 28], ...
        'BackgroundColor',[0.75 0.2 0.2],'FontColor','w','FontWeight','bold', ...
        'Enable','off','ButtonPushedFcn',@(~,~)doDisconnect());

    lblConn = uilabel(pnlL,'Text','Status: Disconnected','Position',[10 224 390 20], ...
        'FontColor',[0.7 0 0]);

    % Poses
    uilabel(pnlL,'Text','Home (X Y Z A B C)','Position',[10 196 300 20], ...
        'FontWeight','bold','FontColor',[0.5 0 0.5]);
    efHome = poseFields(pnlL,174,S.home,@(~,~)refreshPreview());

    uilabel(pnlL,'Text','Pick (X Y Z A B C)','Position',[10 148 300 20], ...
        'FontWeight','bold','FontColor',[0 0.6 0]);
    efPick = poseFields(pnlL,126,S.pick,@(~,~)refreshPreview());

    uilabel(pnlL,'Text','Place (X Y Z A B C)','Position',[10 100 300 20], ...
        'FontWeight','bold','FontColor',[0.8 0 0]);
    efPlace = poseFields(pnlL,78,S.place,@(~,~)refreshPreview());

    % Actions
    btnPreview = uibutton(pnlL,'push','Text','Preview Trajectory','Position',[10 42 390 28], ...
        'BackgroundColor',[0.2 0.5 0.9],'FontColor','w','FontWeight','bold', ...
        'ButtonPushedFcn',@(~,~)doPreview());

    btnExecute = uibutton(pnlL,'push','Text','Execute on Robot','Position',[10 10 190 28], ...
        'BackgroundColor',[0.95 0.55 0.1],'FontColor','w','FontWeight','bold', ...
        'Enable','off','ButtonPushedFcn',@(~,~)doExecute());

    btnStop = uibutton(pnlL,'push','Text','STOP','Position',[210 10 190 28], ...
        'BackgroundColor',[0.85 0.1 0.1],'FontColor','w','FontSize',14,'FontWeight','bold', ...
        'Enable','off','ButtonPushedFcn',@(~,~)setStop());

    %% ===== CENTER: 3D PREVIEW =====
    pnlC = uipanel(fig,'Title','3D Preview','FontWeight','bold', ...
        'Position',[440 210 640 480]);
    ax3d = uiaxes(pnlC,'Position',[10 10 620 440]);
    hold(ax3d,'on'); grid(ax3d,'on'); axis(ax3d,'equal');
    xlabel(ax3d,'X (mm)'); ylabel(ax3d,'Y (mm)'); zlabel(ax3d,'Z (mm)');
    view(ax3d,45,25);

    %% ===== CENTER BOTTOM: LOG =====
    pnlLog = uipanel(fig,'Title','Log','FontWeight','bold', ...
        'Position',[440 10 640 192]);
    taLog = uitextarea(pnlLog,'Position',[5 5 630 165],'Editable','off', ...
        'FontName','Courier New','FontSize',10);

    %% ===== RIGHT: REAL-TIME TELEMETRY =====
    pnlR = uipanel(fig,'Title','Real-Time Telemetry','FontWeight','bold', ...
        'Position',[1090 10 650 680]);

    axTorque = uiaxes(pnlR,'Position',[10 510 630 145]); grid(axTorque,'on');
    title(axTorque,'Joint Torque'); xlabel(axTorque,'Step'); ylabel(axTorque,'N\cdotm');

    axCartesian = uiaxes(pnlR,'Position',[10 350 630 145]); grid(axCartesian,'on');
    title(axCartesian,'Cartesian Position'); xlabel(axCartesian,'Step'); ylabel(axCartesian,'mm');

    axVelocity = uiaxes(pnlR,'Position',[10 190 630 145]); grid(axVelocity,'on');
    title(axVelocity,'Joint Velocity'); xlabel(axVelocity,'Step'); ylabel(axVelocity,'deg/s');

    axCurrent = uiaxes(pnlR,'Position',[10 25 630 145]); grid(axCurrent,'on');
    title(axCurrent,'Joint Current'); xlabel(axCurrent,'Step'); ylabel(axCurrent,'A');

    %% ===== INIT DRAW =====
    refreshPreview();
    logMsg('Ready. Set Home/Pick/Place -> Preview -> Connect -> Execute.');
    logMsg('Magnetic control writes magnetic_cmd, and KRL mirrors it to $OUT[25].');

    %% =====================================================================
    %% CALLBACKS / ACTIONS
    %% =====================================================================

    function onModelChanged(~,~)
        v = ddModel.Value;
        if contains(v,'900')
            S.max_reach = 900; S.robot_model = 'KR6_R900';
            lblCustomReach.Visible = 'off'; efCustomReach.Visible = 'off';
        elseif contains(v,'1100')
            S.max_reach = 1100; S.robot_model = 'KR10_R1100';
            lblCustomReach.Visible = 'off'; efCustomReach.Visible = 'off';
        elseif contains(v,'1610')
            S.max_reach = 1600; S.robot_model = 'KR16_R1610';
            lblCustomReach.Visible = 'off'; efCustomReach.Visible = 'off';
        else
            lblCustomReach.Visible = 'on'; efCustomReach.Visible = 'on';
            S.max_reach = efCustomReach.Value; S.robot_model = 'Custom';
        end
        lblReach.Text = sprintf('Reach: %.0f mm', S.max_reach);
        refreshPreview();
        logMsg(['Robot model set: ' S.robot_model]);
    end

    function onMagneticEnableChanged(~,~)
        S.magnetic_enabled = cbMagneticEnable.Value;
        efEngageDelay.Enable = S.magnetic_enabled;
        efReleaseDelay.Enable = S.magnetic_enabled;
        if S.magnetic_enabled
            logMsg('Magnetic end effector control ENABLED ($OUT[25]).');
        else
            logMsg('Magnetic end effector control DISABLED.');
        end
    end

    function onConnect(~,~)
        if S.connected, return; end
        S.robot_ip = efIP.Value;
        S.port = efPort.Value;
        logMsg(sprintf('Connecting to %s:%d ...', S.robot_ip, S.port));
        try
            S.tcp = tcpclient(S.robot_ip, S.port, 'Timeout', 10);
            S.connected = true;

            lblConn.Text = ['Status: Connected to ' S.robot_ip];
            lblConn.FontColor = [0 0.6 0];
            btnConnect.Enable = 'off';
            btnDisconnect.Enable = 'on';
            btnExecute.Enable = 'on';
            btnTestMagnetic.Enable = 'on';

            % Ensure magnetic OFF on connect
            setMagnetic(false);

            [S.robot_s,S.robot_t] = readSTFromPosAct(S.tcp,S.robot_s,S.robot_t);
            logMsg(sprintf('Connected. S=%d, T=%d. Magnet OFF (safe start).', S.robot_s, S.robot_t));
        catch ME
            logMsg(['Connect FAILED: ' ME.message]);
            uialert(fig, ME.message, 'Connect Failed');
        end
    end

    function doDisconnect()
        if ~S.connected, return; end
        % Safety: magnetic OFF before disconnect
        try
            setMagnetic(false);
            logMsg('Magnet OFF (disconnect safety).');
        catch
        end
        try
            if ~isempty(S.tcp) && isvalid(S.tcp)
                clear S.tcp;
            end
        catch
        end
        S.tcp = [];
        S.connected = false;
        lblConn.Text = 'Status: Disconnected';
        lblConn.FontColor = [0.7 0 0];
        btnConnect.Enable = 'on';
        btnDisconnect.Enable = 'off';
        btnExecute.Enable = 'off';
        btnTestMagnetic.Enable = 'off';
        logMsg('Disconnected.');
    end

    function doTestMagnetic()
        if ~S.connected
            uialert(fig,'Not connected to robot.','Error'); return;
        end
        logMsg('Testing magnetic: magnetic_cmd = TRUE ...');
        setMagnetic(true);
        pause(2.0);
        setMagnetic(false);
        logMsg('Testing magnetic: magnetic_cmd = FALSE. Test complete.');
    end

    function doPreview()
        readUIToState();
        buildTrajectory();
        drawTrajectory();
        logMsg(sprintf('Preview: key_poses=%d, waypoints=%d, magnetic=%s', ...
            size(S.key_poses,1), size(S.waypoints,1), mat2str(S.magnetic_enabled)));
    end

    function doExecute()
        if ~S.connected
            uialert(fig,'Not connected to robot.','Error'); return;
        end

        readUIToState();
        buildTrajectory();
        drawTrajectory();

        magStr = 'DISABLED';
        if S.magnetic_enabled, magStr = sprintf('ENABLED (engage=%.1fs, release=%.1fs)', ...
                S.magnetic_engage_delay, S.magnetic_release_delay); end

        answ = uiconfirm(fig, ...
            sprintf(['Execute on REAL robot?\n\n' ...
                     'Ensure:\n' ...
                     '- MatlabControl.src is RUNNING (not paused)\n' ...
                     '- Robot is in T1, deadman ready\n' ...
                     '- Workspace clear\n' ...
                     '- Magnetic end effector power ON\n' ...
                     '- Object at PICK location\n\n' ...
                     'Waypoints: %d\nPulse: %d ms\nDT: %.3f s\n' ...
                     'Magnetic: %s'], ...
                     size(S.waypoints,1), round(efPulseMs.Value), efDT.Value, magStr), ...
            'Confirm', 'Options',{'EXECUTE','Cancel'}, 'DefaultOption','Cancel', ...
            'Icon','warning');

        if ~strcmp(answ,'EXECUTE')
            logMsg('Execution cancelled.');
            return;
        end

        runStreaming();
    end

    function setStop()
        S.stopRequested = true;
        % Immediate magnetic OFF on stop
        if S.connected
            try
                setMagnetic(false);
            catch
            end
        end
        logMsg('STOP requested. Magnet OFF.');
    end

    function onClose(~,~)
        S.stopRequested = true;
        if S.connected, doDisconnect(); end
        delete(fig);
    end

    %% =====================================================================
    %% MAGNET CONTROL - Direct $OUT[25] via KUKAVARPROXY (NEW)
    %% =====================================================================

    function setMagnetic(on)
        % Write a simple BOOL first, then let KRL mirror it to $OUT[25].
        cmdText = boolToKRL(on);
        ok = writeVar(S.tcp, 'magnetic_cmd', cmdText);
        if ~ok
            error('Failed to write magnetic_cmd via KUKAVARPROXY.');
        end

        % Give the KRL loop a moment to mirror magnetic_cmd to the output.
        pause(0.03);

        if on
            S.magnetic_is_on = true;
            lblMagneticStatus.Text = 'MAGNET ON';
            lblMagneticStatus.FontColor = [0 0.7 0];
        else
            S.magnetic_is_on = false;
            lblMagneticStatus.Text = 'MAGNET OFF';
            lblMagneticStatus.FontColor = [0.7 0 0];
        end

        cmdEcho = readBoolVar(S.tcp, 'magnetic_cmd');
        outEcho = readBoolVar(S.tcp, '$OUT[25]');

        if ~isempty(cmdEcho) && cmdEcho ~= on
            logMsg(sprintf('Warning: magnetic_cmd readback=%s, expected=%s.', ...
                boolToKRL(cmdEcho), cmdText));
        end

        if ~isempty(outEcho) && outEcho ~= on
            logMsg(sprintf(['Warning: $OUT[25] readback=%s, expected=%s. ' ...
                            'Check IOSYS mapping, SYS reservation, or PLC overwrite.'], ...
                boolToKRL(outEcho), cmdText));
        end
    end

    function setMagneticLegacy(on)
        % Writes $OUT[25] directly via KUKAVARPROXY - no KRL changes needed
        if on
            writeVar(S.tcp, '$OUT[25]', 'TRUE');
            S.magnetic_is_on = true;
            lblMagneticStatus.Text = 'MAGNET ON';
            lblMagneticStatus.FontColor = [0 0.7 0];
        else
            writeVar(S.tcp, '$OUT[25]', 'FALSE');
            S.magnetic_is_on = false;
            lblMagneticStatus.Text = 'MAGNET OFF';
            lblMagneticStatus.FontColor = [0.7 0 0];
        end
    end

    %% =====================================================================
    %% STATE / TRAJECTORY (UNCHANGED from your working version)
    %% =====================================================================

    function readUIToState()
        S.z_table = efZTable.Value;
        S.z_clearance = efZClear.Value;
        S.home  = readPose(efHome);
        S.pick  = readPose(efPick);
        S.place = readPose(efPlace);
        S.magnetic_enabled = cbMagneticEnable.Value;
        S.magnetic_engage_delay = efEngageDelay.Value;
        S.magnetic_release_delay = efReleaseDelay.Value;
        if efCustomReach.Visible
            S.max_reach = efCustomReach.Value;
            lblReach.Text = sprintf('Reach: %.0f mm', S.max_reach);
        end
    end

    function buildTrajectory()
        H = S.home; Pk = S.pick; Pl = S.place;
        Zc = S.z_clearance;

        TRANS_X = (Pk(1)+Pl(1))/2;
        TRANS_Y = (Pk(2)+Pl(2))/2;
        TRANS_Z = max(Pk(3), Pl(3)) + Zc;
        MID_Y   = (Pk(2) + TRANS_Y)/2;

        C_TARGET = -179.90;

        ORIENT_HOME           = safeOrientation(H(4:6),   C_TARGET);
        ORIENT_APPROACH_PICK  = safeOrientation([Pk(4), Pk(5), Pk(6)+0.3], C_TARGET);
        ORIENT_PICK           = safeOrientation([Pk(4), Pk(5), Pk(6)+0.1], C_TARGET);
        ORIENT_LIFT_PICK      = safeOrientation([Pk(4), Pk(5), Pk(6)+0.2], C_TARGET);
        ORIENT_MID            = safeOrientation([Pk(4), Pk(5), Pk(6)],     C_TARGET);
        ORIENT_TRANSITION     = safeOrientation([(Pk(4)+Pl(4))/2, (Pk(5)+Pl(5))/2, (Pk(6)+Pl(6))/2], C_TARGET);
        ORIENT_APPROACH_PLACE = safeOrientation([Pl(4), Pl(5), Pl(6)-0.3], C_TARGET);
        ORIENT_PLACE          = safeOrientation([Pl(4), Pl(5), Pl(6)-0.1], C_TARGET);
        ORIENT_LIFT_PLACE     = safeOrientation([Pl(4), Pl(5), Pl(6)-0.2], C_TARGET);

        % Key poses:
        %  1: HOME
        %  2: APPROACH PICK
        %  3: PICK              <- MAGNET ON here
        %  4: LIFT PICK
        %  5: MID
        %  6: TRANSITION
        %  7: APPROACH PLACE
        %  8: PLACE             <- MAGNET OFF here
        %  9: LIFT PLACE
        % 10: HOME
        S.key_poses = [
            H(1)  H(2)  H(3)           ORIENT_HOME;
            Pk(1) Pk(2) Pk(3)+Zc        ORIENT_APPROACH_PICK;
            Pk(1) Pk(2) Pk(3)           ORIENT_PICK;
            Pk(1) Pk(2) Pk(3)+Zc        ORIENT_LIFT_PICK;
            Pk(1) MID_Y Pk(3)+Zc        ORIENT_MID;
            TRANS_X TRANS_Y TRANS_Z     ORIENT_TRANSITION;
            Pl(1) Pl(2) Pl(3)+Zc        ORIENT_APPROACH_PLACE;
            Pl(1) Pl(2) Pl(3)           ORIENT_PLACE;
            Pl(1) Pl(2) Pl(3)+Zc        ORIENT_LIFT_PLACE;
            H(1)  H(2)  H(3)           ORIENT_HOME;
        ];

        ptsPerSeg = max(10, round(efPtsSeg.Value));

        W = [];
        for k = 1:(size(S.key_poses,1)-1)
            p0 = S.key_poses(k,:);
            p1 = S.key_poses(k+1,:);
            s = linspace(0,1,ptsPerSeg).';
            s5 = 6*s.^5 - 15*s.^4 + 10*s.^3;
            seg = p0 + (p1-p0).*s5;
            if isempty(W), W = seg; else, W = [W; seg(2:end,:)]; end %#ok<AGROW>
        end

        for wi = 1:size(W,1)
            W(wi,4:6) = safeOrientation(W(wi,4:6), C_TARGET);
        end

        S.waypoints = W;

        radii = sqrt(sum(S.waypoints(:,1:3).^2,2));
        if max(radii) > S.max_reach
            error('Some waypoints unreachable (outside reach). Reduce distances.');
        end
    end

    function refreshPreview()
        readUIToState();
        drawBase();
        drawPointsOnly();
    end

    function drawTrajectory()
        drawBase();
        drawPointsOnly();

        if isempty(S.waypoints), return; end

        plot3(ax3d, S.waypoints(:,1),S.waypoints(:,2),S.waypoints(:,3), ...
            'b-','LineWidth',2.0);
        scatter3(ax3d, S.waypoints(:,1),S.waypoints(:,2),S.waypoints(:,3), ...
            10, 1:size(S.waypoints,1), 'filled');
        colormap(ax3d, jet);
        drawnow;
    end

    function drawBase()
        cla(ax3d); hold(ax3d,'on'); grid(ax3d,'on'); axis(ax3d,'equal');
        xlabel(ax3d,'X (mm)'); ylabel(ax3d,'Y (mm)'); zlabel(ax3d,'Z (mm)');
        view(ax3d,45,25);

        [xs,ys,zs] = sphere(30);
        surf(ax3d, S.max_reach*xs, S.max_reach*ys, S.max_reach*zs, ...
            'FaceColor','cyan','FaceAlpha',0.06,'EdgeColor','none');

        quiver3(ax3d,0,0,0, 200,0,0,'r','LineWidth',2);
        quiver3(ax3d,0,0,0, 0,200,0,'g','LineWidth',2);
        quiver3(ax3d,0,0,0, 0,0,200,'b','LineWidth',2);

        plot3(ax3d,0,0,0,'ko','MarkerSize',12,'MarkerFaceColor','k');
    end

    function drawPointsOnly()
        plot3(ax3d, S.home(1),S.home(2),S.home(3), 'ms','MarkerSize',12,'MarkerFaceColor','m');
        plot3(ax3d, S.pick(1),S.pick(2),S.pick(3), 'go','MarkerSize',12,'MarkerFaceColor','g');
        plot3(ax3d, S.place(1),S.place(2),S.place(3),'ro','MarkerSize',12,'MarkerFaceColor','r');
        text(ax3d, S.home(1),S.home(2),S.home(3)+40,'HOME','Color','m','FontWeight','bold');
        text(ax3d, S.pick(1),S.pick(2),S.pick(3)+40,'PICK','Color',[0 0.6 0],'FontWeight','bold');
        text(ax3d, S.place(1),S.place(2),S.place(3)+40,'PLACE','Color','r','FontWeight','bold');
    end

    %% =====================================================================
    %% EXECUTION (STREAMING) WITH MAGNET CONTROL
    %% =====================================================================

    function runStreaming()
        if isempty(S.waypoints)
            uialert(fig,'No trajectory. Click Preview first.','Error');
            return;
        end

        S.running = true;
        S.stopRequested = false;
        btnExecute.Enable = 'off';
        btnPreview.Enable = 'off';
        btnStop.Enable = 'on';
        btnTestMagnetic.Enable = 'off';

        dt = efDT.Value;
        pulse_ms = round(efPulseMs.Value);
        ptsPerSeg = max(10, round(efPtsSeg.Value));

        % Calculate waypoint indices for PICK (key pose 3) and PLACE (key pose 8)
        PICK_WAYPOINT_IDX  = 1 + (3-1) * (ptsPerSeg - 1);
        PLACE_WAYPOINT_IDX = 1 + (8-1) * (ptsPerSeg - 1);

        logMsg(sprintf('EXEC start: N=%d, dt=%.3fs, pulse=%dms, magnetic=%s', ...
            size(S.waypoints,1), dt, pulse_ms, mat2str(S.magnetic_enabled)));
        if S.magnetic_enabled
            logMsg(sprintf('  Magnet ON at waypoint %d (PICK), OFF at %d (PLACE)', ...
                PICK_WAYPOINT_IDX, PLACE_WAYPOINT_IDX));
        end

        % Ensure magnetic OFF before starting
        setMagnetic(false);

        % Prepare real-time telemetry plots
        jointColors = {'r','g','b','m','c',[0.8 0.5 0]};
        jointLabels = {'J1','J2','J3','J4','J5','J6'};

        cla(axTorque); grid(axTorque,'on'); hold(axTorque,'on');
        title(axTorque,'Joint Torque');
        hTorque = gobjects(1,6);
        for jj = 1:6
            hTorque(jj) = animatedline(axTorque,'Color',jointColors{jj},'LineWidth',1.2);
        end
        legend(axTorque,jointLabels,'Location','bestoutside','FontSize',7);

        cla(axCartesian); grid(axCartesian,'on'); hold(axCartesian,'on');
        title(axCartesian,'Cartesian Position');
        hCartX = animatedline(axCartesian,'Color','r','LineWidth',1.5);
        hCartY = animatedline(axCartesian,'Color','g','LineWidth',1.5);
        hCartZ = animatedline(axCartesian,'Color','b','LineWidth',1.5);
        legend(axCartesian,{'X','Y','Z'},'Location','bestoutside','FontSize',7);

        cla(axVelocity); grid(axVelocity,'on'); hold(axVelocity,'on');
        title(axVelocity,'Joint Velocity');
        hVelocity = gobjects(1,6);
        for jj = 1:6
            hVelocity(jj) = animatedline(axVelocity,'Color',jointColors{jj},'LineWidth',1.2);
        end
        legend(axVelocity,jointLabels,'Location','bestoutside','FontSize',7);

        cla(axCurrent); grid(axCurrent,'on'); hold(axCurrent,'on');
        title(axCurrent,'Joint Current');
        hCurrent = gobjects(1,6);
        for jj = 1:6
            hCurrent(jj) = animatedline(axCurrent,'Color',jointColors{jj},'LineWidth',1.2);
        end
        legend(axCurrent,jointLabels,'Location','bestoutside','FontSize',7);

        % Move to first waypoint
        p0 = S.waypoints(1,:);
        ok = sendE6Pos(S.tcp, p0, S.robot_s, S.robot_t);
        if ~ok
            finishExec('Failed to write start target_pos.');
            return;
        end
        pulseMoveTrigger(S.tcp, pulse_ms);

        pause(1.2);

        [S.robot_s,S.robot_t] = readSTFromPosAct(S.tcp,S.robot_s,S.robot_t);
        logMsg(sprintf('Start command issued. Controller S=%d, T=%d', S.robot_s, S.robot_t));

        % Streaming loop
        N = size(S.waypoints,1);

        AXIS_POLL_RATE = max(1, round(0.30/dt));
        ST_REFRESH_RATE = max(1, round(0.50/dt));
        PLOT_RATE = max(1, round(0.10/dt));
        TELEM_POLL_RATE = max(1, round(0.15/dt));

        A4_GUARD = 170;
        A5_GUARD = 110;

        exec_sent = nan(N,6);
        a4_log = nan(N,1);
        a5_log = nan(N,1);
        torque_log = nan(N,6);
        cartesian_log = nan(N,3);
        velocity_log = nan(N,6);
        current_log = nan(N,6);
        magnetic_log = false(N,1);

        last_torque = nan(1,6);
        last_cartesian = nan(1,3);
        last_velocity = nan(1,6);
        last_current = nan(1,6);

        tStart = tic;

        for i = 1:N
            if S.stopRequested
                setMagnetic(false);
                finishExec('STOP pressed. Magnet OFF. Aborted.');
                return;
            end

            tStep = tic;

            pos = S.waypoints(i,:);

            ok = sendE6Pos(S.tcp, pos, S.robot_s, S.robot_t);
            if ok
                exec_sent(i,:) = pos;
            end
            pulseMoveTrigger(S.tcp, pulse_ms);

            % ===== MAGNET CONTROL AT KEY POSES (via $OUT[25]) =====
            if S.magnetic_enabled
                if i == PICK_WAYPOINT_IDX
                    logMsg(sprintf('  [%d] $OUT[25]=TRUE - engaging magnet...', i));
                    setMagnetic(true);
                    magneticWait(S.magnetic_engage_delay);
                    logMsg(sprintf('  Engage delay %.1fs complete.', S.magnetic_engage_delay));

                elseif i == PLACE_WAYPOINT_IDX
                    logMsg(sprintf('  [%d] $OUT[25]=FALSE - releasing...', i));
                    setMagnetic(false);
                    magneticWait(S.magnetic_release_delay);
                    logMsg(sprintf('  Release delay %.1fs complete.', S.magnetic_release_delay));
                end
            end
            magnetic_log(i) = S.magnetic_is_on;

            % Refresh S/T sometimes
            if mod(i, ST_REFRESH_RATE) == 0
                [sNew,tNew] = readSTFromPosAct(S.tcp,S.robot_s,S.robot_t);
                if sNew ~= S.robot_s || tNew ~= S.robot_t
                    logMsg(sprintf('S/T changed: S=%d->%d, T=%d->%d', S.robot_s, sNew, S.robot_t, tNew));
                    S.robot_s = sNew; S.robot_t = tNew;
                end
            end

            % Poll A4/A5 for guard
            if mod(i, AXIS_POLL_RATE) == 0 || i==1 || i==N
                [a4,a5] = readA4A5FromAxisAct(S.tcp);
                a4_log(i)=a4; a5_log(i)=a5;

                if ~isnan(a4) && abs(a4) > A4_GUARD
                    logMsg(sprintf('A4 %.1f exceeds guard -> ABORT + MAGNET OFF', a4));
                    setMagnetic(false);
                    finishExec('A4 guard exceeded. Magnet OFF. Aborted.');
                    saveExecLog(exec_sent,a4_log,a5_log,torque_log,cartesian_log,velocity_log,current_log,magnetic_log);
                    return;
                end
                if ~isnan(a5) && abs(a5) > A5_GUARD
                    logMsg(sprintf('A5 %.1f exceeds guard -> ABORT + MAGNET OFF', a5));
                    setMagnetic(false);
                    finishExec('A5 guard exceeded. Magnet OFF. Aborted.');
                    saveExecLog(exec_sent,a4_log,a5_log,torque_log,cartesian_log,velocity_log,current_log,magnetic_log);
                    return;
                end
            end

            % Poll telemetry
            if mod(i, TELEM_POLL_RATE)==0 || i==1 || i==N
                telem = readTelemetry(S.tcp);
                if any(~isnan(telem.torque)), last_torque = telem.torque; end
                torque_log(i,:) = telem.torque;
                if any(~isnan(telem.cartesian)), last_cartesian = telem.cartesian; end
                cartesian_log(i,:) = telem.cartesian;
                if any(~isnan(telem.velocity)), last_velocity = telem.velocity; end
                velocity_log(i,:) = telem.velocity;
                if any(~isnan(telem.current)), last_current = telem.current; end
                current_log(i,:) = telem.current;
            end

            % Plot updates
            if mod(i, PLOT_RATE)==0 || i==N
                for jj = 1:6
                    if ~isnan(last_torque(jj))
                        addpoints(hTorque(jj), i, last_torque(jj));
                    end
                end
                if ~isnan(last_cartesian(1))
                    addpoints(hCartX, i, last_cartesian(1));
                    addpoints(hCartY, i, last_cartesian(2));
                    addpoints(hCartZ, i, last_cartesian(3));
                end
                for jj = 1:6
                    if ~isnan(last_velocity(jj))
                        addpoints(hVelocity(jj), i, last_velocity(jj));
                    end
                end
                for jj = 1:6
                    if ~isnan(last_current(jj))
                        addpoints(hCurrent(jj), i, last_current(jj));
                    end
                end
                drawnow limitrate;
            end

            % Maintain dt
            while toc(tStep) < dt
                drawnow limitrate;
                if S.stopRequested, break; end
            end
        end

        % Ensure magnetic OFF after trajectory
        setMagnetic(false);

        elapsed = toc(tStart);
        saveExecLog(exec_sent,a4_log,a5_log,torque_log,cartesian_log,velocity_log,current_log,magnetic_log);
        finishExec(sprintf('EXEC completed. Time=%.1fs. Magnet OFF.', elapsed));
    end

    function magneticWait(seconds)
        t0 = tic;
        while toc(t0) < seconds
            drawnow limitrate;
            if S.stopRequested, break; end
        end
    end

    function saveExecLog(exec_sent,a4_log,a5_log,torque_log,cartesian_log,velocity_log,current_log,magnetic_log)
        timestamp = datestr(now,'yyyymmdd_HHMMSS');
        fname = sprintf('execution_log_gui_%s.mat', timestamp);
        execution_log = struct();
        execution_log.timestamp = timestamp;
        execution_log.robot_ip = S.robot_ip;
        execution_log.robot_model = S.robot_model;
        execution_log.key_poses = S.key_poses;
        execution_log.waypoints = S.waypoints;
        execution_log.executed_sent = exec_sent;
        execution_log.a4_log = a4_log;
        execution_log.a5_log = a5_log;
        execution_log.torque_log = torque_log;
        execution_log.cartesian_log = cartesian_log;
        execution_log.velocity_log = velocity_log;
        execution_log.current_log = current_log;
        execution_log.magnetic_log = magnetic_log;
        execution_log.magnetic_enabled = S.magnetic_enabled;
        execution_log.magnetic_engage_delay = S.magnetic_engage_delay;
        execution_log.magnetic_release_delay = S.magnetic_release_delay;
        execution_log.dt = efDT.Value;
        execution_log.pulse_ms = round(efPulseMs.Value);
        save(fname,'execution_log');
        logMsg(['Saved log: ' fname]);
    end

    function finishExec(msg)
        S.running = false;
        btnExecute.Enable = S.connected;
        btnPreview.Enable = 'on';
        btnStop.Enable = 'off';
        btnTestMagnetic.Enable = S.connected;
        logMsg(msg);
    end

    %% =====================================================================
    %% UI HELPERS
    %% =====================================================================

    function fields = poseFields(parent,y,vals,cb)
        fields = gobjects(1,6);
        x0 = 10; w = 62; h = 22; gap = 6;
        for n=1:6
            fields(n) = uieditfield(parent,'numeric','Position',[x0+(n-1)*(w+gap) y w h], ...
                'Value',vals(n),'ValueChangedFcn',cb);
        end
    end

    function p = readPose(fields)
        p = zeros(1,6);
        for n=1:6, p(n)=fields(n).Value; end
    end

    function logMsg(s)
        t = datestr(now,'HH:MM:SS');
        v = taLog.Value;
        if ischar(v), v = {v}; end
        v{end+1} = sprintf('[%s] %s', t, s);
        taLog.Value = v;
        scroll(taLog,'bottom');
    end

    %% =====================================================================
    %% ORIENTATION SAFETY (UNCHANGED - your proven working version)
    %% =====================================================================

    function abc = safeOrientation(abc_in, c_target)
        if nargin < 2, c_target = -179.90; end

        a = abc_in(1);
        b = abc_in(2);
        c = abc_in(3);

        a = mod(a + 180, 360) - 180;
        if a > 0, a = a - 360; end
        a = max(-180, a);
        if a > -175, a = -180; end
        if a <= -180, a = -179.99; end

        b = max(-3, min(3, b));

        c = mod(c + 180, 360) - 180;
        if abs(c) >= 180, c = c_target; end
        if abs(c - c_target) > 1.5
            c = c_target + sign(c - c_target) * 0.5;
        end

        abc = [a, b, c];
    end

    %% =====================================================================
    %% ROBOT COMMS (UNCHANGED - your proven working version)
    %% =====================================================================

    function ok = sendE6Pos(tcp, pose6, ~, ~)
        posStr = sprintf('{X %.3f,Y %.3f,Z %.3f,A %.3f,B %.3f,C %.3f}', ...
            pose6(1), pose6(2), pose6(3), pose6(4), pose6(5), pose6(6));
        ok = writeVar(tcp,'target_pos',posStr);
    end

    function pulseMoveTrigger(tcp, pulse_ms)
        writeVar(tcp,'move_trigger','TRUE');
        pause(max(0.003, pulse_ms/1000));
        writeVar(tcp,'move_trigger','FALSE');
    end

    function [Sval,Tval] = readSTFromPosAct(tcp,Sval,Tval)
        try
            pos_act_str = readVar(tcp,'$POS_ACT');
            if ischar(pos_act_str) || isstring(pos_act_str)
                pos_str = char(pos_act_str);
                s_match = regexp(pos_str,'[,\s]S\s+(\d+)','tokens');
                t_match = regexp(pos_str,'[,\s]T\s+(-?\d+)','tokens');
                if ~isempty(s_match), Sval = str2double(s_match{1}{1}); end
                if ~isempty(t_match), Tval = str2double(t_match{1}{1}); end
            end
        catch
        end
    end

    function [a4,a5] = readA4A5FromAxisAct(tcp)
        a4 = NaN; a5 = NaN;
        try
            axis_act = readVar(tcp,'$AXIS_ACT');
            if ischar(axis_act) || isstring(axis_act)
                axis_str = char(axis_act);
                a4_match = regexp(axis_str,'A4\s+([-\d.]+)','tokens');
                a5_match = regexp(axis_str,'A5\s+([-\d.]+)','tokens');
                if ~isempty(a4_match), a4 = str2double(a4_match{1}{1}); end
                if ~isempty(a5_match), a5 = str2double(a5_match{1}{1}); end
            end
        catch
        end
    end

    function telem = readTelemetry(tcp)
        telem = struct();
        telem.torque  = nan(1,6);
        telem.velocity = nan(1,6);
        telem.current  = nan(1,6);
        telem.cartesian = nan(1,3);

        torque_read = false;
        try
            for j = 1:6
                val = readVar(tcp, sprintf('$TORQUE_AXIS_ACT[%d]', j));
                if isnumeric(val) && ~isnan(val)
                    telem.torque(j) = val;
                    torque_read = true;
                end
            end
        catch
        end
        if ~torque_read
            try
                for j = 1:6
                    val = readVar(tcp, sprintf('$TORMON_DAT[%d].TORQUE_ACT', j));
                    if isnumeric(val) && ~isnan(val)
                        telem.torque(j) = val;
                    end
                end
            catch
            end
        end

        try
            for j = 1:6
                val = readVar(tcp, sprintf('$VEL_AXIS_ACT[%d]', j));
                if isnumeric(val) && ~isnan(val)
                    telem.velocity(j) = val;
                end
            end
        catch
        end

        try
            for j = 1:6
                val = readVar(tcp, sprintf('$CURR_ACT[%d]', j));
                if isnumeric(val) && ~isnan(val)
                    telem.current(j) = val;
                end
            end
        catch
        end

        try
            result = readVar(tcp, '$POS_ACT');
            if ischar(result) || isstring(result)
                s = char(result);
                labels = {'X','Y','Z'};
                for k = 1:3
                    tok = regexp(s, sprintf('%s\\s+([-\\d.eE]+)', labels{k}), 'tokens');
                    if ~isempty(tok)
                        telem.cartesian(k) = str2double(tok{1}{1});
                    end
                end
            end
        catch
        end
    end

    function text = boolToKRL(value)
        if value
            text = 'TRUE';
        else
            text = 'FALSE';
        end
    end

    function value = readBoolVar(tcp,varName)
        value = [];
        raw = readVar(tcp,varName);

        if islogical(raw)
            value = raw;
            return;
        end

        if isnumeric(raw) && ~isnan(raw)
            value = raw ~= 0;
            return;
        end

        if ischar(raw) || isstring(raw)
            rawText = upper(strtrim(char(raw)));
            if strcmp(rawText,'TRUE')
                value = true;
            elseif strcmp(rawText,'FALSE')
                value = false;
            end
        end
    end

    function success = writeVar(tcp,varName,varValue)
        success = false;
        try
            msgId = uint16(1);
            mode  = uint8(1);

            varNameBytes  = uint8(varName);
            varValueBytes = uint8(varValue);

            varNameLen  = uint16(length(varNameBytes));
            varValueLen = uint16(length(varValueBytes));

            contentLen = uint16(1 + 2 + length(varNameBytes) + 2 + length(varValueBytes));

            msgIdBytes       = typecast(swapbytes(msgId),      'uint8');
            contentLenBytes  = typecast(swapbytes(contentLen), 'uint8');
            varNameLenBytes  = typecast(swapbytes(varNameLen), 'uint8');
            varValueLenBytes = typecast(swapbytes(varValueLen),'uint8');

            msg = [msgIdBytes(:).' contentLenBytes(:).' mode ...
                   varNameLenBytes(:).' varNameBytes ...
                   varValueLenBytes(:).' varValueBytes];

            write(tcp,msg);

            pause(0.008);

            success = true;
            if tcp.NumBytesAvailable > 0
                response = read(tcp,tcp.NumBytesAvailable);
                if length(response) >= 3
                    tail = response(end-2:end);
                    success = all(tail == [0 1 1]);
                end
            end
        catch
            success = false;
        end
    end

    function value = readVar(tcp,varName)
        value = NaN;
        try
            msgId = uint16(1);
            mode  = uint8(0);

            varNameBytes = uint8(varName);
            varNameLen   = uint16(length(varNameBytes));
            contentLen = uint16(1 + 2 + length(varNameBytes));

            msgIdBytes      = typecast(swapbytes(msgId),      'uint8');
            contentLenBytes = typecast(swapbytes(contentLen), 'uint8');
            varNameLenBytes = typecast(swapbytes(varNameLen), 'uint8');

            msg = [msgIdBytes(:).' contentLenBytes(:).' mode ...
                   varNameLenBytes(:).' varNameBytes];

            write(tcp,msg);
            pause(0.01);

            if tcp.NumBytesAvailable > 0
                response = read(tcp,tcp.NumBytesAvailable);
                if length(response) > 7
                    valueLen = double(swapbytes(typecast(response(6:7),'uint16')));
                    if length(response) >= 7 + valueLen
                        valueBytes = response(8:(7+valueLen));
                        valueStr = char(valueBytes);
                        vnum = str2double(valueStr);
                        if isnan(vnum), value = valueStr; else, value = vnum; end
                    end
                end
            end
        catch
            value = NaN;
        end
    end

end
