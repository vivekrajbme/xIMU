function xIMU3Dashboard
    % xIMU3Dashboard — GUI to plot selected IMU streams and show Temp/Battery
    clc; close all;
    imu = imuStreamingApp.xIMU3(); pause(0.1); imu.flushBuffer();

    % Available streams: {Label, Packet ID, N_raw, inertialType, magType, ahrsType}
    specs = {
      'Quat',        hex2dec('D1'), 4, 0,0,0;   % raw quaternion (w,x,y,z)
      'Euler',       hex2dec('C1'), 3, 0,0,2;   % raw Euler angles
      % 'Rot3x3',      hex2dec('D2'), 9, 0,0,1;   % rotation matrix
      % 'GyroAccel',   hex2dec('C9'), 6, 1,0,0;   % gyro+accel
      % 'LinAcc',      hex2dec('CC'), 3, 2,0,0;   % linear accel
      % 'EarthAcc',    hex2dec('C5'), 3, 3,0,0;   % earth accel
      % 'HighG',       hex2dec('C8'), 3, 4,0,0;   % high-g accel
      'Mag',         hex2dec('CD'), 3, 0,1,0    % magnetometer
    };

    % Build UI
    fig      = figure('Name','xIMU3 Dashboard','NumberTitle','off', ...
                     'MenuBar','none','ToolBar','none','Position',[300 200 1000 600]);
    dd       = uicontrol(fig,'Style','popupmenu','String',specs(:,1), ...
                     'Units','normalized','Position',[.02 .85 .2 .1],'FontSize',12);
    startBtn = uicontrol(fig,'Style','pushbutton','String','Start','Units','normalized', ...
                     'Position',[.02 .7 .2 .1],'FontSize',12,'Callback',@startCB);
    stopBtn  = uicontrol(fig,'Style','pushbutton','String','Stop','Enable','off','Units','normalized', ...
                     'Position',[.02 .55 .2 .1],'FontSize',12,'Callback',@stopCB);
    ax       = axes(fig,'Units','normalized','Position',[.3 .2 .65 .75]); grid(ax,'on');
    xlabel(ax,'Time (s)');
    txtTemp  = uicontrol(fig,'Style','text','Units','normalized','Position',[.02 .4 .2 .05], ...
                     'FontSize',12,'String','Temp: -- °C');
    txtBat   = uicontrol(fig,'Style','text','Units','normalized','Position',[.02 .3 .2 .05], ...
                     'FontSize',12,'String','Bat: -- %');

    % State
    bufLen     = 400;
    maxPlotPts = 200;
    tBuf       = nan(1,bufLen);
    yBuf       = [];
    ptr        = 0;
    lastTemp   = nan;
    lastBat    = nan;
    lines      = gobjects(0);
    tmrPlot    = [];
    tmrLabels  = [];

    function startCB(~,~)
        sel = get(dd,'Value');
        [~, pid, Nraw, inT, magT, ahrsT] = specs{sel,:};
        % determine plot dimension
        if pid==hex2dec('D1')
            Nplot = 3; % drop w, show x,y,z
        else
            Nplot = Nraw;
        end
        % configure
        imu.configureStreams(inT, magT, ahrsT);
        writeline(imu.port,'{"temperature message type":1}');
        writeline(imu.port,'{"temperature message rate divisor":1}');
        writeline(imu.port,'{"battery message type":1}');
        writeline(imu.port,'{"battery message rate divisor":1}');
        writeline(imu.port,'{"rssi message type":1}');
        writeline(imu.port,'{"rssi message rate divisor":1}');
        writeline(imu.port,'{"apply":null}');
        pause(0.05); imu.flushBuffer();

        % reset
        tBuf(:)=nan; yBuf = nan(Nplot,bufLen); ptr=0;
        cla(ax); hold(ax,'on'); lines=gobjects(1,Nplot);
        for k=1:Nplot, lines(k)=plot(ax,nan,nan,'LineWidth',1); end

        % callback
        configureCallback(imu.port,'terminator',@(~,~)onPacket(pid,Nraw,Nplot));
        tmrPlot = timer('ExecutionMode','fixedRate','Period',0.05,'TimerFcn',@updatePlot);
        start(tmrPlot);
        tmrLabels = timer('ExecutionMode','fixedRate','Period',30,'TimerFcn',@updateLabels);
        start(tmrLabels);

        set(startBtn,'Enable','off'); set(stopBtn,'Enable','on');
        title(ax,specs{sel,1});
    end

    function stopCB(~,~)
        configureCallback(imu.port,'off');
        if isvalid(tmrPlot), stop(tmrPlot); delete(tmrPlot); end
        if isvalid(tmrLabels), stop(tmrLabels); delete(tmrLabels); end
        set(stopBtn,'Enable','off'); set(startBtn,'Enable','on');
    end

    function onPacket(pid,Nraw,Nplot)
        pkt = imu.readMessage(); if numel(pkt)<9, return; end
        id  = pkt(1); ts = double(typecast(pkt(2:9),'uint64'))/1e6;
        pl  = pkt(10:end);
        switch id
            case hex2dec('D4'), tmp = typecast(pl,'single'); lastTemp=tmp(1);
            case hex2dec('C2'), bv = typecast(pl,'single'); lastBat=bv(1);
            case pid
                if numel(pl)==4*Nraw
                    d_raw = typecast(pl,'single');
                    if pid==hex2dec('D1')
                        % drop first (w)
                        d = d_raw(2:4);
                    else
                        d = d_raw;
                    end
                    ptr=ptr+1; if ptr>bufLen, ptr=1; end
                    tBuf(ptr)=ts; yBuf(:,ptr)=d;
                end
        end
    end

    function updatePlot(~,~)
        if ptr==0, return; end
        full = all(~isnan(tBuf));
        if ~full, idx=1:ptr;
        else oldest=mod(ptr,bufLen)+1; idx=mod(oldest-1+(0:bufLen-1),bufLen)+1; end
        tP = tBuf(idx); yP = yBuf(:,idx);
        nPts = numel(tP);
        if nPts>maxPlotPts, selIdx=round(linspace(1,nPts,maxPlotPts)); tP=tP(selIdx); yP=yP(:,selIdx); end
        for k=1:numel(lines), lines(k).XData=tP; lines(k).YData=yP(k,:); end
        xlim(ax,[tP(end)-5 tP(end)]);
        drawnow limitrate;
    end

    function updateLabels(~,~)
        if ~isnan(lastTemp), set(txtTemp,'String',sprintf('Temp: %.2f°C',lastTemp)); end
        if ~isnan(lastBat),  set(txtBat,'String',sprintf('Bat:  %.1f%%',lastBat)); end
    end
end
