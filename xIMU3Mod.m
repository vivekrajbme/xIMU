%% Author-Vivek Raj
%  Use imu=xIMU3Mod('tcp', '192.168.1.1',7000)
%  Use imu=xIMU3Mod('serial', 'COM5',115200)
%  Use imu.configureAHRS(0,1) where 0 is type of output(0-Quaternion,2-Euler,..)
%  you need and 1 is the sampling rate divisor.
classdef xIMU3Mod < handle
    properties (Access = private)
        port                % serialport or tcpclient handle
        connType            % 'serial' or 'tcp'
        buffer uint8 = uint8.empty(0,1)
        ESC     uint8 = uint8(219)
        ESC_END uint8 = uint8(220)
        ESC_ESC uint8 = uint8(221)
        LF      uint8 = uint8(10)
        lastEuler double = zeros(3,1)  % previous [roll;pitch;yaw] in radians
    end

    methods
        function obj = xIMU3Mod(connType, addr, portOrBaud)
            arguments
                connType {mustBeMember(connType, {'serial','tcp'})}
                addr
                portOrBaud
            end
            obj.connType = connType;
            obj.lastEuler = zeros(3,1);

            % Initialize port
            if strcmp(connType, 'serial')
                obj.port = serialport(char(addr), portOrBaud, "Timeout",0.05, "DataBits",8, "Parity","none", "StopBits",1);
            else
                obj.port = tcpclient(addr, portOrBaud, 'Timeout', 0.05);
            end

            % Default IMU configuration: binary + Euler @400Hz
            cmds = {
                '{"binary mode enabled":true}',...
                '{"inertial message rate divisor":1}',...
                '{"magnetometer message rate divisor":0}',...
                '{"ahrs message type":2}',...       % Euler
                '{"ahrs message rate divisor":1}',...% 400 Hz
                '{"apply":null}'...
            };
            for c = cmds, obj.sendLine(c{1}); end
            pause(0.1);
            obj.flushBuffer();
        end

        function varargout = readEuler(obj)
            % Read and unwrap Euler angles. Usage:
            %   q = obj.readEuler();    % 1x3 [roll,pitch,yaw]
            %   [r,p,y] = obj.readEuler();
            while true
                n = obj.port.NumBytesAvailable;
                if n>0
                    obj.buffer(end+1:end+n,1) = obj.readBytes(n);
                else
                    pause(0.001);% small yield to avoid 100% CPU while waiting for IMU data
                end
                idx = find(obj.buffer == obj.LF, 1);
                if isempty(idx), continue; end

                pkt = obj.buffer(1:idx);
                obj.buffer(1:idx) = [];
                raw = pkt(1:end-1);
                dest = zeros(numel(raw),1,'uint8'); i=1; j=1;
                while i <= numel(raw)
                    b = raw(i);
                    if b==obj.ESC && i<numel(raw)
                        c = raw(i+1);
                        if     c==obj.ESC_END, dest(j)=obj.LF;
                        elseif c==obj.ESC_ESC, dest(j)=obj.ESC;
                        else                   dest(j)=c;
                        end
                        i = i+2;
                    else
                        dest(j)=b;
                        i = i+1;
                    end
                    j=j+1;
                end
                dest = dest(1:j-1);

                % Euler header = 0xC1
                if numel(dest)>=21 && dest(1)==hex2dec('C1')
                    vDeg = typecast(dest(10:21),'single');     % [roll;pitch;yaw] in deg
                    curr = deg2rad(vDeg(:));                  % radians
                    delta = mod(curr - obj.lastEuler + pi, 2*pi) - pi;
                    unwrapped = obj.lastEuler + delta;
                    obj.lastEuler = unwrapped;
                    q = rad2deg(unwrapped)';                  % 1x3
                    if nargout<=1
                        varargout{1} = q;
                    else
                        varargout{1} = q(1);
                        varargout{2} = q(2);
                        varargout{3} = q(3);
                    end
                    return;
                end
            end
        end
        function [gyro, accel] = readInertial(obj)
            % Read binary inertial packet (gyro+accel) header C9
            while true
                n=obj.port.NumBytesAvailable;
                if n>0
                    rawBytes=obj.readBytes(n);
                    obj.buffer(end+1:end+n,1) = rawBytes(:);
                else
                    pause(0.001);% small yield to avoid 100% CPU while waiting for IMU data
                end
                idx = find(obj.buffer == obj.LF, 1);
                if isempty(idx), continue; end
                pkt = obj.buffer(1:idx);
                obj.buffer(1:idx) = [];
                raw = pkt(1:end-1);
                % SLIP unescape
                dest = zeros(numel(raw),1,'uint8'); i=1; j=1;
                while i<=numel(raw)
                    b = raw(i);
                    if b==obj.ESC && i<numel(raw)
                        c = raw(i+1);
                        if c==obj.ESC_END, dest(j)=obj.LF;
                        elseif c==obj.ESC_ESC, dest(j)=obj.ESC;
                        else 
                            dest(j)=c;
                        end
                        i = i+2;
                    else
                        dest(j)=b; i=i+1;
                    end
                    j=j+1;
                end
                dest = dest(1:j-1);
                % Binary inertial header = 0x80+'I'=0x49+0x80=0xC9
                if numel(dest)>=21 && dest(1)==hex2dec('C9')
                    v = typecast(dest(10:33),'single');  % [gx;gy;gz;ax;ay;az]
                    gyro  = v(1:3)';
                    accel = v(4:6)';
                    return;
                end
            end
        end
        function q = readQuaternion(obj)
            while true
                n = obj.port.NumBytesAvailable();
                if n > 0
                    data = obj.readBytes(n);
                    obj.buffer(end+1:end+n,1) = data;
                else
                    pause(0.001); % small yield to avoid 100% CPU while waiting for IMU data
                end

                idx = find(obj.buffer == obj.LF, 1);
                if isempty(idx), continue; end

                pkt = obj.buffer(1:idx);
                obj.buffer(1:idx) = [];

                raw = pkt(1:end-1);
                dest = zeros(numel(raw),1,'uint8'); i=1; j=1;
                while i <= numel(raw)
                    b = raw(i);
                    if b == obj.ESC && i < numel(raw)
                        c = raw(i+1);
                        if     c == obj.ESC_END, dest(j) = obj.LF;
                        elseif c == obj.ESC_ESC, dest(j) = obj.ESC;
                        else dest(j) = c;
                        end
                        i = i + 2;
                    else
                        dest(j) = b; i = i + 1;
                    end
                    j = j + 1;
                end
                dest = dest(1:j-1);

                if numel(dest) >= 25 && dest(1) == hex2dec('D1')
                    q = typecast(dest(10:25), 'single');
                    return;
                end
            end
        end
        function configureInertial(obj, rateDivisor)
            % Dynamically adjust inertial (gyro+accel) rate
            obj.sendLine(sprintf('{"inertial message rate divisor":%d}',rateDivisor));
            obj.sendLine('{"apply":null}'); pause(0.1); obj.flushBuffer();
        end
        function configureAHRS(obj, msgType, rateDivisor)
            obj.sendLine(sprintf('{"ahrs message type":%d}',       msgType));
            obj.sendLine(sprintf('{"ahrs message rate divisor":%d}',rateDivisor));
            obj.sendLine('{"apply":null}'); pause(0.1); obj.flushBuffer();
        end

        function flushBuffer(obj)
            % Clears serial or TCP buffer
            if strcmp(obj.connType,'serial')
                flush(obj.port);
            else
                while obj.port.NumBytesAvailable>0
                    read(obj.port, obj.port.NumBytesAvailable, 'uint8');
                end
            end
            obj.buffer = uint8.empty(0,1);
        end

        function delete(obj)
            try
                clear obj.port; 
            catch
            end
        end
    end

    methods (Access = private)
        function sendLine(obj, str)
            write(obj.port, uint8(char(string(str)+newline)), 'uint8');
        end
        function data = readBytes(obj, n)
            data = read(obj.port, n, 'uint8');
        end
    end
end
