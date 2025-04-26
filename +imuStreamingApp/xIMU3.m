classdef xIMU3 < handle
    % xIMU3 MATLAB API for x-IMU3 quaternion & sensor streaming
    properties
        port           % serialport handle for JSON I/O%%
    end
    properties (Access=private)
        buffer uint8 = uint8.empty(0,1)
        ESC     = uint8(219)
        ESC_END = uint8(220)
        ESC_ESC = uint8(221)
        LF      = uint8(10)
    end
    methods
        function obj = xIMU3(portName, baud)
            % Constructor: auto-detect port if needed and configure IMU
            if nargin<1 || isempty(portName)
                ports = serialportlist("available");
                if isempty(ports)
                    error("No serial ports available. Connect x-IMU3 and retry.");
                end
                portName = ports(1);
            end
            if nargin<2 || isempty(baud)
                baud = 115200;
            end
            obj.port = serialport(char(portName), baud, 'Timeout',0.05);
            % Default config: binary mode + quaternion AHRS only
            cmds = { ...
                '{"binary mode enabled":true}', ...
                '{"inertial message type":0}',     '{"inertial message rate divisor":1}', ...
                '{"magnetometer message type":0}', '{"magnetometer message rate divisor":1}', ...
                '{"ahrs message type":0}',         '{"ahrs message rate divisor":1}', ...
                '{"temperature message type":0}',  '{"temperature message rate divisor":1}', ...
                '{"battery message type":0}',      '{"battery message rate divisor":1}', ...
                '{"rssi message type":0}',         '{"rssi message rate divisor":1}', ...
                '{"apply":null}'
                };
            for i = 1:numel(cmds)
                writeline(obj.port, cmds{i});
            end
            pause(0.1);
            obj.flushBuffer();
        end
        function flushBuffer(obj)
            % Clear hardware FIFO and internal buffer
            flush(obj.port);
            obj.buffer = uint8.empty(0,1);
        end
        function data = readMessage(obj)
            % READMESSAGE Reads one SLIP-framed packet as raw bytes
            while true
                n = obj.port.NumBytesAvailable;
                if n>0
                    obj.buffer(end+1:end+n,1) = read(obj.port, n, 'uint8');
                else
                    pause(0.001);
                end
                idx = find(obj.buffer==obj.LF, 1);
                if isempty(idx), continue; end
                pkt = obj.buffer(1:idx);
                obj.buffer(1:idx) = [];
                raw = pkt(1:end-1);
                % SLIP destuff
                dest = zeros(numel(raw),1,'uint8'); i=1; j=1;
                while i<=numel(raw)
                    b = raw(i);
                    if b==obj.ESC && i<numel(raw)
                        c = raw(i+1);
                        if     c==obj.ESC_END, dest(j)=obj.LF;
                        elseif c==obj.ESC_ESC, dest(j)=obj.ESC;
                        else                 dest(j)=c;
                        end
                        i = i+2;
                    else
                        dest(j)=b; i=i+1;
                    end
                    j=j+1;
                end
                data = dest(1:j-1);
                return;
            end
        end
        function varargout = parse(obj)
            pkt  = obj.readMessage();
            id   = pkt(1);
            ts   = double(typecast(pkt(2:9),'uint64'));
            pl   = pkt(10:end);
            switch id
                case hex2dec('D1')             % Quaternion
                    q = typecast(pl,'single');
                    varargout = {ts, q'};
                case hex2dec('C1')             % Euler angles
                    e = typecast(pl,'single');
                    varargout = {ts, e'};
                case hex2dec('C9')             % Inertial (gyro+accel)
                    ia = typecast(pl,'single');
                    varargout = {ts, ia'};
                case hex2dec('CD')             % Magnetometer
                    m = typecast(pl,'single');
                    varargout = {ts, m'};
                case hex2dec('D2')             % Rotation matrix
                    R = reshape(typecast(pl,'single'),3,3)';
                    varargout = {ts, R};
                case hex2dec('CC')             % Linear acceleration
                    la = typecast(pl,'single');
                    varargout = {ts, la'};
                case hex2dec('C5')             % Earth acceleration
                    ea = typecast(pl,'single');
                    varargout = {ts, ea'};
                case hex2dec('C8')             % High-g accelerometer
                    hg = typecast(pl,'single');
                    varargout = {ts, hg'};
                case hex2dec('D4')             % Temperature
                    tmp = typecast(pl,'single');
                    varargout = {ts, tmp};
                case hex2dec('C2')             % Battery
                    bv = typecast(pl,'single');
                    varargout = {ts, bv};
                case hex2dec('D7')             % RSSI
                    rssi = typecast(pl,'single');
                    varargout = {ts, rssi};
                otherwise                     % Unhandled or notification/error
                    varargout = {ts, []};
            end
        end

        % Convenience wrappers
        function v=readQuaternion(obj),    v = obj.parse{1}; end
        function v=readEuler(obj),         v = obj.parse{1}; end
        function v=readInertial(obj),      v = obj.parse{1}; end
        function v=readMagnetometer(obj),  v = obj.parse{1}; end
        function M=readRotation(obj),      M = reshape(v,3,3); end
        function v=readLinearAccel(obj),   v = obj.parse{1}; end
        function v=readEarthAccel(obj),    v = obj.parse{1}; end
        function delete(obj)
            % Destructor: ensure port closed
            if isa(obj.port,'serialport')
                clear obj.port;
            end
           
        end
        function configureStreams(obj, inertialType, magType, ahrsType)
            % inertialType: 0=none,1=gyro+accel,2=lin-accel,3=earth-accel,4=high-g
            % magType:      0=none,1=magnetometer
            % ahrsType:     0=none,1=quaternion,2=Euler,3=rotation matrix

            cmds = { ...
                '{"binary mode enabled":true}', ...
                sprintf('{"inertial message type":%d}',           inertialType), ...
                '{"inertial message rate divisor":20}', ...
                sprintf('{"magnetometer message type":%d}',        magType), ...
                '{"magnetometer message rate divisor":1}', ...
                sprintf('{"ahrs message type":%d}',                ahrsType), ...
                '{"ahrs message rate divisor":20}', ...
                '{"temperature message type":1}',                  ...
                '{"temperature message rate divisor":1}', ...
                '{"battery message type":1}',                      ...
                '{"battery message rate divisor":1}', ...
                '{"rssi message type":1}',                         ...
                '{"rssi message rate divisor":1}', ...
                '{"apply":null}' ...
                };
            for i = 1:numel(cmds)
                writeline(obj.port, cmds{i});
            end
        end
    end
end
