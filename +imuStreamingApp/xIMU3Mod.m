classdef xIMU3Mod < handle
    properties (Access = private)
        port                 % serialport or tcpclient handle
        connType             % 'serial' or 'tcp'
        buffer uint8 = uint8.empty(0,1)
        ESC     uint8 = uint8(219)
        ESC_END uint8 = uint8(220)
        ESC_ESC uint8 = uint8(221)
        LF      uint8 = uint8(10)
    end

    methods
        function obj = xIMU3Mod(connType, addr, portOrBaud)
            arguments
                connType {mustBeMember(connType, {'serial','tcp'})}
                addr
                portOrBaud
            end

            obj.connType = connType;

            if strcmp(connType, 'serial')
                % Serial port initialization (USB or Bluetooth SPP)
                obj.port = serialport(char(addr), portOrBaud, ...
                    "Timeout",0.05, "DataBits",8, "Parity","none", "StopBits",1);
            elseif strcmp(connType, 'tcp')
                % TCP socket initialization (Wi-Fi module etc.)
                obj.port = tcpclient(addr, portOrBaud, 'Timeout', 0.05);
            end

            % IMU configuration (works only if the remote accepts these commands)
            cmds = {
                '{"binary mode enabled":true}', ...
                '{"inertial message rate divisor":0}', ...
                '{"magnetometer message rate divisor":0}', ...
                '{"ahrs message type":0}', ...
                '{"ahrs message rate divisor":1}', ...
                '{"apply":null}' ...
            };

            for c = cmds
                obj.sendLine(c{1});
            end

            pause(0.1);
            obj.flushBuffer();
        end

        function q = readQuaternion(obj)
            while true
                n = obj.getNumBytesAvailable();
                if n > 0
                    data = obj.readBytes(n);
                    obj.buffer(end+1:end+n,1) = data;
                else
                    pause(0.001);
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

        function flushBuffer(obj)
            if strcmp(obj.connType, 'serial')
                flush(obj.port);
            elseif strcmp(obj.connType, 'tcp')
                while obj.port.NumBytesAvailable > 0
                    read(obj.port, obj.port.NumBytesAvailable, 'uint8');
                end
            end
            obj.buffer = uint8.empty(0,1);
        end

        function delete(obj)
            try
                if strcmp(obj.connType, 'serial')
                    clear obj.port;
                elseif strcmp(obj.connType, 'tcp')
                    clear obj.port;
                end
            catch
                % Silent cleanup
            end
        end
    end

    methods (Access = private)
        function sendLine(obj, str)
            % Write a string followed by newline
            s = string(str) + newline;
            write(obj.port, uint8(char(s)), 'uint8');
        end

        function n = getNumBytesAvailable(obj)
            n = obj.port.NumBytesAvailable;
        end

        function data = readBytes(obj, n)
            data = read(obj.port, n, 'uint8');
        end
    end
end
