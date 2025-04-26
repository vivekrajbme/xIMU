function p = initIMU(portName,baud)
    if nargin<1, portName="COM8"; end; if nargin<2, baud=115200; end
    obj = imuStreamingApp.xIMU3(portName,baud);
    p = obj; % use obj.parse or read* methods on this object
end