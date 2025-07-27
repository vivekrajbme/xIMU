% xIMU3Mod Toolbox for MATLAB
% Version 1.1 (2025-07-25)
%
% Minimal MATLAB interface for the x-IMU3 sensor via Serial or TCP.
%
% Files:
%   xIMU3Mod.m    - Single class to connect, configure, and stream IMU data
%
% Quick Start:
%   imu = xIMU3Mod('tcp', '192.168.1.1',7000);
%   angles = imu.readEuler();
%
% Features:
%   - Supports TCP and Serial
%   - Reads Euler angles, Quaternion, Gyro & Accel
%   - Handles SLIP decoding & angle unwrapping
%
% Author:
%   Vivek Raj (RISE LAB, IIT Delhi, 2025)
%
% License:
%   MIT
%
% See also serialport, tcpclient
