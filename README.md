# xIMU3Mod – Minimal MATLAB TCP/Serial Interface for x-IMU3

A lightweight, single-file MATLAB class to connect to an x-IMU3 device via **serial** or **TCP** and read **Euler angles, quaternions, and inertial data**.

## Features
- **One class, no dependencies**
- Supports **Serial (`COMx`)** and **TCP (`192.168.x.x`)**
- Reads:
  - Euler angles (roll, pitch, yaw in degrees)
  - Quaternion (4×1)
  - Gyroscope + accelerometer
- Configurable AHRS/inertial streaming rates
- SLIP packet decoding and angle unwrapping

## Quick Usage

```matlab
% Create IMU object
imu = xIMU3Mod('tcp', '192.168.1.1', 7000);
% OR serial
% imu = xIMU3Mod('serial','COM5',115200);

% Read Euler angles (roll,pitch,yaw in deg)
angles = imu.readEuler();  

% Read quaternion
q = imu.readQuaternion();

% Read gyro & accel
[g,a] = imu.readInertial();

% Change AHRS config (0=Quaternion, 2=Euler)
imu.configureAHRS(2,1);

% Clean up
clear imu
