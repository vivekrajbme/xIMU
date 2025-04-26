# xIMUtoolBox

A lightweight MATLAB toolbox for real-time quaternion streaming and visualization from the x-IMU3 sensor.

---

## Overview

**xIMUtoolBox** provides:
- An easy-to-use `xIMU3` class for SLIP-framed binary decoding of quaternion packets.
- A helper `initIMU` function to configure your IMU in binary mode at 400 Hz.
- A simple script (`xIMU3Dashboard`) for live display and basic plotting.
- Full documentation via `Contents.m` and this `README.md`, ready for packaging as a `.mltbx`.

---

## Features

- **Automatic buffering & SLIP decoding** of quaternion packets (0xD1)  
- **Stream control**: disable inertial/mag packets to reduce noise  
- **High-rate output**: default 400 Hz quaternion updates  
- **App Designer UI**: choose COM port, start/stop streaming, and live-plot orientation  
- **Clean teardown**: calls `delete(imu)` to close ports and timers  

---

## Requirements

- MATLAB R2019b or later  
- Instrument Control Toolbox (for `serialport`)  
- x-IMU3 sensor connected via USB  

---

## Usage

    Connect your x‑IMU3 via USB/UART.
    
    Launch xIMU3Dashboard from the Apps tab or command line.
    
    Select a stream from the dropdown and click Start.
    
    View live data and status updates.

## Technical Details

    Communicates over serial using SLIP framing (per x‑IMU3 User Manual v1.11).
    
    Parses binary packets (IDs 0xC9, 0xD1, 0xC1, etc.) into MATLAB arrays.
    
    Configures message types and rates via JSON commands ({"ahrs message type":…}, etc.).

