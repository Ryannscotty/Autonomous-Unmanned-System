# Autonomous-Unmanned-System
An open-source Unmanned Aircraft System (UAS) built around a custom FPV drone, ground control station (GCS), and RC controller — designed as an end-to-end embedded systems and RF project 
FPV Drone

Flight Controller

STM32H723-based, fusing IMU, barometer, magnetometer, and Hall effect sensor data with NEO-M9N GPS for state estimation and navigation
ESC Board for motor control

Video/Cam Board 

Basys 3 FPGA handling camera interfacing and video pipeline, paired with an RF transmitter (module or custom PCB) for the analog/digital FPV video link

LoRa RF Module

long-range telemetry uplink/downlink between the FC and GCS

Ground Control Station (GCS)

ADALM-Pluto SDR for capturing/demodulating LoRa I/Q samples
Jetson Nano as the central compute platform — running mission planning and waypoint management, telemetry display, onboard object detection for the FPV camera feed, and relaying manual RC commands

RC Controller

Joystick-based manual control input to the GCS/drone link
