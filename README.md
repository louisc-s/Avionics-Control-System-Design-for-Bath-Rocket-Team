# Bath-Rocket-Team-Flight-Computer-Code-
Arduino code for controlling the avionics of a sounding rocket during launch and flight 

## Overview

This is the code for the Bath Rocket Team Group Business Design Project. This project involved desinging a sounding rocket cable of reaching a 10,000ft apogee and deploying a payload. This code was developed for the flight computer that monitored and controlled all of the rocket's subsystems during the flight. The code carried out the following tasks for each subsystem:

Avionics - co-ordinate telemetry system and apply kalman filter to altimeter and IMU to remove noise from sensor data for more accurate telemetry  
Proppellant - operate flow control valve to control flow of propellant into combustion chamber and control dump valve in case of aborted launch to dump proppelant
Payload - open payload bay doors and deploy payload using linear actuators 
Recovery - ignite e-matches to deploy rogue parachute 

## Project Structure 

1. Avionics_Code.ino - impelements control loop for all the avionics sub-system operations

2. Kalman_Filter_Code - implements combined kalman filter for IMU and altimeter in Matlab 

## Author 

Louis Chapo-Saunders
