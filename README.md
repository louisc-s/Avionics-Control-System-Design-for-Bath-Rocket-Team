# Bath-Rocket-Team-Flight-Computer-Code-
Arduino code for controlling sounding rocket during launch and flight 

## Overview

This is the code for the Bath Rocket Team group business design project. This project involved desinging a sounding rocket cable of reaching a 10,000ft apogee and deploying a payload. This code was developed for the flight computer that monitored and controlled all of the rocket's subsystems during the flight. The following tasks for each subsystem were carried out by this code:

Avionics - co-ordinate telemetry system and apply kalman filter to altimneter and IMU to remove noise from sensor data for more accurate telemetry  
Proppellant - operate flow control valve to control flow of propellant into combustion chamber and control dump valve in case of aborted launch to dump proppelant
Payload - open payload bay doors and deploy payload using linear actuators 
Recovery - ignite e-matches to deploy rogue parachute 

## Project Structure 

1. Functions.py - contains all the required functions to run the classifier model scripts and the Sort script

2. Sort.py - loads and segments the sEMG data from the Matlab files for each subject and produces three 
CSV files for each subject: training, validation and test (**code must be ammended with the accurate storage location
of Ninapro DB1 data for effective use**)


3. LDA.py - implements linear discriminant analysis classifier 

4. SVM.py - implements support vector machine classifier

5. 1DCNN.py - implements 1d convolutional neural network classifier

6. 2DCNN.py - implements 2d convoltuional neural network classifer

7. LSTM.py - implements long short-term memory network classifier

8. CNN_LSTM.py - impelemnts hybrid classifier consiting of both a 1d convolutional neural network and a long short-term memory network

## Author 

Louis Chapo-Saunders
