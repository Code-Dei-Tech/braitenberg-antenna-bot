# Source Files for the Antenna-based Braitenberg Robot
------------------------------------------------------

Hans Verdolaga  
Developed as part of the MSc Mechatronics Master's Thesis 2023  
Department of Mechanical and Electrical Engineering  
University of Southern Denmark (SDU)

------------------------------------------------------

## Table of Contents
- [Thesis Overview and Abstract](#thesis-overview-and-abstract)
- [Robot Design](#robot-design)
- [File Descriptions](#file-descriptions)
- [References](#references)
- [Acknowledgements](#acknowledgements)
- [License](#license)

------------------------------------------------------

## Thesis Overview and Abstract
### Thesis Title
Online Learning of Gas Localization via Cockroach-inspired Antennae on a Mobile Robot

### Thesis Abstract
Gas source localization is an engineering challenge fulfilled by passive and active
perception. Passive perception involves modelling or visual information to search
for sources of gas plumes, but these are often complex and expensive. Active per-
ception is a promising alternative that aims to track plumes without prior map-
ping or modelling using biologically-inspired means. One of these techniques is the
Braitenberg-based chemotaxis, which directly uses sensor signals to drive a mobile
robot’s motors in response to a stimulus. Recent local studies that developed a cock-
roach antenna-inspired robot have modified this method to apply temporal plume
dynamics to a dynamic Braitenberg. However, they have only conducted idealized
simulations where the robot exhibits slow performance and immobilization exiting
the plume.

This thesis aims to implement, improve and test the antenna-based Braitenberg
algorithm for gas source localization on a physical robotic platform. It implements
a multi-processor robot design that interfaces with ethanol sensors, antenna servo
motors and DC motors, encompassed by a finite state machine. The project also
proposes localization improvements via online learning and plume exit recovery.
Experimental results show that the fully-assembled robot demonstrates poor plume
tracking performance primarily due to severe sensor limitations. Consequently, the
dynamic Braitenberg performs worse than the generic algorithm, and the robot
localizes best when the antennae are statically sideways. Sensor problems and time
constraints have prevented the implementation of online learning, but a periodic
genetic algorithm was proposed and had proved to increase the robot’s gas sensitivity
for better mobility. Plume exit recovery had also helped the robot prolong its source
seeking inside the plume.

------------------------------------------------------

## Robot Design: TODO

------------------------------------------------------

## File Descriptions
- [File Overview](#file-overview)
- [Robot source code](#robot-source-code)
- [Robot design files](#robot-design-files)
- [Control source code](#control-source-code)
- [Evaluation files](#evaluation-files)

### File Overview
The file structure of this repository is organized into source codes for the robot and user control, CAD files for the robot, and evaluation files documenting the robot's experimental results. 

### Robot source code
The robot source code is organized into the following folders:
- `robot-pi-files`: Python files for the Rock Pi 4C Plus microcontroller
- `robot-mtrbrd-files`: Arduino sketch for the robot's DC motor board
- `robot-snsbrd-files`: Arduino sketches for the robot's sensor and servo motor board

The robot's Pi files are further divided into two categories of Python files:
- Libraries and utilities
    - `robot-pi-files/hvArduinoReconfg.py`: Python function library for reconfguring the Arduinos
    - `robot-pi-files/hvClasses.py`: Python class library for the robot's various hardware classes
    - `robot-pi-files/hvStateMachine.py`: Python class library for the robot's finite state machine
    - `robot-pi-files/hvConfiguration.py`: Python parameter library for the robot's complete configuration parameters
    - `robot-pi-files/hvCalibrateSymmetry.py`: Script that reads the sensor data for symmetry calibration
- Main programs
    - `robot-pi-files/hvCommander.py`: Primary control loop program that manages the robot's finite state machine, data processing, logging and instructions to the Arduinos via serial communication
    - `robot-pi-files/hvMain.py`: Control loop program that monitors the MQTT broker for commands and executes the `robot-pi-files/hvCommander.py` program upon receiving the start command
    - `robot-pi-files/hvMeasureSymmetry.py`: Control loop program for measuring and logging left and right sensor data for symmetry calibration
    - `robot-pi-files/hvPIDTune.py`: Control loop program for tuning the PID parameters of the robot's DC motors

The arduino sketches are further detailed as follows:
- DC motor board
    - `robot-mtrbrd-files/arduino_dc.ino`: Arduino sketch for the robot's DC motor board
- Sensor and servo motor board
    - `robot-snsbrd-files/arduino_servo_sensors.ino`: Arduino sketch for the robot's sensor and servo motor board (main program)
    - `robot-snsbrd-files/arduino_symetry_calibrate_read.ino`: Arduino sketch for the robot's sensor and servo motor board (symmetry calibration program)
    - `robot-snsbrd-files/arduino_three_baseline_read.ino`: Arduino sketch for manually calibrating the sensors' baselines before the main program

### Robot design files: TODO

### Control source code: TODO

### Evaluation files: TODO

------------------------------------------------------

## References
All the references for methodologies, source codes and engineering theory have been listed in the thesis document.

------------------------------------------------------

## Acknowledgements: TODO

------------------------------------------------------

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
