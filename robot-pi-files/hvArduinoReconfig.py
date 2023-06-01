# Robot Arduino reconfiguration library
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
# Import user libraries
import hvConfiguration as cfg

# Import system libraries
import time
import numpy as np
import serial

# -----------------------------------------------------------------------------
# Reconfiguration functions for the DC motor Arduino
# -----------------------------------------------------------------------------
def mtrReconfigVerbose(ser: serial.Serial):
    # Set the verbose mode
    print(f"Setting MTRBRD verbose mode to {cfg.mtrVerbose}...")

    ser.write(f"<0,0,{cfg.mtrVerbose}>".encode())
    time.sleep(cfg.reconfigDelay)

def mtrNewBaudSerial(oldSer: serial.Serial, port: str):
    # Restart the serial connection with new baudrate
    print(f"Setting MTRBRD baud rate to {cfg.mtrBaudRate} and "
          f"restarting serial port...")

    if cfg.defaultBaudRate != cfg.mtrBaudRate:
        oldSer.write(f"<0,1,{cfg.mtrBaudRate}>".encode())
        time.sleep(cfg.reconfigDelay)
        oldSer.close()
        return serial.Serial(
            port, cfg.mtrBaudRate, timeout=1
        )
    else: return oldSer

def mtrReconfigPrecision(ser: serial.Serial):
    # Set the incoming precision values for reconfiguration and speeds
    print(f"Setting MTRBRD precision to "
          f"{cfg.mtrSpeedPrecision} speed decimals and "
          f"{cfg.mtrConfigPrecision} config decimals...")
    
    ser.write(f"<0,2,{cfg.mtrSpeedPrecision}>".encode())
    time.sleep(cfg.reconfigDelay)

    ser.write(f"<0,3,{cfg.mtrConfigPrecision}>".encode())
    time.sleep(cfg.reconfigDelay)

def mtrReconfigPID(ser: serial.Serial):
    # Set the PID gains
    print(f"Setting MTRBRD PID gains to {cfg.pidKP}, {cfg.pidKI}, {cfg.pidKD}...")

    serialKP = int(cfg.pidKP * pow(10, cfg.mtrConfigPrecision))
    ser.write(f"<0,4,{serialKP}>".encode())
    time.sleep(cfg.reconfigDelay)

    serialKI = int(cfg.pidKI * pow(10, cfg.mtrConfigPrecision))
    ser.write(f"<0,5,{serialKI}>".encode())
    time.sleep(cfg.reconfigDelay)

    serialKD = int(cfg.pidKD * pow(10, cfg.mtrConfigPrecision))
    ser.write(f"<0,6,{serialKD}>".encode())
    time.sleep(cfg.reconfigDelay)

def mtrReconfigFilter(ser: serial.Serial):
    # Set the filter coefficients given the cutoff frequency
    print(f"Setting MTRBRD filter coefficients to "
          f"cut off {cfg.dcFilterCutoff} Hz...")

    b, a = cfg.dcFilterCoefficients(cfg.dcFilterCutoff, 
                                    cfg.dcSamplingFrequency)

    serialA = int(a[0] * pow(10, cfg.mtrConfigPrecision))
    ser.write(f"<0,7,{serialA}>".encode())
    time.sleep(cfg.reconfigDelay)

    serialB0 = int(b[0] * pow(10, cfg.mtrConfigPrecision))
    ser.write(f"<0,8,{serialB0}>".encode())
    time.sleep(cfg.reconfigDelay)

    serialB1 = int(b[1] * pow(10, cfg.mtrConfigPrecision))
    ser.write(f"<0,9,{serialB1}>".encode())
    time.sleep(cfg.reconfigDelay)

# -----------------------------------------------------------------------------
# Reconfiguration functions for the servo and sensor Arduino
# -----------------------------------------------------------------------------
def snsReconfigSensorLoops(ser: serial.Serial):
    # Set the number of sensor loops
    print(f"Setting SNSBRD per-serial sensor loops to {cfg.sensorLoops}...")

    ser.write(f"<0,0,{cfg.sensorLoops}>".encode())
    time.sleep(cfg.reconfigDelay)

def snsReconfigVerbose(ser: serial.Serial):
    # Set the verbose mode
    print(f"Setting SNSBRD verbose mode to {cfg.snsVerbose}...")

    ser.write(f"<0,1,{cfg.snsVerbose}>".encode())
    time.sleep(cfg.reconfigDelay)
    
def snsNewBaudSerial(oldSer: serial.Serial, port: str):
    # Restart the serial connection with new baudrate
    print(f"Setting SNSBRD baud rate to {cfg.snsBaudRate} and "
          f"restarting serial port...")

    if cfg.defaultBaudRate != cfg.snsBaudRate:
        oldSer.write(f"<0,2,{cfg.snsBaudRate}>".encode())
        time.sleep(cfg.reconfigDelay)
        oldSer.close()
        return serial.Serial(
            port, cfg.snsBaudRate, timeout=2*cfg.serialTime
        )
    else: return oldSer

def snsReconfigServoPosition(ser: serial.Serial):
    # Set the servo position
    print(f"Setting SNSBRD servo position to {cfg.servoDefaultLeft} and "
          f"{cfg.servoDefaultRight} ms...")
    
    ser.write(f"<0,3,{cfg.servoDefaultLeft}>".encode())
    time.sleep(cfg.reconfigDelay)

    ser.write(f"<0,4,{cfg.servoDefaultRight}>".encode())
    time.sleep(cfg.reconfigDelay)

def snsReconfigSerialTime(ser: serial.Serial):
    # Set the serial time
    print(f"Setting SNSBRD sync time to {cfg.serialTime} seconds...")

    ser.write(f"<0,5,{1000*cfg.serialTime}>".encode())
    time.sleep(cfg.reconfigDelay)

def snsReconfigBaselineFlag(ser: serial.Serial):
    # Enable baseline setting during warmup
    print(f"Setting SNSBRD baseline flag to {cfg.sensorBaselineReset}...")

    ser.write(f"<0,6,{cfg.sensorBaselineReset}>".encode())
    time.sleep(cfg.reconfigDelay)

def snsReconfigBaselineTime(ser: serial.Serial):
    # Set the baseline time
    print(f"Setting SNSBRD baseline time to {cfg.sensorBaselineTime} seconds...")

    ser.write(f"<0,7,{1000*cfg.sensorBaselineTime}>".encode())
    time.sleep(cfg.reconfigDelay)

def snsReconfigBaseline(ser: serial.Serial, cfgVar: int, sensorID: int):
    # Set the baseline for a given sensor
    baseline = cfg.sensorBaseline3Arr[sensorID]
    print(f"Setting SNSBRD sensor {cfgVar} baseline to {baseline}...")

    ser.write(f"<0,{8+cfgVar},{baseline}>".encode())
    time.sleep(cfg.reconfigDelay*10)

def snsReconfigBaselineLeft(ser: serial.Serial):
    # Set left sensor baseline
    print(f"Setting SNSBRD left sensor baseline to {cfg.sensorBaselineLeft}...")

    ser.write(f"<0,8,{cfg.sensorBaselineLeft}>".encode())
    time.sleep(cfg.reconfigDelay*10)

def snsReconfigBaselineRight(ser: serial.Serial):
    # Set right sensor baseline
    print(f"Setting SNSBRD right sensor baseline to {cfg.sensorBaselineRight}...")

    ser.write(f"<0,9,{cfg.sensorBaselineRight}>".encode())
    time.sleep(cfg.reconfigDelay*10)