# tVOC reading of three sensors
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
# Import user libraries
import hvConfiguration as cfg
import hvArduinoReconfig as brdCfg

# Import system libraries
import serial
import serial.tools.list_ports
import numpy as np
import csv
import time 
import os
import argparse

# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------
### Argument parsing function
def parseArguments():
    # Parse the arguments
    parser = argparse.ArgumentParser(description='Read data of three sensors.')
    parser.add_argument('-n', '--name', type=str, default="sym_calibration", help='Name of the run')
    parser.add_argument('-p', '--position', type=int, default=0, help='Position of the sensor board')
    parser.add_argument('-i', '--iteration', type=int, default=0, help='Iteration of the sensor board')
    args = parser.parse_args()

    # Return the arguments
    return (args.name, args.position, args.iteration)

### Data parsing functions
def cropLine(line: str, start: str, end: str):
    # Crop a line between two characters
    idxStart = line.find(start)
    idxEnd = line.find(end)
    return line[idxStart+len(start):idxEnd]

def getType(line: str):
    # Get the communication type of the incoming data
    return cropLine(line, '{', '}')

def getContents(line: str):
    # Get the contents of the incoming data
    return cropLine(line, '<', '>')

### Serial communication setup functions
# Function to get the sensor board serial port
def getSerialPorts():
    # List available serial ports
    ports = serial.tools.list_ports.comports()
    calDevice = None
    i = 0

    for port in sorted(ports):
        # Only retrieve the sensor board
        if port.serial_number == cfg.snsSerialNumber:
            calDevice = port.device
            calName = port.name

    if not calDevice: 
        # Raise exception if no sensor board is found
        raise Exception("No sensor board CALBRD found.")
    else:
        # Return serial objects with indexed ports and the port names
        return (serial.Serial(calDevice, cfg.defaultBaudRate, timeout=2),
            calName)

def shutdownSerialPorts(calSer):
    # Stop all motors and close the serial ports
    calSer.flush()
    time.sleep(cfg.serialTime)
    calSer.close()

# -----------------------------------------------------------------------------
# Main function
# -----------------------------------------------------------------------------
def main(args: tuple):
    # CSV configuration
    name = args[0]
    position = args[1]
    iteration = args[2]
    runName = name + "_p" + str(position) + "_i" + str(iteration)
    endTime = cfg.symTime # in seconds
    csvFileName = cfg.sensorSymmetryFolder + cfg.sensorRawFileNamePrefix + runName + ".csv"
    csvHeader = ["timestamp", "sensor0", "sensor1", "sensor2"]

    # Main data variables
    timeStamp = []
    dataLeft = []
    dataRight = []
    dataMiddle = []
    idx = 0

    # Offset variables
    initLeftArr = []
    initRightArr = []
    initMiddleArr = []
    offsetMeasureStop = False
    offsetLeft = 0
    offsetRight = 0
    offsetMiddle = 0
    offsetIdx = 0

    # Start up communication with Arduino
    calSer, calPort = getSerialPorts()
    calSer.flush()

    # Wait for sensor initialization serial reads
    startTime = time.time()
    while time.time() - startTime < cfg.snsPrepTime:
        # Read serial data
        if calSer.in_waiting > 0:
            line = calSer.readline().decode('utf-8').rstrip()
            
            # Check if the data is a message
            if getType(line) == "m":
                # Parse the message
                message = getContents(line)
                print(message)

    # Reconfigure arduino baselines
    # time.sleep(0.5)
    brdCfg.snsReconfigBaseline(calSer, 0, cfg.sensorLeftID) # Set the baseline value of the left sensor
    # time.sleep(0.5)
    brdCfg.snsReconfigBaseline(calSer, 1, cfg.sensorRightID) # Set the baseline value of the right sensor
    # time.sleep(0.5)
    brdCfg.snsReconfigBaseline(calSer, 2, cfg.sensorMiddleID) # Set the baseline value of the middle sensor
    # time.sleep(0.5)

    # Send forward servo command
    calSer.write(f'<1,{cfg.servoFrontLeft},{cfg.servoFrontRight}>'.encode('utf-8'))

    # Loop receiving data
    startTime = time.time()
    while True:
        try:
            # Break at the end of the test
            if time.time() - startTime > endTime:
                break

            # Read serial data
            if calSer.in_waiting > 0:
                line = calSer.readline().decode('utf-8').rstrip()
                
                # Check if the data is a message
                if getType(line) == "m":
                    # Parse the message
                    message = getContents(line)
                    print(message)
                # Check if the data is sensor data
                if getType(line) == "d":
                    # Parse the sensor data
                    sensorData = []
                    sensorData = getContents(line).split(',')
                    sensorData = [int(i) for i in sensorData]

                    # Record initial sensor values for offset calculation
                    if ((time.time() - startTime > cfg.symOffsetStart) and
                        (time.time() - startTime <= cfg.symOffsetEnd) and 
                        not offsetMeasureStop):
                        initLeftArr.append(sensorData[0])
                        initRightArr.append(sensorData[1])
                        initMiddleArr.append(sensorData[2])
                    elif ((time.time() - startTime > cfg.symOffsetEnd) and 
                        not offsetMeasureStop):
                        offsetLeft = int(np.mean(initLeftArr))
                        offsetRight = int(np.mean(initRightArr))
                        offsetMiddle = int(np.mean(initMiddleArr))
                        offsetIdx = idx
                        # print([offsetLeft, offsetRight, offsetMiddle])
                        offsetMeasureStop = True
                    
                    sensorData[0] = sensorData[0] - offsetLeft
                    sensorData[1] = sensorData[1] - offsetRight
                    sensorData[2] = sensorData[2] - offsetMiddle
                    # print(sensorData)

                    # Timestamp of date and time
                    timeStamp.append(time.strftime("%Y-%m-%d %H:%M:%S"))
                    dataLeft.append(sensorData[0])
                    dataRight.append(sensorData[1])
                    dataMiddle.append(sensorData[2])
                    
                idx += 1
        except KeyboardInterrupt:
            print("Keyboard interrupt detected, closing serial port.")
            break
    
    # Write sideways servo position and shutdown serial ports
    calSer.write(f'<1,{cfg.servoDefaultLeft},{cfg.servoDefaultRight}>'.encode('utf-8'))
    shutdownSerialPorts(calSer)

    # Correct first few seconds of measured data with offset
    dataLeft[:offsetIdx] = list(np.array(dataLeft[:offsetIdx]) - offsetLeft)
    dataLeft = [0 if i < 0 else i for i in dataLeft]
    dataRight[:offsetIdx] = list(np.array(dataRight[:offsetIdx]) - offsetRight)
    dataRight = [0 if i < 0 else i for i in dataRight]
    dataMiddle[:offsetIdx] = list(np.array(dataMiddle[:offsetIdx]) - offsetMiddle)
    dataMiddle = [0 if i < 0 else i for i in dataMiddle]
    
    # Write saved arrays to csv
    # Create parent folder if it does not exist
    if not os.path.exists(cfg.parentFolder):
        os.makedirs(cfg.parentFolder)

    if not os.path.exists(cfg.sensorSymmetryFolder):
        os.makedirs(cfg.sensorSymmetryFolder)

    # Write the sensor data to a CSV file
    with open(csvFileName, 'a', newline='') as csvFile:
        csvWriter = csv.writer(csvFile)

        # Write header if the file is empty
        if csvFile.tell() == 0:
            csvWriter.writerow(csvHeader)

        csvWriter = csv.writer(csvFile)
        for i in range(len(timeStamp)):
            csvWriter.writerow([timeStamp[i], dataLeft[i], dataRight[i], dataMiddle[i]])

    # Close the CSV file
    csvFile.close()

# Run the main function
if __name__ == "__main__":
    main(parseArguments())