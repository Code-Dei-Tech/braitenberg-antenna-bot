# Pi straight line run test to measure max loaded speed
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
import time
import csv
import numpy as np
import serial
import serial.tools.list_ports
import os
import traceback
import logging
import matplotlib.pyplot as plt
# import matplotlib.animation as animation
from matplotlib import style

# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------
### Serial communication setup functions
def getSerialPorts():
    # List available serial ports
    ports = serial.tools.list_ports.comports()
    mtrDevice = None
    i = 0

    for port in sorted(ports):
        # Only retrieve the motor board
        if port.serial_number == cfg.mtrSerialNumber:
            mtrDevice = port.device
            mtrName = port.name

    if not mtrDevice:
        # Raise exception if no motor board is found
        raise Exception("No motor board MTRBRD found.")
    else:
        # Return serial objects with indexed ports and the port names
        return (serial.Serial(mtrDevice, cfg.defaultBaudRate, timeout=1),
            mtrName)

def shutdownSerialPorts(mtrSer):
    # Stop all motors and close the serial ports
    mtrSer.write(f"<2>".encode())
    mtrSer.flush()
    time.sleep(cfg.serialTime)
    mtrSer.close()

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

### CSV writing functions
def generateFolders():
    # Create parent folder if it does not exist
    if not os.path.exists(cfg.parentFolder):
        os.makedirs(cfg.parentFolder)

    # Create dc velocity folder if it does not exist
    if not os.path.exists(cfg.dcVelocityFolder):
        os.makedirs(cfg.dcVelocityFolder)

def renameCSVFilename(csvFilename: str):
    # Check if CSV file exists
    if os.path.exists(csvFilename):
        i = 1
        while os.path.exists(csvFilename.split('.')[0] + '_' + str(i) + '.csv'):
            i += 1

        # Return the new filename
        return csvFilename.split('.')[0] + '_' + str(i) + '.csv'

    # Return the original filename
    return csvFilename

def writeCSV(csvFileName: str, csvHeader: list, csvData: list):
    # Write the CSV file
    with open(csvFileName, 'a', newline='') as csvFile:
        csvWriter = csv.writer(csvFile)

        # Write header if the file is empty
        if os.stat(csvFileName).st_size == 0:
            csvWriter.writerow(csvHeader)

        # Write data
        csvWriter.writerow(csvData)

    # Close the CSV file
    csvFile.close()

# -----------------------------------------------------------------------------
# Main function
# -----------------------------------------------------------------------------
# Parameters
endStrTime = 5
endOscTime = 5
oscInterval = 0.5
strName = 'straightLineRun'
oscName = 'oscillatingRun'
csvStrHeader = ['timeStamp', 'velocityLeft', 'velocityRight']
csvStrFileName = cfg.dcVelocityFolder + strName + "_str_velocity_data.csv"
csvOscHeader = ['timeStamp', 'velocityLeft', 'velocityRight', 'velocityRef']
csvOscFileName = cfg.dcVelocityFolder + oscName + "_osc_velocity_data.csv"
suffix = ''

# Connect to MTRBRD
mtrSer, mtrName = getSerialPorts()

# Wait for MTRBRD initialization serial reads
startTime = time.time()
while time.time() - startTime < cfg.snsPrepTime:
    # Read serial data
    if mtrSer.in_waiting > 0:
        line = mtrSer.readline().decode('utf-8').rstrip()
        
        # Check if the data is a message
        if getType(line) == "m":
            # Parse the message
            message = getContents(line)
            print(message)

# Reconfigure MTRBRD
brdCfg.mtrReconfigVerbose(mtrSer) # Set the verbose mode
brdCfg.mtrReconfigPrecision(mtrSer) # Set the serial writing precisions
brdCfg.mtrReconfigPID(mtrSer) # Set the PID gains
brdCfg.mtrReconfigFilter(mtrSer) # Set the filter coefficients

print("\n")
print("High speed motor test")
print("rXXX: Straight run with RPM value")
print("pXX-XX: Oscillating square run between two RPM values")
print("s-AAA: Add suffix to filename")
print("e: Exit")
print("\n")    
while(1):
    try:
        x=input()
        
        # Run at a specified speed for 5 seconds
        if x[0] == 'r' and x[1:].isnumeric():
            timeStamp = []
            velocityLeft = []
            velocityRight = []

            print("Running motor at full speed for 5 seconds.")
            value = int(x[1:])
            mtrSer.write(f"<1,{0*1000},{0*1000}>".encode())
            time.sleep(1)
            mtrSer.write(f"<1,{value*1000},{value*1000}>".encode())
            mtrSer.flush()
            
            startTime = time.time()
            while time.time() - startTime < endStrTime:
                # Record speed data 
                if mtrSer.in_waiting > 0:
                    line = mtrSer.readline().decode('utf-8').rstrip()
                    # print(line)
                    if getType(line) == 'd':
                        try:
                            dataContents = getContents(line)
                            dataContents = dataContents.split(',')

                            timeStamp.append(time.time() - startTime)
                        
                            velocityLeft.append(float(dataContents[0]))
                            velocityRight.append(float(dataContents[1]))
                        except Exception as e:
                            # If any problem with data parsing
                            timeStamp.append(time.time() - startTime)
                            velocityLeft.append(np.nan)
                            velocityRight.append(np.nan)

            mtrSer.write(f"<1,{0*1000},{0*1000}>".encode())
            time.sleep(1)
            mtrSer.flush()
            mtrSer.write(f"<2>".encode())

            # Write results to CSV
            generateFolders()
            fileName = renameCSVFilename(csvStrFileName[0:-4] + suffix + '.csv')
            for i in range(len(timeStamp)):
                csvData = [timeStamp[i], velocityLeft[i], velocityRight[i]]
                writeCSV(fileName, csvStrHeader, csvData)

            # Reset suffix and print done
            suffix = ''
            print('Done')

        # Run at oscillating speeds
        if (x[0] == 'p' and x.split('-')[0][1:].isnumeric()
            and x.split('-')[1].isnumeric):
            timeStamp = []
            velocityLeft = []
            velocityRight = []
            refVel = []

            value1 = int(x.split('-')[0][1:])
            value2 = int(x.split('-')[1])

            print("Running motor at oscillating speeds for 5 seconds.")
            mtrSer.write(f"<1,{0*1000},{0*1000}>".encode())
            time.sleep(1)

            startTime = time.time()
            startInterval = time.time()
            value = 0
            while time.time() - startTime < endOscTime:
                # Write speed at start of interval
                if time.time() - startInterval > oscInterval:
                    # Alternate between value 1 and value 2
                    if value == value1: value = value2
                    else: value = value1

                    mtrSer.write(f"<1,{value*1000},{value*1000}>".encode())
                    startInterval = time.time()

                # Record speed data
                if mtrSer.in_waiting > 0:
                    line = mtrSer.readline().decode('utf-8').rstrip()
                    # print(line)
                    if getType(line) == 'd':
                        try:
                            dataContents = getContents(line)
                            dataContents = dataContents.split(',')

                            timeStamp.append(time.time() - startTime)
                            velocityLeft.append(float(dataContents[0]))
                            velocityRight.append(float(dataContents[1]))
                            refVel.append(value)
                        except Exception as e:
                            # If any problem with data parsing
                            timeStamp.append(time.time() - startTime)
                            velocityLeft.append(np.nan)
                            velocityRight.append(np.nan)
                            refVel.append(value)

            mtrSer.write(f"<1,{0*1000},{0*1000}>".encode())
            time.sleep(1)
            mtrSer.flush()
            mtrSer.write(f"<2>".encode())

            # Write results to CSV
            generateFolders()
            fileName = renameCSVFilename(csvStrFileName[0:-4] + suffix + '.csv')
            for i in range(len(timeStamp)):
                csvData = [timeStamp[i], velocityLeft[i], velocityRight[i], refVel[i]]
                writeCSV(fileName, csvOscHeader, csvData)

            # Plot results
            fig, ax = plt.subplots()
            ax.plot(timeStamp, velocityLeft, label='Left wheel')
            ax.plot(timeStamp, velocityRight, label='Right wheel')
            ax.plot(timeStamp, refVel, label='Reference')
            ax.set(xlabel='Time (s)', ylabel='Velocity (RPM)',
                title='Velocity vs Time')
            ax.grid()
            ax.legend()
            # Save to file
            fig.savefig(cfg.dcVelocityFolder + oscName + "_osc_velocity_plot" + suffix + ".png")
            
            # Reset suffix and print done
            suffix = ''
            print('Done')

        elif x[0]=='s':
            # Add suffix to filename
            suffix = x.split('-')[1]

            print(f'Suffix {suffix} recorded for next instruction.')

        elif x[0]=='i':
            # Change interval time
            oscInterval = float(x.split('-')[1])

            print(f'Interval time {oscInterval} saved for next oscillation control.')

        elif x=='e':
            "Shutting down."
            shutdownSerialPorts(mtrSer)
            break

        else:
            print("Invalid input")
    except KeyboardInterrupt:
        shutdownSerialPorts(mtrSer)
        break
    except Exception as e:
        # Shut down arduino
        shutdownSerialPorts(mtrSer)
        
        # Print error
        logging.error(traceback.format_exc()) # Output the error
        break