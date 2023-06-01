# Pi main command loop
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
# Import user libraries
import hvStateMachine as hvsm
import hvConfiguration as cfg
import hvArduinoReconfig as brdCfg

# Import system libraries
import paho.mqtt.client as paho
import time
import serial
import serial.tools.list_ports
import numpy as np
import os
import traceback
import logging
import argparse

# -----------------------------------------------------------------------------
# MQTT functions
# -----------------------------------------------------------------------------
class mqttStatusClass():
    connected = False # Flag to indicate if MQTT connection is established
    stopFlag = False # MQTT command flag to stop the robot

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print('Runtime connection complete')
        mqttUpdate.connected = True
    else:
        print('Bad runtime connection, returned code=', rc)

def on_message(client, userdata, msg):
    topic = msg.topic
    m_decode = str(msg.payload.decode('utf-8', 'ignore'))
    # print('Message received: ', m_decode)
    # print('Topic: ', topic)

    # Parse the message
    if m_decode != "":
        if m_decode.startswith("stop"):
            mqttUpdate.stopFlag = True


# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------
### Get execution arguments
def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--run_name", type=str, default="run1", help="Name of the experiment run")
    parser.add_argument("--robot_name", type=str, default="robot1", help="Name of the robot")

    args = parser.parse_args()

    return args

### Serial communication setup functions
def getSerialPorts():
    # List available serial ports
    ports = serial.tools.list_ports.comports()
    snsDevice = None
    mtrDevice = None
    i = 0

    for port in sorted(ports):
        # Only retrieve the sensor board
        if port.serial_number == cfg.snsSerialNumber:
            snsDevice = port.device
            snsName = port.name
        elif port.serial_number == cfg.mtrSerialNumber:
            mtrDevice = port.device
            mtrName = port.name

    if not snsDevice: 
        # Raise exception if no sensor board is found
        raise Exception("No sensor board SNSBRD found.")
    if not mtrDevice:
        # Raise exception if no motor board is found
        raise Exception("No motor board MTRBRD found.")
    else:
        # Return serial objects with indexed ports and the port names
        return (serial.Serial(snsDevice, cfg.defaultBaudRate, timeout=1),
            serial.Serial(mtrDevice, cfg.defaultBaudRate, timeout=1),
            snsName, mtrName)

def shutdownSerialPorts(snsSer, mtrSer):
    # Stop all motors and close the serial ports
    snsSer.write(f"<1,0>".encode())
    mtrSer.write(f"<2>".encode())
    snsSer.flush()
    mtrSer.flush()
    time.sleep(cfg.serialTime)
    snsSer.close()
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

# -----------------------------------------------------------------------------
# Main function
# -----------------------------------------------------------------------------
def command(robotName, runName: str, mqttUpdate: mqttStatusClass):
    # -----------------------------------------------------------------------------
    # Global variables
    # -----------------------------------------------------------------------------
    # Misc initial variables
    startRun = False # Flag to start robot run
    robotStarted = False # Flag to indicate if the robot has started
    learningPause = True # Flag to indicate a pause between TUNE states
    i = 0 # Loop counter
    leftSamples = np.zeros(cfg.sensorLoops, dtype=np.uint16) # Left sensor data
    rightSamples = np.zeros(cfg.sensorLoops, dtype=np.uint16) # Right sensor data
    middleSamples = np.zeros(cfg.sensorLoops, dtype=np.uint16) # Middle sensor data
    stateLock = False # Flag for locking the robot's state throughout the run
    baselineLeft = cfg.sensorBaselineLeft # Left baseline value
    baselineRight = cfg.sensorBaselineRight # Right baseline value
    baselineMiddle = cfg.sensorBaselineMiddle # Middle baseline value

    # Initialize timers
    runTimer = time.perf_counter()
    genTimer = time.perf_counter()
    serialInterval = time.perf_counter()
    publishStartTime = time.perf_counter()

    # Initialize MQTT connection and subscription
    if cfg.enableMQTT:
        client = paho.Client('robotCommander')
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(cfg.mqttBroker, cfg.mqttPort)
        client.loop_start()
        while not mqttUpdate.connected: # Wait for connection until timeout
            if time.perf_counter() - runTimer > cfg.mqttConnectTimeout:
                raise Exception("Timed out while waiting for MQTT connection.")
            time.sleep(0.2)
        client.subscribe(cfg.commandTopic, qos=1)

    # Initialize the system
    robot = hvsm.BraitenbergMachine()
    robot.initialize(name=robotName, runName = runName)
    robot.initializeLogs()

    # Initialize the serial connection
    snsSer, mtrSer, snsPort, mtrPort = getSerialPorts()
    snsSer.flush()
    mtrSer.flush()

    # Wait for SNSBRD initialization serial reads
    startTime = time.time()
    
    while time.time() - startTime < cfg.snsPrepTime:
        # Read serial data
        if snsSer.in_waiting > 0:
            line = snsSer.readline().decode('utf-8').rstrip()
            
            # Check if the data is a message
            if getType(line) == "m":
                # Parse the message
                message = getContents(line)
                print(message)

    ### Reconfigure the sensor and servo board
    # snsSer = brdCfg.snsNewBaudSerial(snsSer, snsPort) # Set the baud rate and restart connection
    brdCfg.snsReconfigVerbose(snsSer) # Set the verbose mode
    # brdCfg.snsReconfigBaselineLeft(snsSer) # Set the left baseline value-
    
    brdCfg.snsReconfigSensorLoops(snsSer) # Set the number of sensor loops
    brdCfg.snsReconfigServoPosition(snsSer) # Set the default servo positions
    # brdCfg.snsReconfigBaselineFlag(snsSer) # Set the baseline flag
    # brdCfg.snsReconfigBaselineTime(snsSer) # Set the baseline reset time
    # brdCfg.snsReconfigBaselineRight(snsSer) # Set the right baseline value
    brdCfg.snsReconfigSerialTime(snsSer) # Set the serial time

    brdCfg.snsReconfigBaseline(snsSer, 0, 0) # Set the baseline value of the left sensor
    brdCfg.snsReconfigBaseline(snsSer, 1, 1) # Set the baseline value of the right sensor
    brdCfg.snsReconfigBaseline(snsSer, 2, 2) # Set the baseline value of the middle sensor

    ### Reconfigure the motor board
    # mtrSer = brdCfg.mtrNewBaudSerial(mtrSer, mtrPort) # Set the baud rate and restart connection
    brdCfg.mtrReconfigVerbose(mtrSer) # Set the verbose mode
    brdCfg.mtrReconfigPrecision(mtrSer) # Set the serial writing precisions
    brdCfg.mtrReconfigPID(mtrSer) # Set the PID gains
    brdCfg.mtrReconfigFilter(mtrSer) # Set the filter coefficients

    # Control loop
    startRun = True
    while True:
        try:
            # MQTT controls
            if cfg.enableMQTT and mqttUpdate.connected:
                # Publish robot state every 0.5 seconds
                if (time.perf_counter() - publishStartTime >= 0.5):
                    client.publish(cfg.stateTopic, str(robot.stateID))
                    publishStartTime = time.perf_counter()

                if mqttUpdate.stopFlag:
                    # Stop the robot
                    print("Stop command received. Shutting down.")
                    startRun = False
                    robot.send("toStop")
                    client.publish(cfg.stateTopic, "0")
                    break

            # Start robot run
            if startRun and (time.perf_counter() - runTimer >= cfg.runTimeStart):
                # Start the robot
                robot.startRun()
                # Set start flag to true
                robotStarted = True
                # Restart the run timer
                runTimer = time.perf_counter()
                # Restart the serial timer
                serialInterval = time.perf_counter()
                # Restart the tuning timer from TRANSFER state (TO DO)

                startRun = False # Reset the start flag to prevent multiple starts

            # Stop robot and python script at timeout
            if (time.perf_counter() - runTimer >= cfg.runTimeout) and robotStarted:
                # Stop the robot
                robot.stopRun()
                # Set start flag to false
                robotStarted = False
                break

            # Calculate motor values and send to Arduino boards
            if (time.perf_counter() - serialInterval >= cfg.serialTime) and robotStarted:
                # Update serial timer
                serialInterval = time.perf_counter()

                # Send command to Arduino boards
                snsOut = robot.generateServoCommand()
                snsSer.write(snsOut.encode('utf-8'))

                mtrOut = robot.generateDCCommand()
                mtrSer.write(mtrOut.encode('utf-8'))

                # Update genetic algorithm population
                if cfg.geneticAlgorithm and robot.current_state == robot.TUNE:
                    robot.updateGeneticAlgorithm()

                # if cfg.snsCorrection:
                #     print(f'Original instant signals: Left: {robot.sigInstLeft} | Right: {robot.sigInstRight}')
                #     print(f'Corrected instant signals: Left: {robot.sigInstCorrLeft} | Right: {robot.sigInstCorrRight}')
                # print('MTRBRD << ' + mtrOut)
                # print('SNSBRD << ' + snsOut)

                # Write to log file
                robot.writeToExpLog()
                # time.sleep(cfg.serialTime/2)

            # Receive and print message from motor board MTRBRD
            if mtrSer.in_waiting > 0:
                mtrLine = mtrSer.readline().decode('utf-8').rstrip()
                # print(mtrLine)

                # Parse message type
                mtrMsgType = getType(mtrLine)

                # Parse message contents
                if mtrMsgType == 'm': # Status message as a string
                    mtrMsgContents = getContents(mtrLine)
                    print(mtrMsgContents)

            # Receive message from sensor and servo board SNSBRD
            if snsSer.in_waiting > 0:
                snsLine = snsSer.readline().decode('utf-8').rstrip()
                # print(snsLine)
                # Parse message type
                snsMsgType = getType(snsLine)

                # Parse message contents
                if snsMsgType == 'm': # Status message as a string
                    snsMsgContents = getContents(snsLine)
                    print(snsMsgContents)
                elif snsMsgType == 'b': # Baseline data as a list of samples
                    dataContents = getContents(snsLine)
                    dataContents = dataContents.split(',')

                    # Overwrite default baseline values for logging
                    baselineLeft = int(dataContents[0])
                    baselineRight = int(dataContents[1])
                    baselineMiddle = int(dataContents[2])

                    print(f"Baselines updated: {baselineLeft} | {baselineRight} | {baselineMiddle}")
                elif snsMsgType == 'd': # Sensor data as a list of samples
                    dataContents = getContents(snsLine)
                    dataContents = dataContents.split(';')
                    for x in range(cfg.sensorLoops):
                        localSamples = dataContents[x].split(',')
                        leftSamples[x] = int(localSamples[0])
                        rightSamples[x] = int(localSamples[1])
                        middleSamples[x] = int(localSamples[2])

                    # Update robot sensors
                    print(f"Left: {leftSamples} | Right: {rightSamples} | Middle: {middleSamples}")
                    robot.updateSensorData(leftSamples, rightSamples, middleSamples)

                    # If doing static response tests, force the robot in SURGE state 
                    # and prevent transitions
                    if cfg.staticBraitenbergTest and robot.SEARCH.is_active:
                        robot.current_state = robot.SURGE
                        robot.activateServos = True
                        robot.modeDC = 3
                        stateLock = True

                    # Otherwise, update the state machine normally
                    elif not stateLock:
                        robot.updateRobotState()

                    # If SURGE state is entered, start genetic algorithm timer
                    if (cfg.geneticAlgorithm and 
                        robot.current_state == robot.SURGE and
                        learningPause):
                        genTimer = time.perf_counter()
                        learningPause = False
                    
                    # Print current state
                    print(f"Current state: {robot.current_state.name}")

            # Loop delay (Optional)
            # time.sleep(0.01)

            # Begin genetic algorithm after learning pause interval
            if ((time.perf_counter() - genTimer >= cfg.genInterval) and not
                learningPause and robot.current_state == robot.SURGE):
                # Update the robot's state to TUNE
                robot.send("toTune")

                # Reset pause flag
                learningPause = True

            # Increment loop counter
            i += 1

        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Shutting down.")
            break

        except Exception as e:
            print("Python-side error detected. Shutting down.")
            logging.error(traceback.format_exc()) # Output the error
            break

    # Close serial communication after timeout or exception
    
    shutdownSerialPorts(snsSer, mtrSer)
    robot.writeToCfgLog(
        time.perf_counter() - runTimer, 
        baselineLeft, baselineRight
    )

    # Disconnect MQTT client after finish
    if cfg.enableMQTT:
        client.disconnect()
        client.loop_stop()

if __name__ == "__main__":
    args = get_args()
    mqttUpdate = mqttStatusClass()

    command(
        robotName=args.robot_name,
        runName=args.run_name,
        mqttUpdate=mqttUpdate
    )