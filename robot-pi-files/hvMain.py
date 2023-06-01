# Robot main executable and command listener
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
# Import user libraries
import hvConfiguration as cfg

# Import system libraries
import os
import time
import paho.mqtt.client as paho

# -----------------------------------------------------------------------------
# Global variables
# -----------------------------------------------------------------------------
class mqttStatusClass():
    executable = "hvCommander.py" # Python executable relative path
    runName = "run1" # Name of run
    robotName = "" # Name of robot
    setupCommand = "" # MQTT command from setup topic
    connected = False # Flag to indicate if MQTT connection is established
    messageReceived = False # Flag to indicate if message has been received
    startRun = False # Flag to start robot
    msgTimer = 0

# -----------------------------------------------------------------------------
# MQTT functions
# -----------------------------------------------------------------------------
# setupCommand = "" # MQTT command from setup topic
# connected = False # Flag to indicate if MQTT connection is established
# messageReceived = False # Flag to indicate if message has been received
# startRun = False # Flag to start robot
# msgTimer = 0

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print('Setup connection complete.')
        mqttUpdate.connected = True
        mqttUpdate.msgTimer = time.perf_counter()
    else:
        print('Bad setup connection, returned code=', rc)

def on_message(client, userdata, msg):
    topic = msg.topic
    m_decode = str(msg.payload.decode('utf-8', 'ignore'))
    # print('Message received: ', m_decode)
    # print('Topic: ', topic)
    mqttUpdate.messageReceived = True

    # Parse the message
    if m_decode != "":
        messageList = m_decode.split(',')

        if len(messageList) > 1:
            mqttUpdate.setupCommand = messageList[0]
            mqttUpdate.runName = messageList[1]
        else: 
            mqttUpdate.setupCommand = messageList
    

# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------
# Function to build the shell command to execute
def buildExecString():
    # Check if run name is empty
    if mqttUpdate.runName == "": runArg = ""
    else: runArg = " --run_name " + mqttUpdate.runName
    if mqttUpdate.robotName == "": robotArg = ""
    else: robotArg = " --robot_name " + mqttUpdate.robotName

    return "python " + mqttUpdate.executable + runArg + robotArg

# -----------------------------------------------------------------------------
# Main executable
# -----------------------------------------------------------------------------
def main(flags):
    # Start timer
    runTimer = time.perf_counter()

    if cfg.enableMQTT:
        # MQTT client
        client = paho.Client('robotManager')
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(cfg.mqttBroker, cfg.mqttPort)
        client.loop_start()
        while not mqttUpdate.connected: # Wait for connection until timeout
            if time.perf_counter() - runTimer > cfg.mqttConnectTimeout:
                raise Exception("Timed out while waiting for MQTT connection.")
            time.sleep(0.2)

        # MQTT subscribe to command topic
        client.subscribe(cfg.commandTopic, qos=0)

        # Wait for command
        while True:
            if not mqttUpdate.messageReceived:
                if time.perf_counter() - mqttUpdate.msgTimer > cfg.mqttMsgTimeout:
                    print(f"No message received in last {cfg.mqttMsgTimeout} seconds. Shutting down.")
                    break
                time.sleep(0.2)
            elif mqttUpdate.setupCommand == 'start':
                print("Starting up robot...")
                # Unsubscribe from command topic to delegate stop command to state machine
                client.unsubscribe(cfg.commandTopic)
                os.system(buildExecString())
                # Resubscribe to command topic
                client.subscribe(cfg.commandTopic, qos=0)
                mqttUpdate.messageReceived = False # Continue loop to receive new start command
                mqttUpdate.msgTimer = time.perf_counter()
            else: # If message is received, but not start, wait for next message
                mqttUpdate.messageReceived = False
                mqttUpdate.msgTimer = time.perf_counter()
            
        # At end of robot execution, stop MQTT client
        client.loop_stop()
        client.disconnect()

    else: # Start the robot without waiting for MQTT
        os.system(buildExecString())

if __name__ == '__main__':
    mqttUpdate = mqttStatusClass()

    main(mqttUpdate)