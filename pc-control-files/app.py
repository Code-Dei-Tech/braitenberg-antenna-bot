# Host main script with GUI interface, Apriltag detection and MQTT connection
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
# Import system libraries 
import cv2 as cv
import tkinter as tk
import time
import sys
import os
import csv
import numpy as np
from PIL import Image, ImageTk
import paho.mqtt.client as paho
import winsound

# Import user libraries
import apriltag_detect_record as apriltag
import apriltag_config as cfg

# -----------------------------------------------------------------------------
# Global variables
# -----------------------------------------------------------------------------
# MQTT variables
connected = False # Flag to indicate if MQTT client is connected
broker = '192.168.149.40'
port = 1883
commandTopic = 'commandTopic'
stateTopic = 'stateTopic'
connectTimeout = 60 # Time in seconds to wait for connection
# captureID = 'USB\\VID_046D&PID_08B6&MI_00\\6&218D046D&0&0000'
# captureID = 'USB\\Class_0e'
captureID = 0
captureArg = cv.CAP_DSHOW
# captureArg = None
# captureArg = cv.CAP_ANY

resultsParentFolder = cfg.parentFolder # Folder to store all experiment results
resultsCSV = 'results.csv' # CSV file to store all experiment results
resultsPath = resultsParentFolder + resultsCSV

warmupTime = 15 # Time in seconds to wait for robot sensors to warm up
boundaryCheckTime = warmupTime + 20 # Time after switch to search state to start boundary check
# Create list of numbers from 0 to 8
robotStateList = list(range(9))
robotStateIDs = ['STOP', 'START', 'SEARCH', 'TRANSFER', 'SURGE', 'CAST', 'TUNE', 'CALIBRATE']

# Notification sound for releasing source at 15 seconds
# Comment out winsound line for automated source control
# winsound.Beep has a blocking delay based on duration, but winsound.MessageBeep does not
# frequency = 2500  # Set Frequency To 2500 Hertz
# duration = 100  # Set Duration To 1000 ms == 1 second

# Constants for the app window size and title
VIDEO_WIDTH = 608
VIDEO_HEIGHT = 342
WINDOW_WIDTH = VIDEO_WIDTH+300
WINDOW_HEIGHT = VIDEO_HEIGHT*2
WINDOW_TITLE = "Gas Localization Robot Control App"

# -----------------------------------------------------------------------------
# MQTT functions
# -----------------------------------------------------------------------------
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print('Host connection complete')
        global connected
        connected = True
    else:
        print('Bad host connection, returned code=', rc)

def on_message(client, userdata, msg):
    topic = msg.topic
    store = 0
    m_decode = str(msg.payload.decode('utf-8', 'ignore'))
    # print('Message received: ', m_decode)
    # print('Topic: ', topic)

    # Parse the message
    if m_decode != "":
        if int(m_decode) > 0 and not app.robotStarted: 
            # Enable flags and start timers
            app.expStartFlag = True
            app.beepFlag = True
            app.boundaryFlag = True
            app.expStartTime = time.perf_counter()
            app.boundaryStartTime = time.perf_counter()
            app.robotStarted = True

        # Display robot state in app
        if int(m_decode) < 9 and int(m_decode) != store:
            app.robotState = int(m_decode)
            app.status_label.config(text=f"Robot state: {robotStateIDs[app.robotState]}")
            store = int(m_decode)

# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------
# Function to resize the image based on one dimension, keeping the aspect ratio
def imageResize(image, width = None, height = None, inter = cv.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the image
    resized = cv.resize(image, dim, interpolation = inter)

    # return the resized image
    return resized

# Function to write experiment outcome to CSV file
def writeExpCSV(experimentName, duration, lastCoord, lastHeading, outcome):
    if experimentName != "": # Check if experiment name is not empty
        dateAndTimeLogged = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        csvHeader = ['dateTime', 'experimentName', 'duration', 'lastCoord', 'lastHeading', 'outcome']
        rowData = []
        rowData.append(dateAndTimeLogged)
        rowData.append(experimentName)
        rowData.append(duration)
        rowData.append(lastCoord)
        rowData.append(lastHeading)
        rowData.append(outcome)

        # Create parent folder if it does not exist
        if not os.path.exists(cfg.parentFolder):
            os.makedirs(cfg.parentFolder)

        with open(resultsPath, 'a', newline="") as f:
            writer = csv.writer(f, delimiter=",")

            # Check if file is empty
            if os.stat(resultsPath).st_size == 0:
                writer.writerow(csvHeader)
            
            writer.writerow(rowData)

        f.close()

# -----------------------------------------------------------------------------
# App layout
# -----------------------------------------------------------------------------
# Main app class
class App:
    def __init__(self, window, mqttClient: paho.Client):
        # Instantiate apriltag detection class
        self.apriltag = apriltag.tagDetector()
        self.writeApriltagCSV = False
        self.writeExpCSV = False
        self.lastCoords = np.nan
        self.lastHeading = np.nan

        # Set up flags and timers
        self.expStartFlag = False # Flag to start experiment timer after warmup is done
        self.beepFlag = False # Flag to beep at 15 seconds
        self.expStartTime = 0.0 # Experiment start time
        self.boundaryFlag = False # Flag to start checking if robot crosses boundary
        self.boundaryStartTime = 0.0 # Boundary start time
        self.robotStarted = False # Flag to indicate if robot has started
        self.robotState = 0 # Robot state

        self.outcome = ""

        # Set up MQTT client
        self.mqttClient = mqttClient

        # Set up GUI
        self.window = window
        self.window.title(WINDOW_TITLE)
        # self.window.protocol("WM_DELETE_WINDOW", self.window.destroy())
        # self.window.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")

        # Create a frame to hold the video capture and image
        self.video_frame = tk.Frame(self.window)
        self.video_frame.pack(side=tk.LEFT, padx=10, pady=10)

        # Create a canvas to hold the video capture on the top
        self.video_canvas = tk.Canvas(self.video_frame, width=VIDEO_WIDTH, height=VIDEO_HEIGHT)
        self.video_canvas.pack(side=tk.TOP)

        # Create a canvas to hold the image on the bottom
        self.topdown_canvas = tk.Canvas(self.video_frame, width=VIDEO_WIDTH, height=VIDEO_HEIGHT)
        self.topdown_canvas.pack(side=tk.BOTTOM)

        # Display a random image 
        # self.image = Image.open("Implementation\Host 24-04-23\Wind.png")
        # self.image = self.image.resize((VIDEO_WIDTH, VIDEO_HEIGHT), Image.ANTIALIAS)
        # self.topdown_photo = ImageTk.PhotoImage(self.image)
        # self.topdown_canvas.create_image(0, 0, image=self.topdown_photo, anchor=tk.NW)

        # Create a canvas on the right side
        self.right_frame = tk.Frame(self.window, width=300, height=WINDOW_HEIGHT)
        self.right_frame.pack(side=tk.RIGHT)
        self.right_frame.pack_propagate(False)

        # Create a status frame on the bottom
        self.status_frame = tk.Frame(self.right_frame, height = 100)
        self.status_frame.pack(side=tk.BOTTOM, padx=10, anchor=tk.NW)
    
        # Create user info at the very bottom
        self.user_info = tk.Label(self.status_frame,
            text="Developed by Hans Verdolaga, MSc Mechatronics 2023",
            font=("Helvetica", 8), justify=tk.LEFT)
        self.user_info.pack(side=tk.BOTTOM, anchor=tk.NW)

        # Create a status label on the bottom
        self.status_label = tk.Label(self.status_frame,
            text="Robot state: STOP", font=("Helvetica", 16), justify=tk.LEFT)
        self.status_label.pack(side=tk.BOTTOM, anchor=tk.NW)

        # Create a user info and input frame on the top
        self.user_frame = tk.Frame(self.right_frame, height = 500)
        self.user_frame.pack(side=tk.TOP, pady=10)
        # self.user_frame.pack_propagate(False)

        # Insert SDU logo on the top right side
        self.logo = Image.open("Implementation\Host_24-04-23\App_assets\SDU_logo.png")
        self.logo = self.logo.resize((200, 54), Image.ANTIALIAS)
        self.logo_photo = ImageTk.PhotoImage(self.logo)
        self.logo_canvas = tk.Canvas(self.user_frame, width=300, height=60)
        self.logo_canvas.create_image(50, 0, image=self.logo_photo, anchor=tk.NW)
        self.logo_canvas.pack(side=tk.TOP)

        # Create title box on the right side
        self.title_label = tk.Label(self.user_frame, 
            text="Gas Localization Robot Control",
            wraplength=300, justify=tk.LEFT,
            font=("Helvetica", 16))
        self.title_label.pack(side=tk.TOP)

        # Create a description box with word wrap on the right side
        self.description_label = tk.Label(self.user_frame, 
            text=("    Braitenberg robot designed to locate the source of a gas plume. "
                "For first setup, run apriltag_setup.py with source and robot reference"
                " videos to calibrate the apriltags. Connect the robot to the same wifi"
                " as the robot for the same MQTT broker and port, then run the robot's "
                "hvMain.py script.\n\n Fill the experiment description and click START.\n "
                "For more information, refer to the README.md file."
            ), 
            wraplength=290, justify=tk.LEFT)
        self.description_label.pack(side=tk.TOP, pady=10, anchor=tk.NW)


        # Create blank spacing of height 10
        self.blank_label = tk.Label(self.user_frame, text="", height=1)
        self.blank_label.pack(side=tk.TOP)

        # Create buttons on the bottom right side
        self.button_frame = tk.Frame(self.user_frame)
        self.button_frame.pack(side=tk.BOTTOM, padx=10, pady=10)
        self.button_start = tk.Button(
            self.button_frame, text="START", 
            command=self.start_callback, 
            font=("Helvetica", 16),
        )
        self.button_stop = tk.Button(
            self.button_frame, text="STOP", 
            command=self.stop_callback, 
            font=("Helvetica", 16),
        )
        self.button_exit = tk.Button(
            self.button_frame, text="EXIT", 
            command=self.exit_callback, 
            font=("Helvetica", 16),
        )

        self.button_start.grid(row=0, column=0, padx=10, pady=10)
        self.button_stop.grid(row=0, column=1, padx=10, pady=10)
        self.button_exit.grid(row=0, column=2, padx=10, pady=10)

        # # Create blank spacing of height 10
        # self.blank_label = tk.Label(self.user_frame, text="", height=1)
        # self.blank_label.pack(side=tk.BOTTOM)

        # Create text input fields on the bottom right side
        self.input_frame = tk.Frame(self.user_frame)
        self.input_frame.pack(side=tk.BOTTOM, padx=10, pady=10, anchor=tk.NW)
        input_label_1 = tk.Label(self.input_frame, text="Run category")
        input_label_2 = tk.Label(self.input_frame, text="Run position")
        input_label_3 = tk.Label(self.input_frame, text="Run iteration")
        input_label_4 = tk.Label(self.input_frame, text="Run custom desc")
        input_label_1.grid(row=0, column=0, sticky="w", ipadx=15)
        input_label_2.grid(row=1, column=0, sticky="w", ipadx=15)
        input_label_3.grid(row=2, column=0, sticky="w", ipadx=15)
        input_label_4.grid(row=3, column=0, sticky="w", ipadx=15)

        self.input_var_1 = tk.StringVar(value="simple_run")
        self.input_var_2 = tk.StringVar(value="0")
        self.input_var_3 = tk.StringVar(value="0")
        self.input_var_4 = tk.StringVar(value="")

        input_entry_1 = tk.Entry(self.input_frame, textvariable=self.input_var_1)
        input_entry_2 = tk.Entry(self.input_frame, textvariable=self.input_var_2)
        input_entry_3 = tk.Entry(self.input_frame, textvariable=self.input_var_3)
        input_entry_4 = tk.Entry(self.input_frame, textvariable=self.input_var_4)

        input_entry_1.grid(row=0, column=1)
        input_entry_2.grid(row=1, column=1)
        input_entry_3.grid(row=2, column=1)
        input_entry_4.grid(row=3, column=1)

        # Create text input title
        self.input_title = tk.Label(self.user_frame, 
            text="Experiment description", font=("Helvetica", 16), justify=tk.LEFT)
        self.input_title.pack(side=tk.BOTTOM, padx=10, anchor=tk.NW)

        # Initialize video capture
        self.video_capture = cv.VideoCapture(captureID, captureArg)
        # print(self.video_capture.get(cv.CAP_PROP_FRAME_WIDTH))
        # print(self.video_capture.get(cv.CAP_PROP_FRAME_HEIGHT))
        self.video_capture.set(cv.CAP_PROP_FRAME_WIDTH, cfg.cameraWidth)
        self.video_capture.set(cv.CAP_PROP_FRAME_HEIGHT, cfg.cameraHeight)
        self.video_capture.set(cv.CAP_PROP_FPS, 20); # set fps before set fourcc
        self.video_capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
        self.video_capture.set(cv.CAP_PROP_BUFFERSIZE, 2)
        
        # Start the video capture
        self.update_video()

    def update_video(self):
        # Below functions lag the playback, not recommended
        # self.video_capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
        # self.video_capture.set(cv.CAP_PROP_BUFFERSIZE, 2)
        # self.video_capture.set(cv.CAP_PROP_FRAME_WIDTH, 608)
        # self.video_capture.set(cv.CAP_PROP_FRAME_HEIGHT, 342)

        # Make a beep at 15 seconds
        if self.beepFlag and (self.expElapsedTime > warmupTime - 1):
            winsound.MessageBeep()
            self.beepFlag = False

        # Read the next frame from the video capture
        ret, frame = self.video_capture.read()
        if ret: # Video-playback-based loop
            # print(frame.shape)
            # cv.imwrite('Implementation\\Host_24-04-23\\image_test\\image.jpg', frame)
            # Experiment timer
            if self.expStartFlag: 
                self.expElapsedTime = time.perf_counter() - self.expStartTime
            else:
                self.expElapsedTime = 0.0

            # Send frame to Apriltag detector and retrieve transformed images
            imgTagged, imgTopview = self.apriltag.detectAndDisplayTags(frame, self.expElapsedTime)
            if len(self.apriltag.currentData) > 1:
                self.lastCoords = self.apriltag.currentData[1]
                self.lastHeading = self.apriltag.currentData[2]
            # else:
            #     self.lastCoords = np.nan
            #     self.lastHeading = np.nan

            # Experiment coordinate logging
            if self.writeApriltagCSV:
                self.apriltag.writeToCSV(self.experimentName)

            # Convert the tagged OpenCV BGR image to a PIL RGB image
            imgTagged = cv.cvtColor(imgTagged, cv.COLOR_BGR2RGB)
            imgTagged = imageResize(imgTagged, height=342)
            imgTagged = Image.fromarray(imgTagged)
            
            # Convert the topview OpenCV BGR image to a PIL RGB image
            imgTopview = cv.cvtColor(imgTopview, cv.COLOR_BGR2RGB)
            # imgTopview = imageResize(imgTopview, height=342)
            imgTopview = Image.fromarray(imgTopview)

            # Update the video canvas with the new image
            self.photo = ImageTk.PhotoImage(imgTagged)
            self.video_canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)

            # Update the topdown canvas with the new image
            self.topdown_photo = ImageTk.PhotoImage(imgTopview)
            self.topdown_canvas.create_image(0, 0, image=self.topdown_photo, anchor=tk.NW)

            # Check if boundary has been crossed
            if (self.boundaryFlag and 
                time.perf_counter() - self.boundaryStartTime > boundaryCheckTime):
                boundaryAlarm = self.apriltag.checkBoundary()
                if boundaryAlarm == 1: 
                    print("BOUNDARY CROSSED!")
                    self.writeApriltagCSV = False
                    self.writeExpCSV = False

                    # Write failure to experiment CSV (TO DO)
                    self.outcome = "failure"
                    writeExpCSV(self.experimentName, self.expElapsedTime, 
                                self.lastCoords, self.lastHeading, self.outcome)

                    self.experimentName = ""
                    self.expStartFlag = False
                    self.beepFlag = False
                    self.expStartTime = 0.0
                    self.boundaryFlag = False
                    self.boundaryStartTime = 0.0
                    self.robotStarted = False
                    self.status_label.config(text=f"Robot state: {robotStateIDs[0]}")

                    # Publish stop message to runtime topic
                    if connected:
                        self.mqttClient.unsubscribe(stateTopic)
                        commandMessage = "stop"
                        self.mqttClient.publish(commandTopic, commandMessage)

            # Check if goal has been reached
            if self.expStartFlag: 
                goalAlarm = self.apriltag.checkGoal()
                if goalAlarm == 1:
                    print("GOAL REACHED!")
                    self.writeApriltagCSV = False
                    self.writeExpCSV = False
                    
                    # Update experiment CSV
                    self.outcome = "success"
                    writeExpCSV(self.experimentName, self.expElapsedTime, 
                                self.lastCoords, self.lastHeading, self.outcome)

                    self.experimentName = ""
                    self.expStartFlag = False
                    self.beepFlag = False
                    self.expStartTime = 0.0
                    self.boundaryFlag = False
                    self.boundaryStartTime = 0.0
                    self.robotStarted = False
                    self.status_label.config(text=f"Robot state: {robotStateIDs[0]}")

                    # Publish stop message to runtime topic
                    if connected:
                        self.mqttClient.unsubscribe(stateTopic)
                        commandMessage = "stop"
                        self.mqttClient.publish(commandTopic, commandMessage)

        # Schedule the next video loop iteration
        self.window.after(10, self.update_video)

    def start_callback(self):
        print("START")
        # Set flags and timers
        self.writeApriltagCSV = True
        self.writeExpCSV = True

        self.runCat = self.input_var_1.get()
        self.runPos = self.input_var_2.get()
        self.runIter = self.input_var_3.get()
        self.runCustom = self.input_var_4.get()

        self.experimentName = (self.runCat + '_p' + str(self.runPos) + '_i' + 
                               str(self.runIter))
        if self.runCustom != "":
            self.experimentName += "_" + self.runCustom

        # Publish start message to command topic
        if connected:
            self.mqttClient.subscribe(stateTopic)
            commandMessage = f"start,{self.experimentName}"
            self.mqttClient.publish(commandTopic, commandMessage)

    def stop_callback(self):
        print("STOP")
        # Set apriltag flags 
        self.writeApriltagCSV = False
        self.writeExpCSV = False

        # Update experiment CSV
        if self.expStartFlag:
            self.outcome = "aborted"
            writeExpCSV(self.experimentName, self.expElapsedTime, 
                        self.lastCoords, self.lastHeading, self.outcome)

        self.experimentName = ""
        self.expStartFlag = False
        self.beepFlag = False
        self.expStartTime = 0.0
        self.boundaryFlag = False
        self.boundaryStartTime = 0.0
        self.robotStarted = False
        self.status_label.config(text=f"Robot state: {robotStateIDs[0]}")

        # Publish stop message to runtime topic
        if connected:
            self.mqttClient.unsubscribe(stateTopic)
            commandMessage = "stop"
            self.mqttClient.publish(commandTopic, commandMessage)

    def exit_callback(self):
        print("EXIT")
        # Set flags
        self.writeApriltagCSV = False
        self.writeExpCSV = False

        # If the run was still in progress, update experiment CSV
        if self.expStartFlag:
            self.outcome = "aborted"
            writeExpCSV(self.experimentName, self.expElapsedTime, 
                        self.lastCoords, self.lastHeading, self.outcome)

        self.expStartFlag = False
        self.expStartTime = 0.0
        self.boundaryFlag = False
        self.boundaryStartTime = 0.0
        self.robotStarted = False

        self.experimentName = ''
        # Publish stop message to runtime topic
        if connected:
            self.mqttClient.unsubscribe(stateTopic)
            commandMessage = "stop"
            self.mqttClient.publish(commandTopic, commandMessage)
        self.window.destroy()
        self.mqttClient.disconnect()
        self.mqttClient.loop_stop()

if __name__ == '__main__':
    # Set up MQTT connection
    connectTimeStart = time.perf_counter()
    client = paho.Client('robotApp')
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port)
    client.loop_start()
    while not connected: # Wait for connection until timeout
        print('Connecting to broker...')
        if time.perf_counter() - connectTimeStart > connectTimeout:
            print('Unable to connect to broker.')
            break
        time.sleep(0.5)

    # Subscribe to robot state topic
    message = ""
    client.publish(commandTopic, message) # Reset message contents

    window = tk.Tk()
    app = App(window, client)
    window.mainloop()
