# Apriltag stored configurations 
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
# Import libraries
import numpy as np
import cv2 as cv
import datetime
import os

# -----------------------------------------------------------------------------
# Directory variables
# -----------------------------------------------------------------------------\

parentFolder = os.path.dirname(__file__) + '/' + datetime.datetime.now().strftime("%Y%m%d") + '_results/'
logFolder = parentFolder + 'logs'

# -----------------------------------------------------------------------------
# Image variables
# -----------------------------------------------------------------------------
# Time variables
writeInterval = 0.1 # Write to file every x seconds
refFrames = 30 # Frames for averaging coordinates in calibration

# Default window coordinates
# 540, 960 # Large screen
# 477, 848 # Small screen
# 810, 1440
# 1080, 1920
# 576, 1024
# 702, 1248
# 536, 737 # Same proportion as real world coordinates
defaultHeight, defaultWidth = 342, 608  # Screen coordinate system in pixels
cameraHeight, cameraWidth = 702, 1248 # Camera window size in pixels
scaleFactor = 1.0/(1920/cameraWidth) # Scale factor from 2K to 1K resolution

# Display colours
cameraLineColourA = (255, 0, 0)
cameraLineColourB = (0, 255, 0)
cameraTextColour = (0, 0, 255)
cameraCircleColour = (0, 0, 255)
topDownLineColourA = (0, 0, 0)
topDownLineColourB = (0, 0, 0)
topDownTextColour = (255, 0, 0)
topDownCircleColour = (255, 0, 0)
topDownBorderColour = (150, 150, 150)
topDownSourceColour = (25, 25, 25)
topDownSafetyColour = (120, 120, 240)
topDownGoalColour = (120, 200, 120)

# -----------------------------------------------------------------------------
# Coordinate variables
# -----------------------------------------------------------------------------
# Robot apriltag to center of robot
robotCenterOffset = np.array([210,0]) # in mm
sourceCenterOffset = np.array([-300,0]) # in mm

# Collision zone variables
robotCollisionRadius = 130 # in mm
sourceCollisionRadius = 150 # in mm
safetyClearance = 100 # in mm
sideSafetyDistance = robotCollisionRadius + safetyClearance # in mm
sourceSafetyDistance = robotCollisionRadius + sourceCollisionRadius + safetyClearance # in mm

# Real world coordinates
realHeight, realWidth = 2750, 2000 # Real world coordinate system in mm
realBottomLeft = np.array([0,0]) # Reference from top left of window
realBottomRight = np.array([0,realWidth]) # Bottom left of window
realTopRight = np.array([realHeight,realWidth]) # Bottom right of window
realTopLeft = np.array([realHeight,0]) # Top right of window

# Top view window coordinates
vPad = 30 # Vertical window padding size in pixels
hPad = (int((defaultWidth - defaultHeight*(realHeight/realWidth))/2))+vPad # Horizontal window padding size in pixels
# hPad = 50
topViewHeight, topViewWidth = defaultHeight, defaultWidth # Screen coordinate system in pixels
topViewBottomLeft = np.array([hPad,vPad]) # Reference from top left of window
topViewBottomRight = np.array([hPad,topViewHeight-vPad]) # Bottom left of window
topViewTopRight = np.array([topViewWidth-hPad,topViewHeight-vPad]) # Bottom right of window
topViewTopLeft = np.array([topViewWidth-hPad,vPad]) # Top right of window

# Convert real safety coordinates to screen coordinates
ratio = (topViewHeight-2*vPad)/realWidth
sourceSafetyDistView = int(sourceSafetyDistance*ratio) # Radius of safety zone for topdown view
sideSafetyDistView = int(sideSafetyDistance*ratio) # Vertical safety position for topdown view

# -----------------------------------------------------------------------------
# Volatile variables
# -----------------------------------------------------------------------------
# Perspective transformation matrices (updated by setup_matrices.py)
sourceToRealMatrix = np.array([[0.13509753596755503, -4.689688876611663, 2918.6321277391307], [4.518010622360401, 1.1300311116350623, -1801.1054527868555], [-7.087449729877932e-05, 0.001047087711012422, 1.0]])
robotToRealMatrix = np.array([[0.133881633291013, -4.579851489083182, 2802.0669573716723], [4.380646038528173, 1.122847335718126, -1715.7616011831758], [-6.760764119489208e-05, 0.001043769064964776, 1.0]])
sourceToViewMatrix = np.array([[0.013294366802498054, -0.5999860674323236, 535.26416055603], [0.6349132659987896, 0.19074701959163207, -223.95586998082558], [-7.087449257742216e-05, 0.0010470877476372673, 1.0]])
robotToViewMatrix = np.array([[0.013432354132176192, -0.5838556564198584, 517.8005766976488], [0.6156428606046528, 0.18963454565712434, -211.92238518078656], [-6.760764149504406e-05, 0.0010437690209532358, 1.0]])

# pts1List = []
# pts1List.append(np.array([realBottomLeft], dtype = "float32"))
# pts1List.append(np.array([realBottomRight], dtype = "float32"))
# pts1List.append(np.array([realTopRight], dtype = "float32"))
# pts1List.append(np.array([realTopLeft], dtype = "float32"))

# pts2List = cv.perspectiveTransform(
#                 np.array(pts1List, dtype = "float32"),
robotToRealMatrix = np.array([[0.133881633291013, -4.579851489083182, 2802.0669573716723], [4.380646038528173, 1.122847335718126, -1715.7616011831758], [-6.760764119489208e-05, 0.001043769064964776, 1.0]])

# Stored coordinates for analysis (updated by setup_source_borders.py)
# Real world coordinates
sourceRealCenCoords = [2911.135009765625, 1002.3987426757812]
sourceRealTrueCoords = [2611.135009765625, 1002.3987426757812]
# Top view window coordinates
sourceViewCenCoords = [534, 171]
sourceViewTrueCoords = [489, 171]
bottomLeftViewCenCoords = [98, 30]
bottomRightViewCenCoords = [98, 312]
topRightViewCenCoords = [510, 312]
topLeftViewCenCoords = [510, 30]

