# Apriltag video coordinate detection, transformation and data recording
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
# Import system libraries
import copy
import time
import datetime
import argparse

import numpy as np
import cv2 as cv
import csv
import os
from pupil_apriltags import Detector

# Import user libraries
import apriltag_config as cfg
import apriltag_coord_functions as coordFunc

# -----------------------------------------------------------------------------
# Global variables
# -----------------------------------------------------------------------------
# Tag IDs
sourceID = 0
robotID = 1
bottomLeftID = 2
bottomRightID = 3
topRightID = 4
topLeftID = 5

# Capture variables
# cap_width = 960
# cap_height = 580

# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------
# Function to define layout of topdown view
def backgroundLayout(image):
    image[:] = (255,255,255) # Set background to white

    # Draw border rectangle
    cv.line(image, cfg.bottomLeftViewCenCoords, cfg.bottomRightViewCenCoords, 
            cfg.topDownBorderColour, 1)
    cv.line(image, cfg.bottomRightViewCenCoords, cfg.topRightViewCenCoords, 
            cfg.topDownBorderColour, 1)
    cv.line(image, cfg.topRightViewCenCoords, cfg.topLeftViewCenCoords, 
            cfg.topDownBorderColour, 1)
    cv.line(image, cfg.topLeftViewCenCoords, cfg.bottomLeftViewCenCoords, 
            cfg.topDownBorderColour, 1)
    
    # Draw source cross
    cv.drawMarker(image, cfg.sourceViewTrueCoords, cfg.topDownSourceColour,
                    markerType=cv.MARKER_CROSS, markerSize=10, thickness=2)

    # Draw source safety radius
    cv.circle(image, cfg.sourceViewTrueCoords, cfg.sourceSafetyDistView,
                cfg.topDownGoalColour, 1, cv.LINE_AA, 0)

    # Draw horizontal safety boundaries
    bottomLeftSafetyCoords = [cfg.hPad, 
                              cfg.vPad+cfg.sideSafetyDistView]
    bottomRightSafetyCoords = [cfg.hPad, 
                               (cfg.topViewHeight-cfg.vPad)-cfg.sideSafetyDistView]
    topRightSafetyCoords = [(cfg.topViewWidth-cfg.hPad),
                            (cfg.topViewHeight-cfg.vPad)-cfg.sideSafetyDistView]
    topLeftSafetyCoords = [(cfg.topViewWidth-cfg.hPad),
                           cfg.vPad+cfg.sideSafetyDistView]
    # Draw top line
    cv.line(image, bottomLeftSafetyCoords, topLeftSafetyCoords,
            cfg.topDownSafetyColour, 1)
    # Draw bottom line
    cv.line(image, bottomRightSafetyCoords, topRightSafetyCoords,
            cfg.topDownSafetyColour, 1)

    # Label source
    cv.putText(image, "Source", [cfg.sourceViewTrueCoords[0], 
                                cfg.sourceViewTrueCoords[1]-15], 
                cv.FONT_HERSHEY_SIMPLEX, 0.75, cfg.topDownSourceColour, 1)

    return image

# Function to draw tag shapes on image
def drawTagShapes(image, center, textPos, corners, msg, imgType):
    if imgType == 0: # Camera viewport
        lineColourA = cfg.cameraLineColourA
        lineColourB = cfg.cameraLineColourB
        textColour = cfg.cameraTextColour
        circleColour = cfg.cameraCircleColour
    else: # Topdown
        lineColourA = cfg.topDownLineColourA
        lineColourB = cfg.topDownLineColourB
        textColour = cfg.topDownTextColour
        circleColour = cfg.topDownCircleColour

    # Draw tag ID
    cv.putText(image, msg, (int(textPos[0]) - 10, int(textPos[1]) - 10), 
               cv.FONT_HERSHEY_SIMPLEX, 0.75, textColour, 1, cv.LINE_AA)

    # Draw tag center
    cv.circle(image, (center[0], center[1]), 5, circleColour, 1)

    # Draw tag shape
    cv.line(image, (corners[0][0], corners[0][1]), # Corner 1 to 2
            (corners[1][0], corners[1][1]), lineColourA, 2)
    cv.line(image, (corners[1][0], corners[1][1]), # Corner 2 to 3
            (corners[2][0], corners[2][1]), lineColourA, 2)
    cv.line(image, (corners[2][0], corners[2][1]), # Corner 3 to 4
            (corners[3][0], corners[3][1]), lineColourB, 2)
    cv.line(image, (corners[3][0], corners[3][1]), # Corner 4 to 1
            (corners[0][0], corners[0][1]), lineColourB, 2)

    return image

def recordAndDrawTags(
    imageCamera,
    imageTopView,
    tags,
    expElapsedTime,
):
    currentData = [expElapsedTime]

    for tag in tags:
        tag_family = tag.tag_family
        tagID = tag.tag_id
        center = tag.center
        corners = tag.corners

        intCenter = [int(center[0]), int(center[1])]
        intCorners = [[int(corner[0]), int(corner[1])] for corner in corners]
        drawTagShapes(imageCamera, intCenter, intCenter, intCorners, str(tagID), 0)

        # Transform robot coordinates to top-down coordinates
        if tagID == 1: # Robot
            # Transform tag dimensions to top-down coordinates (REMOVE)
            transCenter, transCrn = coordFunc.transformSetCoords(
                cfg.robotToViewMatrix, center, corners)
            intTransCenter = [int(transCenter[0]), int(transCenter[1])]
            intTransCorners = [(int(element[0]), 
                           int(element[1])) for element in transCrn]

            # Mark tag location
            cv.circle(imageTopView, (int(transCenter[0]), 
                        int(transCenter[1])), 5, (0, 0, 255), 2)

            # Transform tag dimensions to real coordinates
            realCenter, realCrn = coordFunc.transformSetCoords(
                cfg.robotToRealMatrix, center, corners)

            # Calculate real offset coordinates
            realOffset = np.array(coordFunc.getRotatedOffsetCoords(
                cfg.robotCenterOffset, realCenter, realCrn, (1, 2)))

            # Transform real offset to top-down coordinates
            tempOffset = np.array(coordFunc.transformSingleCoords(
                np.linalg.inv(cfg.robotToRealMatrix), realOffset))
            topdownOffsetCenter = np.array(coordFunc.transformSingleCoords(  
                cfg.robotToViewMatrix, tempOffset))
            
            drawTagShapes(imageTopView, intTransCenter, topdownOffsetCenter, 
                          intTransCorners, 'Robot', 1)
            
            # Calculate robot heading relative to source
            # Get robot direction vector
            tempVector = np.array([coordFunc.getUnitVector(
                realCrn[1].flatten(), realCrn[2].flatten())])
            
            # Measure angle between robot direction vector and horizontal line
            heading = np.array(coordFunc.getAngle(
                tempVector, np.array([[1, 0]])))
            heading = heading * 180 / np.pi # Convert to degrees

            # Output data for CSV writing
            currentData.append((realOffset.flatten()[0],
                            realOffset.flatten()[1]))
            currentData.append(heading)
            
            # Mark new centre of robot
            cv.circle(imageTopView, [int(topdownOffsetCenter.flatten()[0]), 
                                     int(topdownOffsetCenter.flatten()[1])], 
                                     5, (255, 0, 0), 2)
    
    # print(currentData.size)
    # if currentData.size <= 1:
    #     np.append(currentData, np.zeros(3) + np.nan)

    # print(currentData)

    cv.putText(imageCamera,
               "Experiment elapsed time: " + '{:.3f}'.format(expElapsedTime) + "s",
               (10, 40), cv.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2,
               cv.LINE_AA)
    
    cv.putText(imageTopView,
               "Experiment elapsed time: " + '{:.3f}'.format(expElapsedTime) + "s",
               (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1,
               cv.LINE_AA)

    return imageCamera, imageTopView, currentData

class tagDetector:
    def __init__(self):
        # Initialize detector
        self.detector = Detector(
            families='tag36h11', 
            nthreads=1, 
            quad_decimate=1.0, 
            quad_sigma=0.0, 
            refine_edges=1, 
            decode_sharpening=0.25, 
            debug=0
        )
        self.currentData = [] # Saved data of timestamp, coordinates and heading for CSV writing
    
    def detectAndDisplayTags(self, image, expElapsedTime):
        # Define new topdown view
        imageTopView = np.zeros((cfg.topViewHeight,cfg.topViewWidth,3), np.uint8)
        imageTopView[:] = (255,255,255) # Set background to white

        # Initialize capture size
        imageTagged = copy.deepcopy(image)
        imageCamera = cv.cvtColor(copy.deepcopy(image), cv.COLOR_BGR2GRAY)
        tags = self.detector.detect(
            imageCamera,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=None,
        )

        imageTopView = backgroundLayout(imageTopView) # Refresh topdown view
        imageTagged, imageTopView, self.currentData = recordAndDrawTags(
            imageTagged, imageTopView, 
            tags, expElapsedTime
        )

        return imageTagged, imageTopView
    
    # Function to check if robot has crossed any boundary
    def checkBoundary(self):
        alarmFlag = 0
        if len(self.currentData) > 1:
            robotCoords = self.currentData[1]

            if (robotCoords[0] <= 0 or
                robotCoords[0] > cfg.realHeight or
                robotCoords[1] <= cfg.sideSafetyDistance or
                robotCoords[1] > cfg.realWidth - cfg.sideSafetyDistance
            ):
                alarmFlag = 1

        return alarmFlag

    # Function to check if robot has reached goal
    def checkGoal(self):
        goalFlag = 0
        if len(self.currentData) > 1:
            robotCoords = self.currentData[1]

            # Calculate distance between robot and goal
            distance = coordFunc.getDistance(
                robotCoords, cfg.sourceRealTrueCoords
            )
            if distance < cfg.sourceSafetyDistance:
                goalFlag = 1

        return goalFlag
    
    # Function to log timestamp, coordinates and heading to CSV
    def writeToCSV(self, experimentName: str):
        # Prepare row data
        rowCol = []
        rowCol.append(self.currentData[0]) # Timestamp
        if len(self.currentData) > 1:
            rowCol.append(self.currentData[1]) # Robot real coordinates
            rowCol.append(self.currentData[2]) # Robot heading
        else:
            rowCol.append(np.zeros(2) + np.nan)

        csvFile = cfg.logFolder + '/' + experimentName + '.csv'

        # Create parent folder if it does not exist
        if not os.path.exists(cfg.parentFolder):
            os.makedirs(cfg.parentFolder)

        # Create log folder if it does not exist
        if not os.path.exists(cfg.logFolder):
            os.makedirs(cfg.logFolder)

        csvHeader = ['timeStamp','robotRealCenCoords','robotHeading']

        # Write to CSV file
        # Check if file exists
        fileExists = os.path.isfile(csvFile)

        with open(csvFile, 'a', newline='') as csvfile:
            # Create writer object
            writer = csv.writer(csvfile, delimiter=',')

            # Write header if the file does not exist
            if not fileExists:
                writer.writerow(csvHeader)

            # Write data
            writer.writerow(rowCol)