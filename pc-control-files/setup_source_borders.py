# Apriltag robot source and border coordinate setup 
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
import copy
import time
import argparse

import numpy as np
import cv2 as cv
import os
from pupil_apriltags import Detector

import apriltag_config as cfg
import apriltag_coord_functions as coordFunc

# Directory
experimentName = 'Perspective_2A'
parentFolder = 'Implementation\\Host_24-04-23\\'

# Configuration
sourceID = 0
robotID = 1
bottomLeftID = 2
bottomRightID = 3
topRightID = 4
topLeftID = 5

video = parentFolder + experimentName + '.mp4'
# outputMatrixPy = parentFolder + 'apriltag_output_matrices.py'
outputPy = parentFolder + 'apriltag_config.py'

# Initialize coordinate variables
# Dictionaries to save all coorinates
sourceImgCoordDict = {'name': 'source', 'id': sourceID, 'list_center': []}
robotImgCoordDict = {'name': 'robot', 'id': robotID, 'list_center': []}
blImgCoordDict = {'name': 'bottomLeft', 'id': bottomLeftID, 'list_center': []}
brImgCoordDict = {'name': 'bottomRight', 'id': bottomRightID, 'list_center': []}
trImgCoordDict = {'name': 'topRight', 'id': topRightID, 'list_center': []}
tlImgCoordDict = {'name': 'topLeft', 'id': topLeftID, 'list_center': []}
dictList = [sourceImgCoordDict, robotImgCoordDict, blImgCoordDict, brImgCoordDict, trImgCoordDict, tlImgCoordDict]

# Coordinate initialization
# Image coordinates
sourceImgCoords = [0, 0]
robotImgCoords = [0, 0]
blImgCoords = [0, 0]
brImgCoords = [0, 0]
trImgCoords = [0, 0]
tlImgCoords = [0, 0]

# Topdown coordinates
sourceTransCoords = [0, 0]
robotTransCoords = [0, 0]
blTransCoords = [0, 0]
brTransCoords = [0, 0]
trTransCoords = [0, 0]
tlTransCoords = [0, 0]

def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--type", type=int, default=0) # 0: source elevation, 1: robot elevation
    parser.add_argument("--video", type=str, default=parentFolder+experimentName+'.mp4')

    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)

    parser.add_argument("--families", type=str, default='tag36h11')
    parser.add_argument("--nthreads", type=int, default=1)
    parser.add_argument("--quad_decimate", type=float, default=2.0)
    parser.add_argument("--quad_sigma", type=float, default=0.0)
    parser.add_argument("--refine_edges", type=int, default=1)
    parser.add_argument("--decode_sharpening", type=float, default=0.25)
    parser.add_argument("--debug", type=int, default=0)

    args = parser.parse_args()

    return args

def main(args, matType, outputType):
    # 引数解析 #################################################################
    # args = get_args()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    families = args.families
    nthreads = args.nthreads
    quad_decimate = args.quad_decimate
    quad_sigma = args.quad_sigma
    refine_edges = args.refine_edges
    decode_sharpening = args.decode_sharpening
    debug = args.debug

    # Import video file ###############################################################
    cap = cv.VideoCapture(args.video)

    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")

    cap.set(cv.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 540)

    # Detector準備 #############################################################
    at_detector = Detector(
        families=families,
        nthreads=nthreads,
        quad_decimate=quad_decimate,
        quad_sigma=quad_sigma,
        refine_edges=refine_edges,
        decode_sharpening=decode_sharpening,
        debug=debug,
    )

    # Prepare coordinate transformation matrix
    # Loop through first 10 frames of video
    for i in range(cfg.refFrames):
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret == True:
            # Resize frame
            # resized = cv.resize(frame, (cfg.cameraWidth, cfg.cameraHeight), interpolation = cv.INTER_AREA)
            # Convert to grayscale
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            # Detect AprilTags (returns an array of detection results)
            tags = at_detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)
            # Loop over all detected tags
            for tag in tags:
                # Extract tag family and tag id
                tag_family = tag.tag_family
                tag_id = tag.tag_id
                # Extract tag center coordinates
                # Extract tag corner coordinates
                # corners = tag.corners
                # corner_01 = (int(corners[0][0]), int(corners[0][1]))
                # corner_02 = (int(corners[1][0]), int(corners[1][1]))
                # corner_03 = (int(corners[2][0]), int(corners[2][1]))
                # corner_04 = (int(corners[3][0]), int(corners[3][1]))
                # Draw tag family and id
                # cv.putText(frame, tag_family + "-" + str(tag_id), center, cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # # Draw tag corners
                # cv.line(frame, corner_01, corner_02, (0, 255, 0), 2)
                # cv.line(frame, corner_02, corner_03, (0, 255, 0), 2)
                # cv.line(frame, corner_03, corner_04, (0, 255, 0), 2)
                # cv.line(frame, corner_04, corner_01, (0, 255, 0), 2)
                # # Draw tag center
                # cv.circle(frame, center, 2, (0, 0, 255), 2)
                # Save coordinates of source, bottom left, bottom right, top right and top left
                for dict in dictList:
                    if tag_id == dict['id']:
                        dict['list_center'].append(tag.center)

    # Scale and calculate average of center coordinates
    # pts1List = []
    for dict in dictList:
        dict['per_center'] = np.mean(dict['list_center'], axis=0) * cfg.scaleFactor
        # print(dict['per_center'] * cfg.scaleFactor)

    # Transform coordinates to real world coordinates
    dictList[0]['real_center'] = coordFunc.transformSingleCoords(cfg.sourceToRealMatrix, 
                                                    dictList[0]['per_center'])

    # Transform coordinates to viewport coordinates
    for dict in dictList:
        if dict['id'] == 1: pass # Skip robot coordinates
        else:
            dict['view_center'] = coordFunc.transformSingleCoords(cfg.sourceToViewMatrix, 
                                                    dict['per_center'])
            # Convert to integers
            dict['view_center'] = dict['view_center'].astype(int)
    
    # Write results to configuration file
    with open(outputPy, 'r') as file:
        filedata = file.read()

    # Edit source coordinates with offset
    # Assuming the source apparatus is placed horizontally
    realTrueCoords = dictList[0]['real_center'] + cfg.sourceCenterOffset
    tempTrueCoords = coordFunc.transformSingleCoords(
        np.linalg.inv(cfg.sourceToRealMatrix), realTrueCoords)
    viewTrueCoords = coordFunc.transformSingleCoords(
        cfg.sourceToViewMatrix, tempTrueCoords)
    viewTrueCoords = viewTrueCoords.astype(int)

    # Write coordinates
    for line in filedata.splitlines():
        if 'sourceRealCenCoords' in line: # Source real coordinates
            filedata = filedata.replace(
                line, f'sourceRealCenCoords = {dictList[0]["real_center"].tolist()}')
        elif 'sourceRealTrueCoords' in line: # Source real true coordinates
            filedata = filedata.replace(
                line, f'sourceRealTrueCoords = {realTrueCoords.tolist()}')
        elif 'sourceViewCenCoords' in line: # Source viewport coordinates
            filedata = filedata.replace(
                line, f'sourceViewCenCoords = {dictList[0]["view_center"].tolist()}')
        elif 'sourceViewTrueCoords' in line: # Source viewport true coordinates
            filedata = filedata.replace(
                line, f'sourceViewTrueCoords = {viewTrueCoords.tolist()}')
        elif 'bottomLeftViewCenCoords' in line: # Bottom left view coordinates
            filedata = filedata.replace(
                line, f'bottomLeftViewCenCoords = {dictList[2]["view_center"].tolist()}')
        elif 'bottomRightViewCenCoords' in line: # Bottom right view coordinates
            filedata = filedata.replace(
                line, f'bottomRightViewCenCoords = {dictList[3]["view_center"].tolist()}')
        elif 'topRightViewCenCoords' in line: # Top right view coordinates
            filedata = filedata.replace(
                line, f'topRightViewCenCoords = {dictList[4]["view_center"].tolist()}')
        elif 'topLeftViewCenCoords' in line: # Top left view coordinates
            filedata = filedata.replace(
                line, f'topLeftViewCenCoords = {dictList[5]["view_center"].tolist()}')

    # Write the file out again
    with open(outputPy, 'w') as file:
        file.write(filedata)

    cap.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    # Retrieve command line arguments
    args = get_args()

    # Write coordinates to configuration file
    for space in [0,1]: # Output type, real or viewport
        main(args, args.type, space)

    