# Apriltag coordinate transformation functions
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
import numpy as np
import cv2 as cv

# -----------------------------------------------------------------------------
# Functions
# -----------------------------------------------------------------------------
# Get perspective transform matrix
def getPerspectiveTransformMatrix(pts1, pts2):
    M = cv.getPerspectiveTransform(pts1, pts2)
    return M

# Function to transform center coordinates
def transformSingleCoords(matrix, coords):
    coords = np.array(coords).flatten()
    convertCenter = np.array([[coords]], dtype = "float32")
    transCenter = cv.perspectiveTransform(convertCenter, matrix)

    return np.array(transCenter).flatten()

# Function to convert coordinates to strings
def convertToString(center, corners):
    strCenter = f"{center[0]:.3f},{center[1]:.3f}"
    strCorners = []
    for i in range(len(corners)):
        strCorners.append(f"{corners[i][0]:.3f},{corners[i][1]:.3f}")

    return strCenter, strCorners

# Function to transform center and corner coordinates
def transformSetCoords(matrix, center, corners):
    convertCenter = np.array([[center]], dtype = "float32")
    convertCorners = np.array([corners], dtype = "float32")
    transCenter = cv.perspectiveTransform(convertCenter, matrix)
    transCrn = cv.perspectiveTransform(convertCorners, matrix)

    return (np.array(transCenter).flatten(), np.squeeze(transCrn))

# Function to calculate mid point of two coordinates
def getMidPoint(coord1, coord2):
    return (coord1[0] + coord2[0]) / 2, (coord1[1] + coord2[1]) / 2

# Function to calculate direction vector of two coordinates
def getVector(coord1, coord2):
    return [coord2[0] - coord1[0], coord2[1] - coord1[1]]

# Function to calculate distance between two coordinates
def getDistance(coord1, coord2):
    # Convert lists to numpy arrays
    coord1 = np.array(coord1)
    coord2 = np.array(coord2)
    return np.linalg.norm(coord2.flatten() - coord1.flatten())

# Function to calculate unit direction vector of two coordinates
def getUnitVector(coord1, coord2):
    return [(coord2[0] - coord1[0]) / np.linalg.norm(coord2 - coord1),
            (coord2[1] - coord1[1]) / np.linalg.norm(coord2 - coord1)]

# Function to calculate rotation matrix between two 2D vectors
def getRotationMatrix(vec1, vec2):
    a = np.squeeze(vec1)
    b = np.squeeze(vec2)
    left = np.array([[b[0], b[1]], [-b[1], b[0]]])
    right = np.linalg.inv(np.array([[a[0], a[1]], [-a[1], a[0]]]))

    # Return rotation matrix
    return np.squeeze(np.matmul(left, right))

# Function to calculate angle between two 2D vectors
def getAngle(vec1, vec2):
    a = np.squeeze(vec1)
    b = np.squeeze(vec2)
    return np.arctan2(b[1], b[0]) - np.arctan2(a[1], a[0])

# Function to calculate rotated offset coordinates
def getRotatedOffsetCoords(offset, center, corners, crnIdx: tuple):
    # Define horizontal unit vector for angle calculation
    unitHVec = np.array([[1,0]])

    # Calculate unit direction vector of tag side
    unitVec = getUnitVector(corners[crnIdx[0]].flatten(), 
                            corners[crnIdx[1]].flatten())

    # Calculate rotation matrix
    rotMat = getRotationMatrix(unitHVec, unitVec)

    # Calculate rotated offset coordinates
    rotOffset = np.matmul(offset, rotMat)

    # Calculate rotated offset coordinates
    rotOffsetCoords = [center.flatten()[0] + rotOffset[0], 
                       center.flatten()[1] + rotOffset[1]]

    return rotOffsetCoords