# Evaluation class library
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
# Import user libraries
import apriltag_config as cfg

# Import system libraries
import numpy as np
import pandas as pd
import cv2 as cv
import ast
import os
import copy
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# -----------------------------------------------------------------------------
# Evaluation class
# -----------------------------------------------------------------------------
class EvalClass():
    def __init__(
            self, expName: str, expDate: str, 
            parentFolder: str, 
            warmupTime: float = 20, 
            sourceCoords: np.ndarray = cfg.sourceRealTrueCoords,
            safetyClearance: int = cfg.safetyClearance,
    ):
        # Initialize directories and files
        self.parentFolder = parentFolder
        self.hostFolder = self.parentFolder + 'host_results/' + expDate + '_results/'
        self.hostLogs = self.hostFolder + 'logs/'
        self.robotFolder = self.parentFolder + 'robot_data/' + expDate + '_data/'
        
        # Initialize variables
        self.expName = expName
        self.expDate = expDate
        self.warmupTime = warmupTime
        self.sourceCoords = sourceCoords
        self.sideSafetyDistance = cfg.robotCollisionRadius + safetyClearance
        self.sourceSafetyDistance = cfg.robotCollisionRadius + cfg.sourceCollisionRadius + safetyClearance

        # Initialize dataframe
        resultsFile = self.hostFolder + 'results.csv'
        resultsTotal = pd.read_csv(resultsFile)
        self.results = self.prepareDataframe(
            resultsTotal[resultsTotal['experimentName'].str.startswith(expName)])
        
        # Initialize canvas
        self.padV = 100
        self.padH = 100
        self.canvas = self.prepareTopview()

    def prepareDataframe(self, df: pd.DataFrame):
        # Enumerate outcome column
        df['outcome'] = df['outcome'].apply(lambda x: 1 if x == 'success' else 0)

        # Get only the latest entry of duplicate experiment names
        df = df.drop_duplicates(subset=['experimentName'], keep='last')

        # Convert non-nan lastCoord entries
        df['lastCoord'] = df['lastCoord'].apply(
            lambda x: ast.literal_eval(x) if type(x) == str else x)

        # Add columns for position and iteration
        df['position'] = df['experimentName'].apply(
            lambda x: int(''.join(filter(str.isdigit, x.split('_')[-2]))))
        df['iteration'] = df['experimentName'].apply(
            lambda x: int(''.join(filter(str.isdigit, x.split('_')[-1]))))

        # Loop through all data files based on experiment name and append to dataframe column
        logFiles = os.listdir(self.hostLogs)

        # Remove dataframe rows that have no csv log file
        for name in df['experimentName']:
            if not any(name in s for s in logFiles):
                df = df[df['experimentName'] != name]

        # Prepare new columns for data entry
        df['logTime'] = None
        df['logCoord'] = None
        df['logHeading'] = None
        df['logHeadingRelative'] = None
        df['distanceTravelled'] = 0
        df['spacedCoord'] = None
        df['spacedHeading'] = None
        df['spacedHeadingRelative'] = None
        df['directness'] = None

        df['logTime'] = df['logTime'].astype(object)
        df['logCoord'] = df['logCoord'].astype(object)
        df['logHeading'] = df['logHeading'].astype(object)
        df['logHeadingRelative'] = df['logHeadingRelative'].astype(object)
        df['spacedCoord'] = df['spacedCoord'].astype(object)
        df['spacedHeading'] = df['spacedHeading'].astype(object)
        df['spacedHeadingRelative'] = df['spacedHeadingRelative'].astype(object)

        # Loop through all data files based on experiment name and append to dataframe column
        for logFile in logFiles:
            if logFile.startswith(self.expName):
                logFileName = logFile.split('.')[0]
                logFile = self.hostLogs + logFile
                logData = pd.read_csv(logFile)

                # Remove all entries below warmup time
                logData = logData.drop(
                    logData[logData['timeStamp'] < self.warmupTime].index)

                # Add time series to logTime column
                expNameIdx = df[df['experimentName'] == logFileName].index[0]
                df.at[expNameIdx, 'logTime'] = logData['timeStamp'].values

                # Add coordinates to logCoord column
                df.at[expNameIdx, 'logCoord'] = [
                    (np.fromstring(x.replace("[nan nan]", "(nan,nan)").strip("()"), sep=',')) 
                    for x in logData['robotRealCenCoords'].values
                ]

                # Add headings to logHeading column
                df.at[expNameIdx, 'logHeading'] = logData['robotHeading'].values

                # Replace lastCoord and lastHeading if nan with last known value from log 
                if np.isnan(df.at[expNameIdx,'lastHeading']):
                    lastCoordIdx = [num for num,x in enumerate(
                        df.at[expNameIdx, 'logCoord']) 
                        if not np.isnan(x).any()][-1]
                    df.at[expNameIdx, 'lastCoord'] = tuple(df.at[
                        expNameIdx, 'logCoord'][lastCoordIdx])

                    lastHeadingIdx = pd.DataFrame(df.at[
                        expNameIdx, 'logHeading']).apply(pd.Series.last_valid_index)
                    df.at[expNameIdx, 'lastHeading'] = df.at[
                        expNameIdx, 'logHeading'][lastHeadingIdx]
                    
                # Use logCoords to calculate total distance travelled
                for i in range(len(df.at[expNameIdx, 'logCoord'])):
                    if i == 0:
                        continue
                    else:
                        temp = np.linalg.norm(
                            np.array(df.at[expNameIdx, 'logCoord'][i]) - np.array(df.at[expNameIdx, 'logCoord'][i-1]))
                        if not np.isnan(temp):
                            df.at[expNameIdx, 'distanceTravelled'] += abs(temp)
                    
                # Add relative heading to logHeadingRelative column
                angleSeries = []
                for i in range(len(df.at[expNameIdx, 'logHeading'])):
                    angleSeries.append(
                        df.at[expNameIdx, 'logHeading'][i] + np.arctan2(
                        self.sourceCoords[1] - df.at[expNameIdx, 'logCoord'][i][1],
                        self.sourceCoords[0] - df.at[expNameIdx, 'logCoord'][i][0]
                    ) * 180 / np.pi )
                df.at[expNameIdx, 'logHeadingRelative'] = angleSeries
        
        # Sort by position and iteration
        df = df.sort_values(by=['position', 'iteration'])

        # Remove all rows with invalid positions (nan or greater than 8)
        df = df.drop(df[df['position'] > 8].index)
        df = df.drop(df[df['position'].isnull()].index)

        # Loop over time column and append to spacedCoord column
        for index, row in df.iterrows():
            # Get time series
            # print(row)
            timeSeries = row['logTime']
            coordSeries = row['logCoord']
            headingSeries = row['logHeading']
            headingRelativeSeries = row['logHeadingRelative']

            # Initialize spacedCoord list
            spacedCoord = []
            spacedHeading = []
            spacedHeadingRelative = []

            i = 0
            while i < len(timeSeries):
                if np.isnan(coordSeries[i]).any():
                    i+=1 # Traverse until non-nan value
                else:
                    spacedCoord.append(coordSeries[i])
                    spacedHeading.append(headingSeries[i])
                    spacedHeadingRelative.append(headingRelativeSeries[i])
                    i+=10

            # Append spacedCoord list to dataframe
            df.at[index, 'spacedCoord'] = spacedCoord
            df.at[index, 'spacedHeading'] = spacedHeading
            df.at[index, 'spacedHeadingRelative'] = spacedHeadingRelative

        # Loop over dataframe and calculate directness polar metric
        for index, row in df.iterrows():
            # Get spacedCoord and spacedHeading lists
            spacedCoordList = row['spacedCoord']
            #spacedHeadingList = row['spacedHeading']
            spacedHeadingRelativeList = row['spacedHeadingRelative']

            # Initialize lists
            lengthList = []
            radiusList = []
            angleList = []

            # Calculate directness radius and angle
            for i in range(len(spacedCoordList)):
                if i == 0:
                    lengthList.append(0)
                    radiusList.append(0)
                    angleList.append(0)
                else:
                    lengthList.append(np.linalg.norm(spacedCoordList[i] - spacedCoordList[i-1]))
                    radiusList.append(np.linalg.norm(spacedCoordList[i] - spacedCoordList[i-1])*np.cos(spacedHeadingRelativeList[i] * -np.pi/180))
                    angleList.append(np.linalg.norm(spacedCoordList[i] - spacedCoordList[i-1])*np.sin(spacedHeadingRelativeList[i] * -np.pi/180))

            radius = np.sum(radiusList)/np.sum(lengthList)
            angle = -np.sum(angleList)/np.sum(lengthList)

            # Append directnessLength and directnessHeading list to dataframe
            df.at[index, 'directness'] = np.array([radius, angle])

        return df

    # Prepare topview layout
    def prepareTopview(self):
        # Initialize canvas
        width = 2750
        height = 2000
        canvas = np.ones((height, width, 3), dtype=np.uint8)*255

        # plt.imshow(canvas)
        # plt.axis('off')

        # Draw topdown rectangle with width and height
        cv.rectangle(canvas, (0,0), (width, height), 
                    cfg.topDownBorderColour, 8)

        # Draw source cross
        cv.drawMarker(canvas, (np.array(self.sourceCoords)).astype(np.uint16), (0,0,0), 
                    markerType=cv.MARKER_CROSS, markerSize=100, thickness=4)

        # Draw source saftey radius
        cv.circle(canvas, (np.array(self.sourceCoords)).astype(np.uint16),
                    self.sourceSafetyDistance, cfg.topDownSafetyColour, 4, cv.LINE_AA, 0)

        # Draw horizontal safety boundaries
        topLeftSafetyCoords = [0, self.sideSafetyDistance]
        bottomLeftSafetyCoords = [0, height - self.sideSafetyDistance]
        bottomRightSafetyCoords = [width, height - self.sideSafetyDistance]
        topRightSafetyCoords = [width, self.sideSafetyDistance]

        # Draw top line
        cv.line(canvas, topLeftSafetyCoords, topRightSafetyCoords,
                cfg.topDownSafetyColour, 3)

        # Draw bottom line
        cv.line(canvas, bottomLeftSafetyCoords, bottomRightSafetyCoords,
                cfg.topDownSafetyColour, 3)

        # Label source
        cv.putText(canvas, "Source", [int(cfg.sourceRealTrueCoords[0]-200), 
                                        int(cfg.sourceRealTrueCoords[1])-100], 
                        cv.FONT_HERSHEY_SIMPLEX, 3, cfg.topDownSourceColour, 3)

        # IMAGE PREVIEW
        # Pad array with white border
        return canvas

    # Generate trajectory map
    def generateMap(
            self, image: np.ndarray, dataframe: pd.DataFrame, 
            position: list = None, iteration: list = None,
            successOnly = False
    ):
        # Copy canvas to new image
        generatedMap = copy.deepcopy(image)

        # Copy dataframe based on selected position, iteration or both
        if position is not None and iteration is not None:
            dataframe = dataframe[(dataframe['position'].isin(position)) & 
                                  (dataframe['iteration'].isin(iteration))]
        elif position is not None:
            dataframe = dataframe[dataframe['position'].isin(position)]
        elif iteration is not None:
            dataframe = dataframe[dataframe['iteration'].isin(iteration)]
        else:
            dataframe = dataframe

        # Draw pathlines for all rows in dataframe
        for index, row in dataframe.iterrows():
            # Get spacedCoord list
            spacedCoordList = row['spacedCoord']
            spacedHeadingList = row['spacedHeading']

            # Draw spaced positions and headings on canvas
            for i in range(len(spacedCoordList)):
                # Draw robot circle
                if row['outcome'] == 1:
                    cv.circle(generatedMap, 
                              (np.array(spacedCoordList[i])).astype(np.int16),
                              10, (0,120,0), 4, cv.LINE_AA, 0)
                elif not successOnly:
                    cv.circle(generatedMap, 
                              (np.array(spacedCoordList[i])).astype(np.int16),
                              10, (155,155,155), 2, cv.LINE_AA, 0)
                
                # Draw robot heading
                coord1 = (np.array(spacedCoordList[i])).astype(np.int16)
                coord2 = ((np.array(spacedCoordList[i])).astype(np.int16) + 
                        (np.array([np.cos(spacedHeadingList[i] * -np.pi/180),
                                    np.sin(spacedHeadingList[i] * -np.pi/180)])*100).astype(np.int16))
                
                if row['outcome'] == 1:
                    cv.line(generatedMap, coord1, coord2, (0,120,0), 4, cv.LINE_AA, 0)
                elif not successOnly:
                    cv.line(generatedMap, coord1, coord2, (225,225,225), 2, cv.LINE_AA, 0)

            # Mark last coordinate based on outcome
            if row['outcome'] == 1:
                cv.drawMarker(generatedMap, (np.array(row['lastCoord'])).astype(np.int16), (0,120,0), 
                            markerType=cv.MARKER_DIAMOND, markerSize=100, thickness=8)
            elif not successOnly:
                cv.drawMarker(generatedMap, (np.array(row['lastCoord'])).astype(np.int16), (180,0,0), 
                            markerType=cv.MARKER_DIAMOND, markerSize=100, thickness=8)

        # Return padded image
        return np.pad(generatedMap, ((self.padV, self.padV), 
                                     (self.padH, self.padH), (0,0)), 
                                     mode='constant', constant_values=255)
    
    # Plot map
    def plotMap(
            self, title: str, dataframe: pd.DataFrame, 
            position: list = None, iteration: list = None,
            successOnly = False
    ):
        # Generate map
        generatedMap = self.generateMap(
            self.canvas, dataframe, position, iteration, successOnly)

        # Plot map
        plt.figure(figsize=(4,3))
        plt.title(title, fontsize=10)

        # Set ticks starting from 100
        plt.xticks(np.arange(100, 2950, 500), 
                   ['0', '500', '1000', '1500', '2000', '2500'])
        plt.yticks(np.arange(100, 2200, 500), 
                   ['0', '500', '1000', '1500', '2000'])
        
        # Set axis labels
        plt.xlabel('x (mm)')
        plt.ylabel('y (mm)')

        # Custom legend
        custom_lines = [Line2D([0], [0], marker='o', 
                               color=np.array([0,120,0])/255, 
                               lw=1, markersize=2),
                        Line2D([0], [0], marker='o', 
                               color=np.array([155,155,155])/255, 
                               lw=1, markersize=2),
                        Line2D([0], [0], marker='D', 
                               color=np.array([0,120,0])/255, 
                               markerfacecolor='w', lw=0),
                        Line2D([0], [0], marker='D', 
                               color=np.array([180,0,0])/255, 
                               markerfacecolor='w', lw=0)]
        plt.legend(custom_lines, ['Successful path', 'Failed path', 
                                'Last successful position', 
                                'Last failed position'], 
                                loc='upper left', bbox_to_anchor=(-0.155, -0.2), 
                                ncol=2, fontsize=9)

        # Plot image
        plt.imshow(generatedMap)

    # Plot directness polar metric
    def plotDirectness(
            self, title: str,
            dataframe: pd.DataFrame
    ):
        fig = plt.figure(figsize=(2.5,2.5))
        ax = fig.add_subplot(111, projection='polar')

        # Plot directness polar metric
        for index, row in dataframe.iterrows():
            # Get directness
            directness = row['directness']

            # Plot directness
            if row['outcome'] == 1: # If success
                ax.plot(directness[1], directness[0], 'o', 
                        color=[0/255,160/255,0/255], markersize=2)
            else: # If failure
                ax.plot(directness[1], directness[0], 'o', 
                        color=[210/255,100/255,100/255], markersize=2)
                
        # Custom legend
        custom_lines = [Line2D([0], [0], marker='o', 
                               color=np.array([0,160,0])/255, 
                               lw=0, markersize=3),
                        Line2D([0], [0], marker='o', 
                               color=np.array([210,100,100])/255, 
                               lw=0, markersize=3),
                        Line2D([0], [0], marker='o', 
                               color=np.array([80,80,255])/255, 
                               lw=0, markersize=3)]
        plt.legend(custom_lines, ['Success', 'Failure', 'Straight path'], 
                loc='upper left', bbox_to_anchor=(-0.2, -0.15), 
                ncol=2, fontsize=9)

        # Set axis limits
        ax.set_ylim(0, 1)
        ax.set_xlim(-np.pi, np.pi)

        # Plot ideal straight line
        ax.plot(0,0.975,'o', color=[80/255,80/255,255/255], markersize=3)

        # Set axis labels with degrees and zero at right side
        ax.set_theta_zero_location('E')
        ax.set_xticklabels([r'$180$', r'$210$', r'$240$', r'$270$', 
                            r'$300$', r'$330$', r'$0$', r'$30$', r'$60$', 
                            r'$90$', r'$120$', r'$150$'])
        ax.set_xticks(np.linspace(-np.pi, np.pi, 12, endpoint=False))
        ax.set_yticklabels(['0', r'$0.25$', r'$0.5$', r'$0.75$', r'$1$'], fontsize=6)
        ax.set_yticks(np.linspace(0, 1, 5, endpoint=False))

        # Set grid with dashed lines
        ax.grid(True, linestyle='--')

        # Set title
        ax.set_title(title, y=1.15, fontsize=10)

        # Show plot
        plt.show()

    # Plot total duration
    def plotDuration(
            self, title: str,
            dataframe: pd.DataFrame
    ):
        # Plot duration
        fig = plt.figure(figsize=(3,2.5))
        ax = fig.add_subplot(111)

        # Get mean and std of duration for all positions
        durationMean = dataframe.groupby('position')['duration'].mean()
        durationStd = dataframe.groupby('position')['duration'].std()

        # Plot duration in a bar chart of positions with error bars
        ax.bar(range(0,9), durationMean, yerr=durationStd, 
            ecolor='black', alpha=1, align='center', capsize=3, 
            color=[220/255,220/255,220/255], error_kw={'elinewidth': 0.75})

        # Plot uniform bar chart of warmup time on all positions
        ax.bar(range(0,9), np.ones(9)*20, color=[180/255,180/255,180/255], 
               alpha=1, align='center')

        # Plot single set of duration points per outcome for legend
        sIdx = 0
        sBreak = False
        fIdx = 0
        fBreak = False
        for index, row in dataframe.iterrows():
            if sBreak and fBreak:
                break
            elif row['outcome'] == 1 and sIdx == 0 and not sBreak: # If success
                ax.plot(row['position'], row['duration'], 
                    'x', color=[0/255,140/255,0/255], 
                    markersize=7,linewidth=0.5)
                sIdx = index
                sBreak = True
            elif not fBreak:
                ax.plot(row['position'], row['duration'], 
                    'x', color=[210/255,70/255,70/255], 
                    markersize=7,linewidth=0.5)
                fIdx = index
                fBreak = True

        # Show legend
        if sIdx < fIdx:
            ax.legend(['Success', 'Failure', 'Mean duration', 'Warmup duration'], 
                    loc='upper left', ncol=2, bbox_to_anchor=(-0.275, -0.25), 
                    fontsize=9)
        else:
            ax.legend(['Failure', 'Success', 'Mean duration', 'Warmup duration'], 
                    loc='upper left', ncol=2, bbox_to_anchor=(-0.275, -0.25), 
                    fontsize=9)

        # Plot rest of individual duration points based on outcome
        for index, row in dataframe.iterrows():
            if sIdx == index or fIdx == index:
                continue
            elif row['outcome'] == 1: # If success
                ax.plot(row['position'], row['duration'], 
                    'x', color=[0/255,140/255,0/255], 
                    markersize=7,linewidth=0.5)
            else:
                ax.plot(row['position'], row['duration'], 
                    'x', color=[210/255,70/255,70/255], 
                    markersize=7,linewidth=0.5)

        # Set ticks
        ax.set_xticks(range(0,9))

        # Set axis labels
        ax.set_xlabel('Starting position')
        ax.set_ylabel('Duration (s)')
        ax.set_title(title, y=1, fontsize=10)

        # Show plot
        plt.show()

    # Plot total distance travelled
    def plotDistance(
            self, title: str,
            dataframe: pd.DataFrame
    ):
        # Plot distance
        fig = plt.figure(figsize=(3,2.5))
        ax = fig.add_subplot(111)

        # Get mean and std of distance travelled for all positions
        distanceTravelledMean = dataframe.groupby('position')['distanceTravelled'].mean()
        distanceTravelledStd = dataframe.groupby('position')['distanceTravelled'].std()

        # Plot distance travelled in a bar chart of positions with error bars
        ax.bar(range(0,9), distanceTravelledMean, yerr=distanceTravelledStd, 
            ecolor='black', alpha=1, align='center', capsize=3, 
            color=[220/255,220/255,220/255], error_kw={'elinewidth': 0.75})

        # Plot single set of duration points per outcome for legend
        sIdx = 0
        sBreak = False
        fIdx = 0
        fBreak = False
        for index, row in dataframe.iterrows():
            if sBreak and fBreak:
                break
            elif row['outcome'] == 1 and sIdx == 0 and not sBreak: # If success
                ax.plot(row['position'], row['distanceTravelled'], 
                        'x', color=[0/255,140/255,0/255], markersize=7,linewidth=0.5)
                sIdx = index
                sBreak = True
            elif not fBreak:
                ax.plot(row['position'], row['distanceTravelled'], 
                        'x', color=[210/255,70/255,70/255], markersize=7,linewidth=0.5)
                fIdx = index
                fBreak = True

        # Show legend
        if sIdx < fIdx:
            ax.legend(['Success', 'Failure', 'Mean distance'], 
                      loc='upper left', ncol=2, bbox_to_anchor=(-0.275, -0.25), 
                      fontsize=9)
        else:
            ax.legend(['Failure', 'Success', 'Mean distance'], 
                      loc='upper left', ncol=2, bbox_to_anchor=(-0.275, -0.25), 
                      fontsize=9)

        # Plot rest of individual distance points based on outcome
        for index, row in dataframe.iterrows():
            if sIdx == index or fIdx == index:
                continue
            elif row['outcome'] == 1: # If success
                ax.plot(row['position'], row['distanceTravelled'], 
                        'x', color=[0/255,140/255,0/255], markersize=7,linewidth=0.5)
            else:
                ax.plot(row['position'], row['distanceTravelled'], 
                        'x', color=[210/255,70/255,70/255], markersize=7,linewidth=0.5)

        # Set ticks
        ax.set_xticks(range(0,9))

        # Set axis labels
        ax.set_xlabel('Starting position')
        ax.set_ylabel('Distance travelled (mm)')
        ax.set_title(title, y=1, fontsize=10)

        # Show plot
        plt.show()

    # Plot success rate
    def plotSuccess(
            self, title: str,
            dataframe: pd.DataFrame
    ):
        # Plot success rate
        fig = plt.figure(figsize=(3,2.5))
        ax = fig.add_subplot(111)

        # Plot stack bar chart of success and failure
        ax.bar(range(0,9), dataframe.groupby('position')['outcome'].sum(), 
               color=[75/255,210/255,25/255], alpha = 1.0, align='center')
        ax.bar(range(0,9), 5 - dataframe.groupby('position')['outcome'].sum(), 
               bottom=dataframe.groupby('position')['outcome'].sum(), 
               color=[100/255,0/255,0/255], alpha = 0.2,align='center')

        # Get legend
        ax.legend(['Success', 'Failure'], loc='upper left',
                  ncol=2, bbox_to_anchor=(-0.2, -0.25), fontsize=9)

        # Draw horizontal white lines to separate units
        for i in range(1,5):
            ax.axhline(i, color='white', linewidth=2)

        # Set ticks
        ax.set_xticks(range(0,9))

        # Set axis labels
        ax.set_xlabel('Starting position')
        ax.set_ylabel('Experiment count')
        ax.set_title(title, y=1, fontsize=10)

        # Show plot
        plt.show()