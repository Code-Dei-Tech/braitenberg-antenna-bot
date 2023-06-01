# Custom classes library
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
# Import user libraries
import hvConfiguration as cfg

# Import system libraries
import time
import numpy as np
import csv
import os.path
import random
from scipy.signal import butter,filtfilt

# -----------------------------------------------------------------------------
# Class for sensor storage
# -----------------------------------------------------------------------------
class SensorClass():
    "Class to encapsulate and manipulate sensor data"
    # Class variables
    surging = False # Boolean to indicate if the robot has entered the plume
    aboveThreshold = False # Boolean to indicate if the sensor state is above the threshold
    absenceCount = 0 # Counter to keep track of the number of consecutive absence samples
    nominalSigArray = [] # Array to store the signal values for nominal signal averaging
    nominalSig = 0 # Nominal signal value

    # Constructor
    def __init__(
            self, name: str = "senArray",
            size: int = cfg.sensorStorageSize,
            filtering: int = cfg.sensorFiltering,
            filterSampleFreq: float = cfg.sensorSamplingFrequency,
            filterCutOff: float = cfg.sensorFilterCutOff,
            thresholdValue: int = cfg.sensorThresholdValue,
            absenceLimit: int = cfg.sensorAbsenceLimit
            ):
        # Initialize the class variables
        self.name = name # Name of the object, must be unique for each sensor
        self.filtering = filtering # Data filtering type
        self.filterSampleFreq = filterSampleFreq # Data filtering sample frequency
        self.filterCutOff = filterCutOff # Data filtering cut-off frequency
        self.thresholdValue = thresholdValue # Signal threshold value 
        self.absenceCountLimit = absenceLimit # Number of consecutive absence samples to change state

        # Initialize the data arrays
        self.data = np.zeros(size) # Initialize the data array
        self.filteredData = np.zeros(size) # Initialize the filtered data array

    # Data manipulation methods
    ### Filtering methods
    def butterLowPassFilter(self, data, fs, order = 2):
        nyquist = 0.5 * fs
        normal_cutoff = self.filterCutOff / nyquist
        # Get the filter coefficients 
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = filtfilt(b, a, data)
        return y

    ### Register new datapoint and shift forward
    def addPoint(self, samples: list):
        # Add to list
        self.data = np.append(self.data, samples)

        # Remove last few samples from list
        self.data = np.delete(self.data, range(cfg.sensorLoops))

        # Repeat process for the filtered data
        if self.filtering == 0: # No filtering
            self.filteredData = self.data
        elif self.filtering == 1: # Simple average
            # Append new data, delete old data and apply average filter on original data
            self.filteredData = np.append(self.filteredData, samples)
            self.filteredData = np.delete(self.filteredData, range(cfg.sensorLoops))
            self.filteredData[-cfg.sensorAverageRange:] = np.ones(cfg.sensorAverageRange)*np.mean(self.data[-cfg.sensorAverageRange:])
        elif self.filtering == 2:
            # Append new data and refilter entire array
            tempData = np.copy(self.data)
            tempData = np.append(tempData, samples)
            tempData = np.delete(tempData, range(cfg.sensorLoops))
            self.filteredData = self.butterLowPassFilter(tempData, self.filterSampleFreq)

    ### Register new point for nominal signal averaging
    def addNominalPoint(self, samples: list):
        # Add to list (do not delete any points)
        self.nominalSigArray = np.append(self.nominalSigArray, samples)

    ### Append new datapoint to CSV
    def appendRawCSV(
            self, timeStamp: float, samples: list,
            description: str = "run",
            fileNamePrefix: str = cfg.sensorRawFolder + cfg.sensorRawFileNamePrefix
        ):
        # Define file name and CSV header
        fileName = fileNamePrefix + description + "_" + self.name + ".csv"
        header = ["timeStamp", "value"]

        # Check if file exists
        fileExists = os.path.isfile(fileName)

        # Time substep
        timeSubStep = cfg.serialTime/cfg.sensorLoops
        
        # Append to CSV
        with open(fileName, 'a', encoding='UTF8', newline='') as f:
            writer = csv.writer(f, delimiter=',')

            # Write the header if the file does not exist
            if not fileExists:
                writer.writerow(header)

            # Write the data
            for i in range(len(samples)):
                offsetTime = timeStamp - cfg.serialTime + timeSubStep*(i+1)
                writer.writerow([offsetTime,samples[i]])

        # Close the file
        f.close()

    ### Append new filtered set to CSV
    def appendFilteredCSV(
            self, timeStamp: float, 
            description: str = "run",
            fileNamePrefix: str = cfg.sensorFilteredFolder + cfg.sensorFilteredFileNamePrefix
        ):
        # Define file name and CSV header
        fileName = fileNamePrefix + description + "_" + self.name + ".csv"
        header = ["timeStamp", "filteredValues"]

        # Check if file exists
        fileExists = os.path.isfile(fileName)
        
        # Append to CSV
        with open(fileName, 'a', encoding='UTF8', newline='') as f:
            writer = csv.writer(f, delimiter=',')

            # Write the header if the file does not exist
            if not fileExists:
                writer.writerow(header)

            # Write the data
            temp = np.copy(self.filteredData)
            temp = np.insert(temp, 0, timeStamp)
            writer.writerow(temp)

        # Close the file
        f.close()

    # Value return methods
    ### Return samples given a custom sample size
    def getNSamples(self, nSamples: int = cfg.sensorLoops):
        if nSamples > len(self.filteredData):
            return self.filteredData
        return self.filteredData[-nSamples:]
    
    ### Return last instantaneous sample
    def getInstantLast(self):
        return self.filteredData[-1]
    
    ### Return last instantaneous sample set of serial window
    def getInstantWindow(self, range = cfg.sensorInstantRange):
        return np.mean(self.filteredData[-range:])
    
    ### Return increment and decrements of sample set
    def getDynamics(self):
        # Select data for dynamics calculation
        if self.filtering == 0: 
            original = np.copy(self.data)[-cfg.sensorDynamicRange:]
        else: 
            original = np.copy(self.filteredData)[-cfg.sensorDynamicRange:]

        # Copy and shift data array backwards by one step
        shifted = np.copy(original)
        shifted = np.delete(shifted, shifted.size-1)
        shifted = np.insert(shifted, 0, shifted[0])

        # Calculate the difference between the two arrays
        diff = np.subtract(original, shifted)
        incArr = np.greater(diff, 0) # Get increments
        decArr = np.less(diff, 0) # Get decrements

        # Calculate and return normalized increments and decrements
        return np.sum(incArr)/len(incArr), np.sum(decArr)/len(decArr)
    
    ### Update the nominal sensor value
    def updateNominal(self):
        self.nominalSig = np.mean(self.nominalSigArray[-cfg.sensorNominalRange:])

    # Sensor state update methods
    ### Check signal against threshold and update metrics
    def updateSensorState(
            self,
            thresholdWindow: int = cfg.sensorThresholdRange,
            buffering: bool = False
        ):
        # Select data for threshold crossing calculation
        if self.filtering == 0: detect = np.copy(self.data)
        else: detect = np.copy(self.filteredData)

        # If the signal drop is gradual, account for decrement
        # Check if the signal is above the threshold
        # Get average of threshold window subtracted by nominal signal
        avg = np.mean(detect[-thresholdWindow:]) - self.nominalSig
        inc, dec = self.getDynamics()

        # Check if the robot has entered the plume
        if avg >= self.thresholdValue and not buffering:
            self.aboveThreshold = True
            self.surging = True

        # Reset the absence count if the robot is still inside the plume
        if inc > 0.5 and self.surging == True:
            self.absenceCount = 0

        # Detect if the robot is starting to exit the plume
        if avg < self.thresholdValue and self.surging == True:
            self.absenceCount += 1
        elif dec > 0.5 and self.surging == True:
            self.absenceCount += 1

        # If the absence count exceeds the allowable amount, raise alarm
        if (self.absenceCount >= self.absenceCountLimit and 
            self.surging == True and buffering == False):
            self.aboveThreshold = False
            self.surging = False

# -----------------------------------------------------------------------------
# Class for Braitenberg DC motor control
# -----------------------------------------------------------------------------
class DCMtrClass():
    "Class to wrap DC motor speed control according to Braitenberg algorithm"
    # Class variable
    cast = False # Boolean to indicate if the motor is selected for casting
    
    # Constructor
    def __init__(
            self, name: str = "dcMotor",
            pBehaviour: bool = True,
            dynamic: bool = False,
            sigMax: int = cfg.sensorSigMax,
            # rpmRange: int = cfg.dcRpmRange,
            rpmMax: int = cfg.dcRpmMax,
            rpmInit: int = cfg.dcRpmInit,
            alpha: float = cfg.dcAlpha,
            kS: float = cfg.dcKS,
            kN: float = cfg.dcKN,
            kI: float = cfg.dcKI,
            kD: float = cfg.dcKD,
            ):
        # Initialize the class variables
        self.name = name # Name of the object, must be unique for each motor
        self.pBehaviour = pBehaviour # Braitenberg positive polarity boolean
            # True - Fear and Aggression models
            # False - Love and Exploration models
        self.dynamic = dynamic # Braitenberg dynamics
            # True - Enable ON/OFF velocity components
            # False - Disable ON/OFF velocity components

        # Initialize motor constraints
        self.sigMax = sigMax # Maximum sensor signal value in PPM
        # self.rpmRange = rpmRange # Velocity range in RPM
        self.rpmMax = rpmMax # Maximum allowable speed in RPM
        self.rpmFactor = rpmMax/sigMax # Motor velocity conversion factor
        self.rpmInit = rpmInit # Initial / constant speed in RPM

        # Initialize equation constants
        self.alpha = alpha # Equation scaling constant
        self.kS = kS # Instantaneous signal weight for Simple Braitenberg
        self.kN = kN # Instantaneous signal weight for Dynamic Braitenberg
        self.kI = kI # Incremental signal weight for Dynamic Braitenberg 
        self.kD = kD # Decremental signal weight for Dynamic Braitenberg

    # Reconfiguration
    def updateWeights(
            self,
            alpha: float = cfg.dcAlpha,
            kS: float = cfg.dcKS,
            kN: float = cfg.dcKN,
            kI: float = cfg.dcKI,
            kD: float = cfg.dcKD,
            ):
        # Reconfigure the class variables
        self.alpha = alpha
        self.kS = kS
        self.kN = kN
        self.kI = kI
        self.kD = kD
    
    # Braitenberg algorithm methods
    ### Simple Braitenberg
    @staticmethod
    def braitenbergSimple(
        signals: float, # Single value tuple recast as float
        params: tuple = (cfg.dcAlpha, cfg.dcKS),
        training: bool = False, pBehaviour: bool = True, 
        sigMax: float = cfg.sensorSigMax
    ):
        # Calculate resulting velocity
        signalInstant, signalInc, signalDec = signals
        dcAlpha, dcKS = params
        sI = dcKS*signalInstant

        if pBehaviour: # Positive polarity
            if not training: # Apply limits in normal operation
                return np.min([dcAlpha*sI, sigMax])
            else: # Return unbounded velocity for training
                return dcAlpha*sI
        else: # Negative polarity
            if not training:
                return np.max([dcAlpha*(sigMax - sI), 0])
            else:
                return dcAlpha*(sigMax - sI)
    
    ### Dynamic Braitenberg
    @staticmethod
    def braitenbergDynamic(
        signals: tuple,
        params: tuple = (cfg.dcAlpha, cfg.dcKN, cfg.dcKI, cfg.dcKD),
        training: bool = False, pBehaviour: bool = True, 
        sigMax: float = cfg.sensorSigMax
    ):
        # Calculate resulting velocity
        dcAlpha, dcKN, dcKI, dcKD = params
        signalInstant, signalInc, signalDec = signals
        sI = dcKN*signalInstant + dcKI*signalInc + dcKD*signalDec

        if pBehaviour: # Positive polarity
            if not training: # Apply limits in normal operation
                return np.min([dcAlpha*sI, sigMax])
            else: # Return unbounded velocity for training
                return dcAlpha*sI
        else: # Negative polarity
            if not training:
                return np.max([dcAlpha*(sigMax - sI), 0])
            else:
                return dcAlpha*(sigMax - sI)

    # Return next DC motor velocity
    def nextVelocity(
            self, signalInstant: int, signalInc: float, signalDec: float,
    ):
        # Calculate internal signal component
        # if self.dynamic: sI = self.kN*signalInstant + self.kI*signalInc + self.kD*signalDec
        # else: sI = self.kS*signalInstant

        # print(f'Normal: {self.alpha*self.kS*signalInstant}')
        # print(f'Dynamic: {self.alpha*sI}')

        # Calculate velocity
        if self.dynamic: 
            velocity = self.braitenbergDynamic(
                signals = (signalInstant, signalInc, signalDec),
                params = (self.alpha, self.kN, self.kI, self.kD),
                training=False, pBehaviour=self.pBehaviour, sigMax=self.sigMax
            )
        else:
            velocity = self.braitenbergSimple(
                signals = (signalInstant, 0, 0),
                params = (self.alpha, self.kS),
                training=False, pBehaviour=self.pBehaviour, sigMax=self.sigMax
            )

        # Return RPM velocity with velocity limits
        return np.max([0,np.min([velocity*self.rpmFactor, self.rpmMax])])

# -----------------------------------------------------------------------------
# Class for servo trajectory storage
# -----------------------------------------------------------------------------
class ServoMtrClass():
    "Class to store and update the servo motor trajectory"
    # Class variables
    currentIdx = 0 # Current index of the trajectory

    # Constructor
    def __init__(
            self, name: str = "SrvArray",
            position: int = 0,
            antennae: int = 0,
            serialTime: int = cfg.serialTime,
            maxSpeed: float = cfg.servoMaxSpeed,
            initSpeed: float = cfg.servoInitSpeed,
            frontPos: int = 1000,
            backPos: int = 2000,
            midpoint: int = 1500,
            amplitude: int = 500,
            defPos: int = 1500
            ):
        # Initialize the class variables
        self.name = name # Name of the object, must be unique for each servo
        self.position = position # Current position of the servo
            # 0 - Left position 
            # 1 - Right position
        self.antennaeType = antennae # Antennae type
            # 0 - Fixed position
            # 1 - Fixed uniform speed
            # 2 - Fixed sinusoidal speed
            # 3 - Adaptive Hopf oscillation
        self.serialTime = serialTime # Serial communication time in seconds
        self.maxSpeed = maxSpeed # Maximum speed of the servo in RPM
        self.frontPos = frontPos # Front position of the servo in microseconds
        self.backPos = backPos # Back position of the servo in microseconds
        self.midpoint = midpoint # Midpoint of the servo in microseconds
        self.amplitude = amplitude # Amplitude of the servo in microseconds
        self.defPos = defPos # Default position of the servo in microseconds

        # Limit motor speed to maximum loaded speed
        if initSpeed > maxSpeed: self.initSpeed = maxSpeed
        else: self.initSpeed = initSpeed

        # Initialize trajectory arrays
        if self.antennaeType == 0: # Fixed position at default position
            self.data = np.array([self.defPos])
        elif self.antennaeType == 1: # Fixed uniform speed
            speedRad = self.initSpeed*2*np.pi/60
            totalPeriod = 2*np.pi/speedRad
            nSteps = np.floor(totalPeriod/self.serialTime).astype(np.uint8)
            
            tempA = np.linspace( # Linespace of first half
                self.frontPos, self.backPos, nSteps//2, 
                endpoint=False, dtype=np.uint16
                )
            tempB = np.linspace( # Linespace of second half
                self.backPos, self.frontPos, nSteps - nSteps//2, 
                endpoint=False, dtype=np.uint16
                )
            temp = np.concatenate(
                    (tempA, tempB), axis=None
                ).astype(np.uint16)
            
            # Left position
            if self.position == 0: 
                temp[temp > self.backPos] = self.backPos
                temp[temp < self.frontPos] = self.frontPos
                self.data = temp
            # Right position
            elif self.position == 1:
                temp[temp > self.frontPos] = self.frontPos
                temp[temp < self.backPos] = self.backPos
                self.data = temp

        elif self.antennaeType == 2: # Fixed sinusoidal speed
            speedRad = self.initSpeed*2*np.pi/60
            totalPeriod = 2*np.pi/speedRad

            tLinespace = np.linspace(
                0, totalPeriod - self.serialTime, 
                np.floor(totalPeriod/self.serialTime).astype(np.uint16)
                )
            
            # Left position
            if self.position == 0:
                # Calculate 
                temp = np.array(
                    [self.midpoint + self.amplitude*np.sin(
                        speedRad*t) for t in tLinespace], dtype=np.uint16)
                temp[temp > self.backPos] = self.backPos
                temp[temp < self.frontPos] = self.frontPos
                self.data = temp
            
            # Right position
            elif self.position == 1:
                temp = np.array(
                    [self.midpoint - self.amplitude*np.sin(
                        speedRad*t) for t in tLinespace], dtype=np.uint16)
                temp[temp > self.frontPos] = self.frontPos
                temp[temp < self.backPos] = self.backPos
                self.data = temp

        #elif self.antennaeType == 3: # Adaptive Hopf oscillation

    # Reconfiguration
    def reconfigure(
            self, antennae: int = 0, 
            initSpeed: float = cfg.servoInitSpeed,
            frontPos: int = 1000,
            backPos: int = 2000,
            midpoint: int = 1500,
            amplitude: int = 500,
            ):
        # Reconfigure the class variables
        self.antennaeType = antennae # Antennae type
        self.initSpeed = initSpeed # Initial speed of the servo in RPM
        self.frontPos = frontPos # Front position of the servo in microseconds
        self.backPos = backPos # Back position of the servo in microseconds
        self.midpoint = midpoint # Midpoint of the servo in microseconds
        self.amplitude = amplitude # Amplitude of the servo in microseconds
    
    # Return next step in servo trajectory
    def nextStep(self):
        # Increment index by 1
        self.currentIdx += 1
        # If index out of bounds, reset to 0
        if self.currentIdx >= self.data.shape[0]: self.currentIdx = 0
        # Return next step
        return self.data[self.currentIdx]

    # Update Hopf trajectory TO DO
    #def updateHopf(self, signalInstant: float): 

# -----------------------------------------------------------------------------
# Class for genetic algorithm optimization
# -----------------------------------------------------------------------------
class GeneticClass():
    "Class to manage and exploit genetic optimization"
    # Class variables
    population = [] # Array to store the population
    preserved = np.array([[]]) # Array to store the best previous individuals
    fitnessArray = [] # Array to store the fitness values
    
    # Constructor
    def __init__(
            self, name: str,
            nParams: int,
            maxParams: list,
            fitFunction: callable,
            nPop: int = cfg.genPopulationSize,
            crossover: float = cfg.genCrossoverRate,
            mutation: float = cfg.genMutationRate
    ):
        # Initialize the class variables
        self.name = name
        self.nParams = nParams
        self.maxParams = maxParams
        self.fitFunction = fitFunction
        self.nPop = nPop
        self.crossoverRate = crossover
        self.mutation = mutation

        # Braitenberg polarity
        if cfg.braitenbergBehaviour == 0 or cfg.braitenbergBehaviour == 1:
            self.pBehaviour = True
        else:
            self.pBehaviour = False

    # Genetic algorithm methods
    ### Initialize population
    def initPopulation(self):
        # Initialize the population
        self.population = np.random.rand(self.nPop, self.nParams)
        # Scale the population
        self.population = self.population*self.maxParams
        # Replace some individuals with the preserved individuals
        self.population[0:len(self.preserved),:] = self.preserved if \
            self.preserved.shape[1] > 0 else \
            self.population[0:len(self.preserved),:]

    ### Calculate individual fitness
    def indFitness(self, individual: list, signal: tuple):
        result = self.fitFunction(signal, individual, 
            training = True, pBehaviour = self.pBehaviour)

        # Penalize the fitness of excessive and insufficient fitness
        if result >= cfg.genFitnessUpper:
            return cfg.genFitnessUPenalty 
        elif result <= cfg.genFitnessLower:
            return cfg.genFitnessLPenalty
        else: return result

    ### Update fitness array
    def updateFitness(self, signalsLeft: tuple, signalsRight: tuple):
        # Calculate arrays of individual fitnesses for each side
        fitnessScoresLeft = [self.indFitness(individual, signalsLeft)
                             for individual in self.population]
        fitnessScoresRight = [self.indFitness(individual, signalsRight)
                             for individual in self.population]

        # Calculate the average fitness for each individual
        self.fitnessArray = np.array(
            [np.mean([fitnessScoresLeft[i], fitnessScoresRight[i]])
                for i in range(len(self.population))]
        )

    ### Update to next generation 
    def updateGeneration(self):
        # Select parents for crossover
        parent1, parent2 = random.choices(
            self.population, weights=self.fitnessArray, k=2
        )

        # Perform crossover
        if np.random.rand() < self.crossoverRate:
            offspring = []
            for i in range(self.nPop): # Loop through each individual
                child = []
                for j in range(self.nParams): # Loop through each parameter
                    gene1 = parent1[j]
                    gene2 = parent2[j]
                    if np.random.rand() < 0.5: # Randomly select gene from parent
                        child.append(gene1)
                    else:
                        child.append(gene2)
                offspring.append(child)
        else:
            offspring = self.population[:]

        # Perform mutation
        for i in range(self.nPop): # Loop through each individual
            for j in range(self.nParams): # Loop through each parameter
                if np.random.rand() < self.mutation:
                    offspring[i][j] = np.random.rand()*self.maxParams[j]

        # Update the population
        self.population = np.array(offspring)

    ### Preserve the elite individuals
    def preserveBest(self, ratio: float = cfg.genElite):
        # Get the index of the best individuals
        bestIdx = np.argsort(self.fitnessArray)[-int(self.nPop*ratio):]
        # Store the best individuals
        self.preserved = self.population[bestIdx]

    ### Return best performing individual
    def getBestIndividual(self):
        # Get the index of the best individual below the upper limit
        valid_idx = np.where(self.fitnessArray < cfg.genFitnessUpper)[0]
        bestIdx = valid_idx[np.array(self.fitnessArray[valid_idx]).argmax()]
        #bestIdx = np.argmax(self.fitnessArray)
        # Return the best individual set of parameters
        return self.population[bestIdx]
    
    ### Append best individual to CSV
    def writeToGenLog(
            self, timeStamp: float, 
            description: str,
            fileNamePrefix: str = cfg.genFolder + cfg.genFileNamePrefix
        ):
        # Define file name and CSV header
        fileName = fileNamePrefix + description + ".csv"
        header = ["timeStamp", "bestIndividual"]

        # Check if file exists
        fileExists = os.path.isfile(fileName)

        # Append to CSV
        with open(fileName, 'a', encoding='UTF8', newline='') as f:
            writer = csv.writer(f, delimiter=',')

            # Write the header if the file does not exist
            if not fileExists:
                writer.writerow(header)

            # Write the data
            writer.writerow([timeStamp, self.getBestIndividual()])

        # Close the file
        f.close()