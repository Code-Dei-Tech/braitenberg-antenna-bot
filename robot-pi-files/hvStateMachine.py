# State machine library
# Hans Verdolaga
# MSc Mechatronics 2023
# MC-F23 Thesis

# -----------------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------------
# Import user libraries
import hvConfiguration as cfg
import hvClasses as hvcl

# Import system libraries
import time
import datetime
import numpy as np
import csv
import os
import inspect
import shutil
from statemachine import StateMachine, State

# -----------------------------------------------------------------------------
# State machine class
# -----------------------------------------------------------------------------
class BraitenbergMachine(StateMachine):
    "Braitenberg machine state machine class"

    # -------------------------------------------------------------------------
    # State machine attributes
    # -------------------------------------------------------------------------
    # Time variables
    timerDuration = 2.5 # Duration for internal timer
    timerStart = 0 # Start time for internal timer
    timerStop = 0 # Stop time for internal timer
    postCastBuffer = 0 # Buffer time after casting

    # Sensor variables
    sigNomLeft = 0 # Nominal left sensor signal
    sigInstLeft = 0 # Instantaneous left sensor signal
    sigIncLeft = 0 # Incremental left sensor signal
    sigDecLeft = 0 # Decremental left sensor signal
    sigNomRight = 0 # Nominal right sensor signal
    sigInstRight = 0 # Instantaneous right sensor signal
    sigIncRight = 0 # Incremental right sensor signal
    sigDecRight = 0 # Decremental right sensor signal

    sigInstMiddle = 0 # Instantaneous middle sensor signal
    sigNomMiddle = 0 # Nominal middle sensor signal
    sigInstCorrLeft = 0 # Instantaneous corrected left sensor signal 
    sigInstCorrRight = 0 # Instantaneous corrected right sensor signal
    sigInstCorrMiddle = 0 # Instantaneous corrected middle sensor signal

    # Servo motor variables
    activateServos = False # Flag to activate servo motors
    srvLeftPos = 0 # Latest left servo motor position
    srvRightPos = 0 # Latest right servo motor position

    # DC Motor variables
    modeDC = 0 # Mode of DC motor activation
        # 0: Off
        # 1: Constant speed
        # 2: Transferring speed
        # 3: Braitenberg adaptive speed
        # 4: Casting speed (single motor action)

    # Command variables
    stateID = 0 # Enumerated state ID

    # States
    STOP = State(initial=True) # stateID = 0
    START= State() # stateID = 1
    SEARCH = State() # stateID = 2
    TRANSFER = State() # stateID = 3
    SURGE = State() # stateID = 4
    CAST = State() # stateID = 5
    TUNE = State() # stateID = 6
    CALIBRATE = State() # stateID = 7

    # Define transitions
    toStart = STOP.to(START) # Manual startup
    toCalibrate = START.to(CALIBRATE) # Nominal signal measuring and calculation
    toSearch = CALIBRATE.to(SEARCH)
    toTransfer = SEARCH.to(TRANSFER) #| CAST.to(TRANSFER)
    toSurge = TRANSFER.to(SURGE) | TUNE.to(SURGE) | CAST.to(SURGE)
    toCast = SURGE.to(CAST)
    toTune = TRANSFER.to(TUNE) | SURGE.to(TUNE) # Manual tuning
    toStop = ( # Manual turn off
        CALIBRATE.to(STOP) | START.to(STOP) | SEARCH.to(STOP) | 
        TRANSFER.to(STOP) | SURGE.to(STOP) | CAST.to(STOP) | TUNE.to(STOP)
    )

    # Wrap all mobile transitions to event call
    cycleMobileState = (
        toSearch | toTransfer | toSurge | toCast # |
        # toTune # Tuning must be manually scheduled
    )

    # -------------------------------------------------------------------------
    # State machine initialization
    # -------------------------------------------------------------------------
    # Manual constructor
    def initialize( # Initialize basic parameters and objects
        self, name: str = "robot",
        runName: str = "run",
        serialTime: float = cfg.serialTime,
        warmupTime: float = cfg.warmupTime,
        calibrateTime: float = cfg.calibrateTime,
        transferTime: float = cfg.transferTime,
        bufferTime: float = cfg.bufferTime,
        genTime: float = cfg.genTime,
        castTime: float = cfg.pathCastTime,
        castPauseTime: float = cfg.pathCastPause,
        bbBehaviour: int = cfg.braitenbergBehaviour,
        bbDynamic: bool = cfg.braitenbergDynamic,
        bbTransferWgt: float = cfg.braitenbergTransferWeight,
        senLeft: hvcl.SensorClass = hvcl.SensorClass('sensorLeft'),
        senRight: hvcl.SensorClass = hvcl.SensorClass('sensorRight'),
        senMiddle: hvcl.SensorClass = hvcl.SensorClass('sensorMiddle'),
        srvMtrLeft: hvcl.ServoMtrClass = hvcl.ServoMtrClass(
            'servoMotorLeft', position=0, antennae=cfg.servoAntennaeType,
            frontPos=cfg.servoFrontLeft, backPos=cfg.servoBackLeft,
            midpoint=cfg.servoMidPointLeft, amplitude=cfg.servoInitAmpLeft,
            defPos=cfg.servoDefaultLeft
            ),
        srvMtrRight: hvcl.ServoMtrClass = hvcl.ServoMtrClass(
            'servoMotorRight', position=1, antennae=cfg.servoAntennaeType,
            frontPos=cfg.servoFrontRight, backPos=cfg.servoBackRight,
            midpoint=cfg.servoMidPointRight, amplitude=cfg.servoInitAmpRight,
            defPos=cfg.servoDefaultRight
            ),
        dcMtrLeft: hvcl.DCMtrClass = hvcl.DCMtrClass('dcMotorLeft'),
        dcMtrRight: hvcl.DCMtrClass = hvcl.DCMtrClass('dcMotorRight'),
    ):
        self.name = name # Unique state machine name
        self.runName = runName # Unique experimental run name
        self.behaviour = bbBehaviour # Braitenberg behaviour model
            # 0 - Fear model - Positive parallel
            # 1 - Aggression model - Positive crossed
            # 2 - Love model - Negative parallel
            # 3 - Exploration model - Negative crossed
        self.dynamic = bbDynamic # Braitenberg dynamic behaviour
            # False - Disable ON/OFF velocity components
            # True - Enable ON/OFF velocity components
        self.transferWgt = bbTransferWgt # Braitenberg transfer weight
            # 0.0 - Full constant velocity in transfer
            # 0.5 - Half Braitenberg, half constant velocity 
            # 1.0 - Full Braitenberg velocity in transfer
        
        # Time variables
        self.serialTime = serialTime # Serial read time in seconds
        self.warmupTime = warmupTime # Sensor warmup time in seconds
        self.calibrateTime = calibrateTime # Sensor offset measurement time in seconds
        self.transferTime = transferTime # Transfer to surge time in seconds
        self.bufferTime = bufferTime # State buffer time in seconds
        self.genTime = genTime # Genetic algorithm learning time in seconds
        self.castTime = castTime # Casting time in seconds
        self.castPause = castPauseTime # Post-cast pause time in seconds

        # Initialize objects
        self.senLeft = senLeft # Left sensor object
        self.senRight = senRight # Right sensor object
        self.senMiddle = senMiddle # Middle sensor object
        self.srvMtrLeft = srvMtrLeft # Left servo motor object
        self.srvMtrRight = srvMtrRight # Right servo motor object
        self.dcMtrLeft = dcMtrLeft # Left DC motor object
        self.dcMtrRight = dcMtrRight # Right DC motor object

        # Configure DC motor objects
        if self.behaviour == 0 or self.behaviour == 1: # Positive polarity
            self.dcMtrLeft.pBehaviour = True
            self.dcMtrRight.pBehaviour = True
        elif self.behaviour == 2 or self.behaviour == 3: # Negative polarity
            self.dcMtrLeft.pBehaviour = False
            self.dcMtrRight.pBehaviour = False
        if self.dynamic: # Dynamic behaviour
            self.dcMtrLeft.dynamic = True
            self.dcMtrRight.dynamic = True
        else: # Simple behaviour
            self.dcMtrLeft.dynamic = False
            self.dcMtrRight.dynamic = False

        # Instantiate genetic algorithm object
        if self.dynamic: 
            fitFunction = hvcl.DCMtrClass.braitenbergDynamic
            nParams = 4
            maxParams = [cfg.dcAlphaMax, cfg.dcKNMax, cfg.dcKIMax, cfg.dcKDMax]
        else: 
            fitFunction = hvcl.DCMtrClass.braitenbergSimple
            nParams = 2
            maxParams = [cfg.dcAlphaMax, cfg.dcKSMax]
        self.genObject = hvcl.GeneticClass(
            "genObject", nParams, maxParams, fitFunction,
            cfg.genPopulationSize, cfg.genCrossoverRate, cfg.genMutationRate
        )

    ### Initialize logfile
    def initializeLogs(
            self, 
            expFileName: str = cfg.expLogFolder + cfg.expLogFileNamePrefix,
            cfgFileName: str = cfg.cfgLogFolder + cfg.cfgLogFileNamePrefix,
            sumFileName: str = cfg.expSumFileName
        ):
        # Initialize description
        description = self.runName

        # Check if parent folder does not exist yet
        if not os.path.exists(cfg.parentFolder): # Make parent log folder
            os.makedirs(cfg.parentFolder)

        # Check if log subfolders do not exist yet
        if not os.path.exists(cfg.expLogFolder): # Make experiment log folder
            os.makedirs(cfg.expLogFolder)
        if not os.path.exists(cfg.cfgLogFolder): # Make config log folder
            os.makedirs(cfg.cfgLogFolder)
        if not os.path.exists(cfg.sensorRawFolder): # Make sensor raw data folder
            os.makedirs(cfg.sensorRawFolder)
        if not os.path.exists(cfg.sensorFilteredFolder): # Make sensor filtered data folder
            os.makedirs(cfg.sensorFilteredFolder)
        if not os.path.exists(cfg.genFolder): # Make genetic algorithm data folder
            os.makedirs(cfg.genFolder)

        # Experimental data logfile
        self.logFile = expFileName + description + ".csv"

        # Configuration parameters logfile
        self.srcCfgFile = inspect.getfile(cfg) # Path to original config file
        self.destCfgFile = cfgFileName + description + ".py" # Destination filename

        # Configuration summary file
        self.sumFile = sumFileName + ".csv"

    # -------------------------------------------------------------------------
    # State machine methods
    # -------------------------------------------------------------------------
    ### Warm up sensors for run
    def startRun(self):
        print("Warming up sensors...")
        self.toStart() # Shift state to start and start timer

    ### Stop existing run
    def stopRun(self):
        print("Shutting down robot...")
        self.toStop()

    ### Update sensor data
    def updateSensorData(self, 
        samplesLeft: list, samplesRight: list, samplesMiddle: list
    ):
        # Record timestamp and description for logging
        timeStamp = time.perf_counter() - self.totalTimerStart
        description = self.runName

        # Register datapoint
        self.senLeft.addPoint(samplesLeft)
        self.senRight.addPoint(samplesRight)
        self.senMiddle.addPoint(samplesMiddle)

        # Register nominal signals
        if self.CALIBRATE.is_active:
            self.senLeft.addNominalPoint(samplesLeft)
            self.senRight.addNominalPoint(samplesRight)
            self.senMiddle.addNominalPoint(samplesMiddle)

        # Log datapoints to CSVs
        self.senLeft.appendRawCSV(timeStamp, samplesLeft, description)
        self.senRight.appendRawCSV(timeStamp, samplesRight, description)
        self.senMiddle.appendRawCSV(timeStamp, samplesMiddle, description)
        if self.senLeft.filtering != 0: 
            self.senLeft.appendFilteredCSV(timeStamp, description)
        if self.senRight.filtering != 0:
            self.senRight.appendFilteredCSV(timeStamp, description)
        if self.senMiddle.filtering != 0:
            self.senMiddle.appendFilteredCSV(timeStamp, description)

        # Get instantaneous sensor samples
        self.sigInstLeft = self.senLeft.getInstantWindow()
        self.sigInstRight = self.senRight.getInstantWindow()
        self.sigInstMiddle = self.senMiddle.getInstantWindow()

        # Calculate increments and decrements for dynamics
        if self.dynamic:
            self.sigIncLeft, self.sigDecLeft = self.senLeft.getDynamics()
            self.sigIncRight, self.sigDecRight = self.senRight.getDynamics()

        # Update sensor state to determine if inside the plume minus nominal signal
        if (not self.STOP.is_active and not self.START.is_active and 
            not self.CALIBRATE.is_active):
            self.senLeft.updateSensorState(buffering = self.buffering())
            self.senRight.updateSensorState(buffering = self.buffering())
            self.senMiddle.updateSensorState(buffering = self.buffering())

    ### Update servo motor trajectory (TO DO)

    ### Update robot state
    def updateRobotState(self):
        # Trigger transition of states
        if not self.buffering() and not self.STOP.is_active:
            # Enter CALIBRATE state after START for bias correction
            if self.START.is_active: self.send("toCalibrate")

            # Enter SEARCH state to find plume
            elif self.CALIBRATE.is_active: self.send("cycleMobileState")

            # Enter TRANSFER state upon finding plume 
            elif self.enterPlume() and (self.SEARCH.is_active):
                print("Entering plume...")
                self.send("cycleMobileState")

            # Enter SURGE state after TRANSFER, TUNE or CAST complete
            elif self.TRANSFER.is_active or self.TUNE.is_active or self.CAST.is_active: 
                self.send("cycleMobileState")

            # Enter CAST state when exiting plume (only when enabled)
            elif (self.exitPlume() and cfg.pathRecovery and
                  (self.TRANSFER.is_active or self.SURGE.is_active)):
                print("Casting...")
                self.send("cycleMobileState")
    
    ### Generate servo control command
    def generateServoCommand(self):
        if self.activateServos:
            # Calculate next servo motor step 
            srvLeft = self.srvMtrLeft.nextStep()
            srvRight = self.srvMtrRight.nextStep()
        else: 
            # Set servo motors to default position
            srvLeft = self.srvMtrLeft.defPos
            srvRight = self.srvMtrRight.defPos
        
        # Save new positions for logging
        self.srvLeftPos = srvLeft
        self.srvRightPos = srvRight

        # Return state and servo command string
        return (f"<1,{self.stateID},{srvLeft},{srvRight}>")

    ### Correct instantaneous sensor signals
    def correctSignals(self, sigLeft: float, sigRight: float, sigMiddle: float):
        if cfg.symCorrection:
            # Apply offset correction to the middle sensor
            if cfg.offsetCorrection: 
                sigInstCorrMiddle = sigMiddle - self.sigNomMiddle
            else: 
                sigInstCorrMiddle = sigMiddle

            # Assign power and polarity of correction
            if cfg.symMinu == 0: # Subtract from left, add to right
                leftFactor = -cfg.symPower
                rightFactor = cfg.symPower
            elif cfg.symMinu == 1: # Subtract from right, add to left
                leftFactor = cfg.symPower
                rightFactor = -cfg.symPower

            sigInstCorrLeft = sigLeft + leftFactor*(
                            cfg.func0(sigInstCorrMiddle, 
                            cfg.symCoeffs[0], cfg.symCoeffs[1]))/2
            sigInstCorrRight = sigRight + rightFactor*(
                            cfg.func0(sigInstCorrMiddle, 
                            cfg.symCoeffs[0], cfg.symCoeffs[1]))/2

        else: # Otherwise, apply originally read signal data
            sigInstCorrLeft = sigLeft
            sigInstCorrRight = sigRight

        if cfg.offsetCorrection: # Apply offset correction
            sigInstCorrLeft = np.max([sigInstCorrLeft - 
                                            self.sigNomLeft, 0])
            sigInstCorrRight = np.max([sigInstCorrRight - 
                                            self.sigNomRight, 0])

        return sigInstCorrLeft, sigInstCorrRight, sigInstCorrMiddle

    ### Prepare signals for Braitenberg
    def prepareSignals(self):
        self.sigInstCorrLeft, self.sigInstCorrRight, self.sigInstCorrMiddle = self.correctSignals(
            self.sigInstLeft, self.sigInstRight, self.sigInstMiddle)
            
        instLeft = self.sigInstCorrLeft
        instRight = self.sigInstCorrRight

        # Assign signal orientation based on Braitenberg setting
        if self.behaviour == 0 or self.behaviour == 2: # Parallel connection
            sigToLeft, incToLeft, decToLeft = instLeft, self.sigIncLeft, self.sigDecLeft
            sigToRight, incToRight, decToRight = instRight, self.sigIncRight, self.sigDecRight
        if self.behaviour == 1 or self.behaviour == 3: # Crossed connection
            sigToLeft, incToLeft, decToLeft = instRight, self.sigIncRight, self.sigDecRight
            sigToRight, incToRight, decToRight = instLeft, self.sigIncLeft, self.sigDecLeft

        # Return input Braitenberg signals
        return sigToLeft, sigToRight, incToLeft, incToRight, decToLeft, decToRight

    ### Generate motor control command
    def generateDCCommand(self):
        if self.modeDC == 0: # No speed
            dcLeft, dcRight = 0, 0

        elif self.modeDC == 1: # Constant speed
            dcLeft, dcRight = self.dcMtrLeft.rpmInit, self.dcMtrRight.rpmInit

        elif self.modeDC == 2 or self.modeDC == 3: # Transfer and surging
            # Prepare signals for Braitenberg
            (sigToLeft, sigToRight, incToLeft, 
             incToRight, decToLeft, decToRight) = self.prepareSignals()

            # Calculate RPM velocities
            dcBraitenbergLeft = self.dcMtrLeft.nextVelocity(
                sigToLeft, incToLeft, decToLeft
            )
            dcBraitenbergRight = self.dcMtrRight.nextVelocity(
                sigToRight, incToRight, decToRight
            )

            # Distribute velocities based on state
            if self.modeDC == 2: # Transfer state - Distribute velocities
                dcLeft = self.transferWgt * dcBraitenbergLeft + (
                        1 - self.transferWgt) * self.dcMtrLeft.rpmInit
                dcRight = self.transferWgt * dcBraitenbergRight + (
                    1 - self.transferWgt) * self.dcMtrRight.rpmInit
            else: # Surge state - Use full Braitenberg
                dcLeft, dcRight = dcBraitenbergLeft, dcBraitenbergRight

        elif self.modeDC == 4: # Casting
            dcLeft = cfg.pathCastSpeed if self.dcMtrLeft.cast else 0
            dcRight = cfg.pathCastSpeed if self.dcMtrRight.cast else 0

        # Save current motor values for logging
        self.dcLeftVel = dcLeft
        self.dcRightVel = dcRight

        # Convert velocities to precision integers for serial writing
        serialDCLeft = int(dcLeft * pow(10, cfg.mtrSpeedPrecision))
        serialDCRight = int(dcRight * pow(10, cfg.mtrSpeedPrecision))

        # Return motor command string
        # Format: <commandType, stateID, servoLeft, servoRight, dcLeft, dcRight>
        return (f"<1,{serialDCLeft},{serialDCRight}>")

    ### Update genetic algorithm fitness and population
    def updateGeneticAlgorithm(self):
        # Prepare signals for fitness function
        (sigToLeft, sigToRight, incToLeft, 
         incToRight, decToLeft, decToRight) = self.prepareSignals()
        signalsLeft = (sigToLeft, incToLeft, decToLeft)
        signalsRight = (sigToRight, incToRight, decToRight)

        # Update fitness of current individual
        self.genObject.updateFitness(signalsLeft, signalsRight)

        # Update population
        self.genObject.updateGeneration()

    ### Log run configuration to file and csv
    def writeToCfgLog(self, duration: float = 0.0,
                      baselineLeft: int = cfg.sensorBaselineLeft, 
                      baselineRight: int = cfg.sensorBaselineRight,
                      baselineMiddle: int = cfg.sensorBaselineMiddle):
        # Copy config python file to destination
        shutil.copyfile(self.srcCfgFile, self.destCfgFile)

        # Fill up row data
        header = ["dateTime", "experimentName", "duration",
                  "braitType", "braitDynamic",
                  "dcRpmInit", "dcRpmMax",
                  "dcAlpha", "dcK", "dcKN", "dcKI", "dcKD",
                  "antennaType", "antennaInitSpeed",
                  "snsBaselineLeft", "snsBaselineRight", "snsBaselineMiddle",
                  "snsSensorLeftID", "snsSensorRightID", "snsSensorMiddleID",
                  "offsetCorrection", "symCorrection",
                  "snsSensorLeftNominal", "snsSensorRightNominal",
                  "snsSymmetryCoeffs", "snsSymmetryPower",
                  "snsSensorLoops", "snsStorageSize", "sensorSigMax", 
                  "snsThreshold",
                  "snsFilterType", "snsFilterCutoff"]

        rowData = []
        rowData.append(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        rowData.append(self.runName)
        rowData.append(duration)
        rowData.append(cfg.braitenbergBehaviour)
        rowData.append(cfg.braitenbergDynamic)
        rowData.append(cfg.dcRpmInit)
        rowData.append(cfg.dcRpmMax)
        rowData.append(cfg.dcAlpha)
        rowData.append(cfg.dcKS)
        rowData.append(cfg.dcKN)
        rowData.append(cfg.dcKI)
        rowData.append(cfg.dcKD)
        rowData.append(cfg.servoAntennaeType)
        rowData.append(cfg.servoInitSpeed)
        rowData.append(baselineLeft)
        rowData.append(baselineRight)
        rowData.append(baselineMiddle)
        rowData.append(cfg.sensorLeftID)
        rowData.append(cfg.sensorRightID)
        rowData.append(cfg.sensorMiddleID)
        rowData.append(cfg.offsetCorrection)
        rowData.append(cfg.symCorrection)
        rowData.append(self.sigNomLeft)
        rowData.append(self.sigNomRight)
        rowData.append(cfg.symCoeffs)
        rowData.append(cfg.symPower)
        rowData.append(cfg.sensorLoops)
        rowData.append(cfg.sensorStorageSize)
        rowData.append(cfg.sensorSigMax)
        rowData.append(cfg.sensorThresholdValue)
        rowData.append(cfg.sensorFiltering)
        rowData.append(cfg.sensorFilterCutOff)

        # Open CSV for logging
        with open(self.sumFile, "a", newline="") as sumFile:
            sumWriter = csv.writer(sumFile, delimiter=",")

            # If log file is empty, write header
            if os.stat(self.sumFile).st_size == 0:
                sumWriter.writerow(header)

            # Write row data
            sumWriter.writerow(rowData)

        # Close CSV
        sumFile.close()

        
    ### Log all conditions to file
    def writeToExpLog(self):
        # Record timestamp for logging
        timeStamp = time.perf_counter() - self.totalTimerStart

        # Fill up row data
        # Intermediate sensor readings
        # Format: <timeStamp, stateID, sensorLeft, sensorRight, servoLeft, servoRight, dcLeft, dcRight>
        header = ["timeStamp", "stateID", 
                  "servoLeft", "servoRight", 
                  "dcLeft", "dcRight",
                  "instantSenLeft", "instantSenRight", "instantSenMiddle",
                  "incSenLeft", "incSenRight",
                  "decSenLeft", "decSenRight",
                  "corrSenLeft", "corrSenRight", "corrSenMiddle"]
        rowData = []
        rowData.append(timeStamp)
        rowData.append(self.stateID)
        rowData.append(self.srvLeftPos)
        rowData.append(self.srvRightPos)
        rowData.append(self.dcLeftVel)
        rowData.append(self.dcRightVel)
        rowData.append(self.sigInstLeft)
        rowData.append(self.sigInstRight)
        rowData.append(self.sigInstMiddle)
        rowData.append(self.sigIncLeft)
        rowData.append(self.sigIncRight)
        rowData.append(self.sigDecLeft)
        rowData.append(self.sigDecRight)
        rowData.append(self.sigInstCorrLeft)
        rowData.append(self.sigInstCorrRight)
        rowData.append(self.sigInstCorrMiddle)

        # Open CSV for logging
        with open(self.logFile, "a", newline="") as logFile:
            logWriter = csv.writer(logFile, delimiter=",")

            # If log file is empty, write header
            if os.stat(self.logFile).st_size == 0:
                logWriter.writerow(header)

            # Write row data
            logWriter.writerow(rowData)

        # Close CSV
        logFile.close()

    # -------------------------------------------------------------------------
    # State machine conditions
    # -------------------------------------------------------------------------
    ### Buffer time condition
    def buffering(self):
        self.timerStop = time.perf_counter()
        if self.timerStop - self.timerStart < self.timerDuration: return True
        else: return False

    ### Robot entering plume condition
    def enterPlume(self):
        # Picking up the signal for the first time
        # Condition called only in SEARCH and CAST states
        if (self.senLeft.aboveThreshold or self.senRight.aboveThreshold):
        # if (self.senMiddle.aboveThreshold):
            return True
        else: return False

    ### Robot leaving plume condition
    def exitPlume(self):
        # Condition called only in TRANSFER state and SURGE states
        if not self.senMiddle.surging: return True
        else: return False

    # -------------------------------------------------------------------------
    # State machine transition actions  
    # -------------------------------------------------------------------------
    ### Transitioning between mobile states
    def after_cycleMobileState(self):
        # Start state machine timer for buffering
        self.timerStart = time.perf_counter()
    
    ### Transitioning to STOP state
    def on_enter_STOP(self):
        self.stateID = 0
        self.totalTimerStart = 0 # Stop experiment timer

        self.activateServos = False # Disable servos
        self.modeDC = 0 # Disable DC motors

    ### Transitioning to START state
    def on_enter_START(self):
        self.stateID = 1
        self.timerDuration = self.warmupTime # Set timer to warmup
        self.timerStart = time.perf_counter() # Start timer
        self.totalTimerStart = time.perf_counter() # Start experiment timer

        self.activateServos = False # Disable servos
        self.modeDC = 0 # Disable DC motors

    ### Transitioning to SEARCH state
    def on_enter_SEARCH(self):
        self.stateID = 2
        self.timerDuration = self.bufferTime # Set timer to buffer
        self.timerStart = time.perf_counter() # Start timer

        self.activateServos = True # Enable servos
        self.modeDC = 1 # Set DC motors to constant speed

    ### Transitioning to TRANSFER state
    def on_enter_TRANSFER(self):
        self.stateID = 3
        self.timerDuration = self.transferTime # Set timer to transfer
        self.timerStart = time.perf_counter()

        self.activateServos = True # Enable servos
        self.modeDC = 2 # Set DC motors to transferring speed

    ### Transitioning to SURGE state
    def on_enter_SURGE(self):
        self.stateID = 4
        self.timerDuration = self.bufferTime + self.postCastBuffer # Set timer to buffer
        self.timerStart = time.perf_counter()

        self.activateServos = True # Enable servos
        self.modeDC = 3 # Set DC motors to Braitenberg adaptive speed

    ### Transitioning to CAST state
    def on_enter_CAST(self):
        self.stateID = 5
        self.timerDuration = self.castTime # Set timer to casting
        self.timerStart = time.perf_counter()
        self.postCastBuffer = self.castPause # Set post-cast pause

        self.activateServos = True # Enable servos
        self.modeDC = 4 # Set one DC motor to casting speed
        
        # Assign which motor to cast
        meanSigLeft = self.senLeft.getInstantWindow(len(self.senLeft.data))
        meanSigRight = self.senRight.getInstantWindow(len(self.senRight.data))

        # Correct signals
        if cfg.offsetCorrection:
            meanCorrSigLeft, meanCorrSigRight, tempMiddle = self.correctSignals(
            meanSigLeft, meanSigRight, self.sigInstMiddle)
        else: 
            meanCorrSigLeft, meanCorrSigRight = meanSigLeft, meanSigRight

        if meanCorrSigLeft > meanCorrSigRight:
            if self.behaviour == 0 or self.behaviour == 2: # Parallel connection
                self.dcMtrLeft.cast = True
                self.dcMtrRight.cast = False
            if self.behaviour == 1 or self.behaviour == 3: # Crossed connection
                self.dcMtrLeft.cast = False
                self.dcMtrRight.cast = True
        else:
            if self.behaviour == 0 or self.behaviour == 2: # Parallel connection
                self.dcMtrLeft.cast = False
                self.dcMtrRight.cast = True
            if self.behaviour == 1 or self.behaviour == 3: # Crossed connection
                self.dcMtrLeft.cast = True
                self.dcMtrRight.cast = False

    ### Exiting from CAST state
    def on_exit_CAST(self):
        # Reset motor cast flags
        self.dcMtrLeft.cast = False
        self.dcMtrRight.cast = False

    ### Transitioning to TUNE state
    def on_enter_TUNE(self):
        self.stateID = 6
        self.timerDuration = self.genTime # Set timer to learn
        self.timerStart = time.perf_counter()

        self.activateServos = True # Enable servos
        self.modeDC = 0 # Disable DC motors

        # Initialize genetic algorithm population
        self.genObject.initPopulation()

    ### Exiting from TUNE state
    def on_exit_TUNE(self):
        # Extract best weights from genetic algorithm object
        weights = self.genObject.getBestIndividual()

        # Update the Braitenberg weights with best values
        if self.dynamic:
            self.dcMtrLeft.updateWeights(
                alpha = weights[0], kN = weights[1], kI = weights[2], kD = weights[3]
            )
            self.dcMtrRight.updateWeights(
                alpha = weights[0], kN = weights[1], kI = weights[2], kD = weights[3]
            )
        else:
            self.dcMtrLeft.updateWeights(
                alpha = weights[0], kS = weights[1]
            )
            self.dcMtrRight.updateWeights(
                alpha = weights[0], kS = weights[1]
            )

        # Preserve best individuals for next TUNE state
        self.genObject.preserveBest()

        # Log best individual to file
        timeStamp = time.perf_counter() - self.totalTimerStart
        self.genObject.writeToGenLog(timeStamp, self.runName)

    ### Transitioning to CALIBRATE state
    def on_enter_CALIBRATE(self):
        self.stateID = 7
        self.timerDuration = self.calibrateTime # Set timer to calibrate
        self.timerStart = time.perf_counter() # Start timer

        self.activateServos = False # Disable servos
        self.modeDC = 0 # Disable DC motors

    ### Exiting from CALIBRATE state
    def on_exit_CALIBRATE(self):
        # Update the nominal sensor values
        self.senLeft.updateNominal()
        self.senRight.updateNominal()
        self.senMiddle.updateNominal()

        if cfg.offsetCorrection:
            self.sigNomLeft = self.senLeft.nominalSig
            self.sigNomRight = self.senRight.nominalSig
            self.sigNomMiddle = self.senMiddle.nominalSig