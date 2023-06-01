// Arduino servo and sensor main command loop
// Hans Verdolaga
// MSc Mechatronics 2023
// MC-F23 Thesis

// ----------------------------------------------------------------------------
// Libraries
// ----------------------------------------------------------------------------
// Standard libraries
#include <Arduino.h>
#include <Servo.h>
#include <stdio.h>
#include <stdlib.h>

// Custom libraries
#include <TCA9548A.h> 
#include <sensirion_common.h>
#include <sgp30.h>

//// Multiplexer
# include <Wire.h>
TCA9548A<TwoWire> TCA;
#define WIRE Wire

// ----------------------------------------------------------------------------
// Global variables
// ----------------------------------------------------------------------------
//// Communication
long baudRate = 38400; // Baud rate for serial communication
unsigned long serialTime = 800; // Time between serial reads in ms
unsigned int sensorCount = 4; // Number of sensors

//// Serial reading
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
boolean newData = false;

// Arduino board identifier - Sensor (and servo) board
char arduinoName[7] = "SNSBRD"; // 3 characters + null terminator

//// Command variables
int commandType; // 0 - reconfigure, 1 - control, 2 - stop
int stateID = 0;
int servoLeft = 0;
int servoRight = 0;
// int dcLeft = 0;
// int dcRight = 0;
int *ptrCommandArr[] = {&commandType, &stateID, &servoLeft, &servoRight};

bool verbose = false; // Enable only for testing
char reconfigName[numChars] = {0}; // Name of variable to reconfigure
int reconfigVariable = 0; // Variable to reconfigure
u32 reconfigValue = 0; // Value of variable to reconfigure

//// Sensors
int sensorLoops = 5; // Number of sensor loops per serial communication step
u32 baselineLeft = 2520092980; // Initialize left baseline
u32 baselineRight = 2543556557; // Initialize right baseline
u32 baselineMiddle = 2487588433; // Initialize middle baseline
bool updateBaseline = false; // Flag to update baseline after START state
int updateBaselineTime = 15000; // Time to update baseline at end of START state in milliseconds
bool baselineStartTimer = false; // Flag to start baseline timer to avoid resetting each command
float baselineStartTime = 0; // Tracking time during warmup for baseline setting

//// Servo motors
Servo servoLeftMotor;
Servo servoRightMotor;
int defaultServoLeft = 2000; // Set to sideways position for reset
int defaultServoRight = 1000; // Set to sideways position for reset
int servoLeftPrev = defaultServoLeft; // Previous value of servoLeft
int servoRightPrev = defaultServoRight; // Previous value of servoRight

// ----------------------------------------------------------------------------
// Helper functions
// ----------------------------------------------------------------------------
void printMessageType(char messageType) {
    // 'd' = data, 'm' = message, 'b' = baseline, 'e' = error
    Serial.print("{");
    Serial.print(messageType);
    Serial.print("}");
}

// ----------------------------------------------------------------------------
// Setup functions
// ----------------------------------------------------------------------------
//// Sensors
//--// Setup multiplexer
void setupMultiplexer()
{
    TCA.begin(WIRE);
    TCA.closeAll(); // Close all channels
}

//--// Setup individual sensor
void setupIndSensor(int sensorIdx)
{
    s16 err;
    u16 scaled_ethanol_signal, scaled_h2_signal;

    // Check status of sensor
    while (sgp_probe() != STATUS_OK) {
        printMessageType('m');
        Serial.print("<");
        Serial.print(arduinoName);
        Serial.print(": Sensor ");
        Serial.print(sensorIdx);
        Serial.println(" SGP failed.>");
        while (1);
    }

    // Read H2 and ethanol signals
    err = sgp_measure_signals_blocking_read(
        &scaled_ethanol_signal, &scaled_h2_signal);

    // Check if sensor is ready
    printMessageType('m');
    Serial.print("<");
    Serial.print(arduinoName);
    Serial.print(": Sensor ");
    Serial.print(sensorIdx);
    if (err == STATUS_OK) {
        Serial.println(" ready.>");
    } else {
        Serial.println(" has an error.>");
    }
    err = sgp_iaq_init();
}   

//--// Setup all sensors
void setupSensors()
{
    setupMultiplexer();

    // Left sensor
    TCA.openChannel(TCA_CHANNEL_0);
    setupIndSensor(0);
    sgp_set_iaq_baseline(baselineLeft);
    TCA.closeChannel(TCA_CHANNEL_0);

    // Right sensor
    TCA.openChannel(TCA_CHANNEL_1);
    setupIndSensor(1);
    sgp_set_iaq_baseline(baselineRight);
    TCA.closeChannel(TCA_CHANNEL_1);

    // Middle sensor
    TCA.openChannel(TCA_CHANNEL_2);
    setupIndSensor(2);
    sgp_set_iaq_baseline(baselineMiddle);
    TCA.closeChannel(TCA_CHANNEL_2);
}

//// Servo motors
//--// Attach servos to pins
void setupServos()
{
    servoLeftMotor.attach(5);
    servoRightMotor.attach(6);
}

//--// Set initial position
void initServos()
{
    servoLeftMotor.writeMicroseconds(defaultServoLeft);
    servoRightMotor.writeMicroseconds(defaultServoRight);
//   delay(1000);
  // Forward position
//   servoLeft.writeMicroseconds(1000);
//   servoRight.writeMicroseconds(2000);
//   delay(1000);
}

// ----------------------------------------------------------------------------
// Main functions
// --------------------------------------------------------------------                                                                                 --------
//// Reconfigure variables
void reconfigure(int variable, u32 value) {
    if (variable == 0)
    { // Reconfigure sensor loops
        sensorLoops = value;
        if (verbose) {strncpy(reconfigName, "sensorLoops", numChars);}
    }
    else if (variable == 1)
    { // Reconfigure verbose
        if (value == 0) {verbose = false;}
        else {verbose = true;}
        if (verbose) {strncpy(reconfigName, "verbose", numChars);}
    }
    else if (variable == 2)
    { // Reconfigure baud rate (restart serial monitor)
        baudRate = value;
        if (verbose) {strncpy(reconfigName, "baudRate", numChars);}
        Serial.flush();
        Serial.end();
        Serial.begin(baudRate);
        Serial.flush();
    }
    else if (variable == 3)
    { // Reconfigure servo left
        defaultServoLeft = value;
        if (verbose) {strncpy(reconfigName, "defaultServoLeft", numChars);}
        servoLeftMotor.writeMicroseconds(defaultServoLeft);
    }
    else if (variable == 4)
    { // Reconfigure servo right
        defaultServoRight = value;
        if (verbose) {strncpy(reconfigName, "defaultServoRight", numChars);}
        servoRightMotor.writeMicroseconds(defaultServoRight);
    }
    else if (variable == 5)
    { // Reconfigure serial time
        serialTime = value; // Received time in milliseconds
        if (verbose) {strncpy(reconfigName, "serialTime", numChars);}
    }
    else if (variable == 6)
    { // Reconfigure baseline set flag
        if (value == 0) {updateBaseline = false;}
        else {updateBaseline = true;}
        if (verbose) {strncpy(reconfigName, "updateBaseline", numChars);}
    }
    else if (variable == 7)
    { // Reconfigure baseline time set
        updateBaselineTime = value;
        if (verbose) {strncpy(reconfigName, "updateBaselineTime", numChars);}
    }
    else if (variable == 8)
    { // Reconfigure left sensor baseline
        baselineLeft = value;
        // Set baseline
        TCA.openChannel(TCA_CHANNEL_0);
        sgp_set_iaq_baseline(baselineLeft);
        TCA.closeChannel(TCA_CHANNEL_0);
        if (verbose) {strncpy(reconfigName, "baselineLeft", numChars);}
    }
    else if (variable == 9)
    { // Reconfigure right sensor baseline
        baselineRight = value;
        // Set baseline
        TCA.openChannel(TCA_CHANNEL_1);
        sgp_set_iaq_baseline(baselineRight);
        TCA.closeChannel(TCA_CHANNEL_1);
        if (verbose) {strncpy(reconfigName, "baselineRight", numChars);}
    }
    else if (variable == 10)
    { // Reconfigure middle sensor baseline
        baselineRight = value;
        // Set baseline
        TCA.openChannel(TCA_CHANNEL_2);
        sgp_set_iaq_baseline(baselineMiddle);
        TCA.closeChannel(TCA_CHANNEL_2);
        if (verbose) {strncpy(reconfigName, "baselineMiddle", numChars);}
    }
    else
    { // Error
        printMessageType('m');
        Serial.print("<");
        Serial.print(arduinoName);
        Serial.print(": Reconfigure variable ");
        Serial.print(variable);
        Serial.println(" does not exist.>");
    }
}

//// Serial reading
//--// Receive with markers
void recvWithStartEndMarkers()
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//--// Parse integer
void parseInteger(int **ptrArr, int idx, char *strtokIndx) {
    int tempInt = atoi(strtokIndx);
    *ptrArr[idx] = tempInt;
}

//--// Parse command
void parseCommand() {
    char *strtokIndx; // this is used by strtok() as an index
    char **intPtrIndx; // this is used by strtoul() as an index

    // Receive command type
    strtokIndx = strtok(tempChars,",");
    parseInteger(ptrCommandArr, 0, strtokIndx);

    if (commandType == 0)
    { // Reconfigure
        // Format: <commandType, variable, value>
        strtokIndx = strtok(NULL, ","); 
        reconfigVariable = atoi(strtokIndx);
        strtokIndx = strtok(NULL, ","); 

        reconfigValue = strtoul(strtokIndx, intPtrIndx, 10);
        reconfigure(reconfigVariable, reconfigValue);
    }
    else if (commandType == 1)
    { // Control
        // Format: <commandType, stateID, servoLeft, servoRight>
        // Receive state ID
        strtokIndx = strtok(NULL, ",");
        parseInteger(ptrCommandArr, 1, strtokIndx);

        // Receive servo motor values
        for (int i = 2; i < 4; i++) 
        {
            strtokIndx = strtok(NULL, ",");
            parseInteger(ptrCommandArr, i, strtokIndx);
        }
    }
}

//--// Show parsed command
void showParsedCommand() {
    printMessageType('m');
    Serial.print("<");
    Serial.print(arduinoName);
    Serial.print(": Command ");

    if (commandType == 0)
    { // Reconfigure
        Serial.print("reconfig, ");
        Serial.print(reconfigName);
        Serial.print(" = ");
        Serial.print(reconfigValue);
        Serial.println(">");
    }
    else if (commandType == 1)
    { // Control
        Serial.print("control: ");
        for (int i = 1; i < 4; i++) 
        {
        Serial.print(*ptrCommandArr[i]);
        if (i < 3){Serial.print(", ");}
        }
        Serial.println(">");
    }
}

//// Sensors and servo motors
//--// Read individual sensor
void readIndSensor(long *dataArray, int sensorIdx, int elementIdx)
{
    s16 err = 0;
    u16 tvoc_ppb, co2_eq_ppm;
    //u16 scaled_ethanol_signal, scaled_h2_signal;

    // Read sensor
    // err = sgp_measure_signals_blocking_read(
    //     &scaled_ethanol_signal, &scaled_h2_signal);
    err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
    if (err == STATUS_OK) {
        // dataArray[elementIdx + sensorIdx] = scaled_ethanol_signal;
        dataArray[elementIdx + sensorIdx] = tvoc_ppb;
    } else {
        dataArray[elementIdx + sensorIdx] = 0;

        printMessageType('m');
        Serial.print("<");
        Serial.print(arduinoName);
        Serial.print(": Sensor ");
        Serial.print(sensorIdx);
        Serial.println(" error>");
    }
}

//--// Update servo array values
void updateServoArray(int **servoArray, int servoIdx, 
    int servoValue, int servoPrev)
{
    // Create linespace of servo values
    int diff = servoValue - servoPrev;
    int servoStep = diff / sensorLoops;
    for (int i = 0; i < sensorLoops; i++)
    {
        servoArray[servoIdx][i] = servoPrev + servoStep*i;
    }
}

//--// Write to servo and read sensors in steps
void handleServosAndSensors()
{
    // Initialize variables
    unsigned long stepTime = serialTime / (sensorLoops); // Delay in ms
    int ptr = 0;
    int trajLeft[sensorLoops] = {0}; // Initialize trajectory array
    int trajRight[sensorLoops] = {0}; // Initialize trajectory array
    int bufferSize = 2*(sensorCount*6*sensorLoops - 1 + 2);
    char outbuf[bufferSize];

    // Reset variables
    outbuf[0] = {0}; // Reset output buffer
    long sensorArray[sensorCount*sensorLoops] = {0}; // Initialize sensor array
    int *servoArray[2] = {trajLeft, trajRight}; // Initialize servo pointer array

    // Calculate servo motor steps
    updateServoArray(
        servoArray, 0, servoLeft, servoLeftPrev);
    updateServoArray(
        servoArray, 1, servoRight, servoRightPrev);

    // Loop over sensor loops
    for (int i = 0; i < sensorLoops; i++)
    {
        int elementIdx = i*sensorCount;

        // Servo motors
        // if (servoLeftChanged) {servoLeftMotor.write(servoArray[0][i]);}
        // if (servoRightChanged) {servoRightMotor.write(servoArray[1][i]);}
        servoLeftMotor.write(servoArray[0][i]);
        servoRightMotor.write(servoArray[1][i]);

        // Left sensor
        TCA.openChannel(TCA_CHANNEL_0);
        readIndSensor(sensorArray, 0, elementIdx);
        TCA.closeChannel(TCA_CHANNEL_0);

        // Right sensor
        TCA.openChannel(TCA_CHANNEL_1);
        readIndSensor(sensorArray, 1, elementIdx);
        TCA.closeChannel(TCA_CHANNEL_1);

        // Middle sensor
        TCA.openChannel(TCA_CHANNEL_2);
        readIndSensor(sensorArray, 2, elementIdx);
        TCA.closeChannel(TCA_CHANNEL_2);

        // Serial.println(sensorArray[(i*sensorCount) + 1]);
        // char testoutbuf[22];
        // int testptr = 0;
        // testoutbuf[0] = {0};
        // testptr += sprintf(testoutbuf + testptr,"{m}<%.5lu,%.5lu,%.5lu>",sensorArray[i*sensorCount], sensorArray[(i*sensorCount)+1],
        //         sensorArray[(i*sensorCount)+2]);
        // Serial.println(testoutbuf);

        // Delay
        // delay(stepTime);
    }

    // Prepare output buffer of sensor data
    ptr += sprintf(outbuf+ptr, "{d}<");
    for(int i = 0; i < sensorLoops; i++)
    {
        for (int j = 0; j < sensorCount; j++) {
            ptr += sprintf(outbuf+ptr, "%.5u", sensorArray[i*sensorCount+j]);
            if (j < (sensorCount-1)) {ptr += sprintf(outbuf+ptr, ",");}
        }
        if (i < (sensorLoops-1)) {ptr += sprintf(outbuf+ptr, ";");}
    }
    ptr += sprintf(outbuf+ptr, ">");

    // Update previous values of servos
    servoLeftPrev = servoLeft;
    servoRightPrev = servoRight;

    // Send output buffer
    Serial.println(outbuf);
}

//--// Write to servo and get sensor baseline in steps
// void handleServosAndBaselines()
// {
//     // Initialize variables
//     unsigned long stepTime = serialTime / (sensorLoops); // Delay in ms
//     int ptr;
//     int trajLeft[sensorLoops] = {0}; // Initialize trajectory array
//     int trajRight[sensorLoops] = {0}; // Initialize trajectory array
//     int bufferSize = 2*(2*6*sensorLoops - 1 + 2);
//     char outbuf[bufferSize];

//     int *servoArray[2] = {trajLeft, trajRight}; // Initialize servo pointer array
//     //memset(sensorArray, 0, sizeof(sensorArray)); // Reset sensor array
//     //memset(servoArray, 0, sizeof(servoArray)); // Reset servo array

//     // Calculate servo motor steps
//     updateServoArray(
//         servoArray, 0, servoLeft, servoLeftPrev);
//     updateServoArray(
//         servoArray, 1, servoRight, servoRightPrev);

//     // Loop over sensor loops
//     for (int i = 0; i < sensorLoops; i++)
//     {   
//         // Reset for next loop
//         int elementIdx = i*2;

//         ptr = 0;
//         outbuf[0] = {0}; // Reset output buffer

//         // Servo motors
//         servoLeftMotor.write(servoArray[0][i]);
        
//         // Left sensor
//         TCA.openChannel(TCA_CHANNEL_0);
//         readIndBaseline(baselineLeft, 0, elementIdx);
//         TCA.closeChannel(TCA_CHANNEL_0);

//         servoRightMotor.write(servoArray[1][i]);

//         // Right sensor
//         TCA.openChannel(TCA_CHANNEL_1);
//         readIndBaseline(baselineRight, 1, elementIdx);
//         TCA.closeChannel(TCA_CHANNEL_1);

//         // Write to output
//         // printMessageType('m');
//         // Serial.print("<");
//         // Serial.print(arduinoName);
//         // Serial.print(": ");
//         // Serial.print(baselineLeft);
//         // Serial.print(", ");
//         // Serial.print(baselineRight);
//         // Serial.println(">");
//     }
//     // Update previous values of servos
//     servoLeftPrev = servoLeft;
//     servoRightPrev = servoRight;
// }

// ----------------------------------------------------------------------------
// Arduino executables
// ----------------------------------------------------------------------------
void setup() {
    // Serial
    Serial.begin(baudRate);
    Serial.setTimeout(3000);
    printMessageType('m');
    Serial.print("<");
    Serial.print(arduinoName);
    Serial.println(": Initializing Arduino...>");

    // Sensors
    setupSensors();

    // Servo motors
    setupServos();
    initServos();
}

void loop() {
    // Start timer
    unsigned long startTime = millis();

    // Receive motor control or configuration command
    recvWithStartEndMarkers();
    // Serial.println(receivedChars);
    if ( newData == true ) {
        strcpy(tempChars, receivedChars);
        parseCommand();
        if (verbose) {showParsedCommand();}
        newData = false;

        // Handle command based on state
        if (commandType == 1)
        {
            if (stateID != 0){ // Not in STOP state
                // Handle servos and sensors
                handleServosAndSensors();
            }
        }

        // Record timestamp and print time difference
        if (verbose)
        {
            unsigned long stopTime = millis(); // Stop timer
            unsigned long diffTime = stopTime - startTime;
            // Print time difference message
            printMessageType('m');
            Serial.print("<");
            Serial.print(arduinoName);
            Serial.print(": Time difference: ");
            Serial.print(diffTime);
            Serial.println(" ms>");
        }

    }

}
