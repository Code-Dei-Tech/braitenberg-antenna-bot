// Arduino tVOC measurement of left, right and middle sensors
// Hans Verdolaga
// MSc Mechatronics 2023
// MC-F23 Thesis

// --------------------------------------------
// Libraries
// --------------------------------------------
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

// --------------------------------------------
// Global variables
// --------------------------------------------
//// Communication
long baudRate = 38400; // Baud rate for serial communication
unsigned int sensorCount = 3; // Number of sensors

//// Serial reading
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
boolean newData = false;

// Arduino board identifier - Sensor (and servo) board
char arduinoName[7] = "CALBRD"; // 3 characters + null terminator

//// Sensors
u32 baselineLeft = 2543489246; // Initialize left baseline
u32 baselineRight = 2461699939; // Initialize right baseline
u32 baselineMiddle = 2517604439; // Initialize middle baseline

//// Command variables
int commandType; // 0 - reconfigure, 1 - control
int servoLeft = 0;
int servoRight = 0;
// int dcLeft = 0;
// int dcRight = 0;
int *ptrCommandArr[] = {&commandType, &servoLeft, &servoRight};

bool verbose = true; // Enable only for testing
char reconfigName[numChars] = {0}; // Name of variable to reconfigure
int reconfigVariable = 0; // Variable to reconfigure
u32 reconfigValue = 0; // Value of variable to reconfigure

//// Servo motors
Servo servoLeftMotor;
Servo servoRightMotor;
int defaultServoLeft = 2000; // Set to sideways position for reset
int defaultServoRight = 1000; // Set to sideways position for reset

// --------------------------------------------
// Helper functions
// --------------------------------------------
void printMessageType(char messageType) {
    // 'd' = data, 'm' = message, 'e' = error
    Serial.print("{");
    Serial.print(messageType);
    Serial.print("}");
}

// --------------------------------------------
// Setup functions
// --------------------------------------------
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
    // Sensor 0
    TCA.openChannel(TCA_CHANNEL_0);
    setupIndSensor(0);
    sgp_set_iaq_baseline(baselineLeft);
    TCA.closeChannel(TCA_CHANNEL_0);

    // Sensor 1
    TCA.openChannel(TCA_CHANNEL_1);
    setupIndSensor(1);
    sgp_set_iaq_baseline(baselineRight);
    TCA.closeChannel(TCA_CHANNEL_1);

    // Sensor 2
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
// void initServos()
// {
//   servoLeftMotor.writeMicroseconds(defaultServoLeft);
//   servoRightMotor.writeMicroseconds(defaultServoRight);
//   delay(1000);
  // Forward position
//   servoLeft.writeMicroseconds(1000);
//   servoRight.writeMicroseconds(2000);
//   delay(1000);
// }

// --------------------------------------------
// Main functions
// --------------------------------------------
//// Reconfigure variables
void reconfigure(int variable, u32 value) {
    if (variable == 8)
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
        baselineMiddle = value;
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
        // Format: <commandType, servoLeft, servoRight>
        // Receive servo motor values
        for (int i = 1; i < 3; i++) 
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
        for (int i = 1; i < 3; i++) 
        {
        Serial.print(*ptrCommandArr[i]);
        if (i < 2){Serial.print(", ");}
        }
        Serial.println(">");
    }
}

//// Servos
//--// Write servo motors
void writeServos(int servoL, int servoR)
{
    servoLeftMotor.writeMicroseconds(servoL);
    servoRightMotor.writeMicroseconds(servoR);
}

//// Sensors
//--// Read individual sensor
void readIndSensor(u16 *dataArray, int sensorIdx)
{
    s16 err = 0;
    u16 tvoc_ppb, co2_eq_ppm;
    //u16 scaled_ethanol_signal, scaled_h2_signal;

    // Read sensor
    // err = sgp_measure_signals_blocking_read(
    //     &scaled_ethanol_signal, &scaled_h2_signal);
    err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);

    if (err == STATUS_OK) {
        dataArray[sensorIdx] = tvoc_ppb;
    } else {
        dataArray[sensorIdx] = 0;
        
        printMessageType('m');
        Serial.print("<");
        Serial.print(arduinoName);
        Serial.print(": Sensor ");
        Serial.print(sensorIdx);
        Serial.println(" error>");
    }
}

//--// Send data
void sendData(u16 *dataArray)
{
    int ptr = 0;
    int bufferSize = 2*(sensorCount*6 - 1 + 2);
    char outbuf[bufferSize];
    outbuf[0] = {0}; // Reset output buffer

    // printMessageType('d');
    // Serial.print("<");
    // for (int i = 0; i < sensorCount; i++) {
    //     Serial.print(dataArray[i]);
    //     if (i < (sensorCount-1)) {Serial.print(",");}
    // }
    // Serial.println(">");
    // Serial.print(dataArray[0]);
    // Serial.print(",");
    // Serial.print(dataArray[1]);
    // Serial.print(",");
    // Serial.println(dataArray[2]);

    // Transform data to string in outbuf
    ptr += sprintf(outbuf+ptr, "{d}<");
    for (int i = 0; i < sensorCount; i++) {
        ptr += sprintf(outbuf+ptr, "%.5u", dataArray[i]);
        if (i < (sensorCount-1)) {ptr += sprintf(outbuf+ptr, ",");}
    }
    ptr += sprintf(outbuf+ptr, ">");

    Serial.println(outbuf);
}

//--// Read all sensors
void readSensors()
{
    u16 dataArray[sensorCount];
    // Reset array
    for (int i = 0; i < sensorCount; i++) {
        dataArray[i] = 0;
    }
        
    // Sensor 0
    TCA.openChannel(TCA_CHANNEL_0);
    readIndSensor(dataArray, 0);
    TCA.closeChannel(TCA_CHANNEL_0);

    // Sensor 1
    TCA.openChannel(TCA_CHANNEL_1);
    readIndSensor(dataArray, 1);
    TCA.closeChannel(TCA_CHANNEL_1);

    // Sensor 2
    TCA.openChannel(TCA_CHANNEL_2);
    readIndSensor(dataArray, 2);
    TCA.closeChannel(TCA_CHANNEL_2);

    // Send data

    sendData(dataArray);
}

// --------------------------------------------
// Arduino executables
// --------------------------------------------
void setup() {
    // Setup serial communication
    Serial.begin(baudRate);
    printMessageType('m');
    Serial.print("<");
    Serial.print(arduinoName);
    Serial.println(": Initializing Arduino...>");

    // Setup sensors
    setupSensors();

    // Setup servos
    setupServos();
    writeServos(defaultServoLeft, defaultServoRight);
}

void loop() {
    // Read sensors
    readSensors();

    // Receive motor control
    recvWithStartEndMarkers();
    // Serial.println(receivedChars);
    if ( newData == true ) {
        strcpy(tempChars, receivedChars);
        parseCommand();
        if (verbose) {showParsedCommand();}
        writeServos(servoLeft, servoRight); // Write to servos
        newData = false;
    }
}
