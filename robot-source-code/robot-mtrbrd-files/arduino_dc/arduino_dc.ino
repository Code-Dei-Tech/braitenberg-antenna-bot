// Arduino DC motor main command loop
// Hans Verdolaga
// MSc Mechatronics 2023
// MC-F23 Thesis

// ----------------------------------------------------------------------------
// Libraries
// ----------------------------------------------------------------------------
// Standard libraries
#include <Arduino.h>
#include <util/atomic.h>
// #include <PIDController.h>

// ----------------------------------------------------------------------------
// Global variables
// ----------------------------------------------------------------------------
long baudRate = 38400; // Baud rate for serial communication
long lastRecvTime = 0; // Time of last received command

//// Serial reading
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
boolean newData = false;

// Arduino board identifier - DC motor board
char arduinoName[7] = "MTRBRD"; // 3 characters + null terminator

//// Command variables
int commandType; // 0 - reconfigure, 1 - control, 2 - stop
double speedLeft = 0;
double speedRight = 0;
double *ptrSpeedArr[] = {&speedLeft, &speedRight};

int speedPrecision = 3; // Number of digits after decimal point

bool verbose = false; // Enable only for testing
char reconfigName[numChars] = {0}; // Name of variable to reconfigure
int reconfigVariable = 0; // Variable to reconfigure
u32 reconfigValue = 0; // Value of variable to reconfigure

int configPrecision = 4; // Number of digits after decimal point

//// DC motors
#define enR 8
#define inR1 9
#define inR2 10

#define inL1 11
#define inL2 12
#define enL 13

//// DC encoders
#define encoderLA 18
#define encoderLB 19
#define encoderRA 20
#define encoderRB 21
float ppr = 8*120;
long leftPrevT = 0;
long rightPrevT = 0;
int leftPosPrev = 0;
int rightPosPrev = 0;
volatile int leftPos_i = 0;
volatile int rightPos_i = 0;

//// Filtering
float a = 0.854; // Filter coefficient of previous filtered value
float b0 = 0.0728; // Filter coefficient of current raw value
float b1 = 0.0728; // Filter coefficient of previous raw value

float leftV1Filt = 0;
float leftVPrev = 0;
float leftV2Filt = 0;
float leftV2Prev = 0;
float rightV1Filt = 0;
float rightVPrev = 0;
float rightV2Filt = 0;
float rightV2Prev = 0;

//// PID controller
float kP = 7; // Proportional gain
float kI = 5; // Integral gain
float kD = 0.1; // Derivative gain

float leftEIntegral = 0; // Left integral variable
float leftEPrev = 0; // Left previous error
float rightEIntegral = 0; // Right integral variable
float rightEPrev = 0; // Right previous error

// ----------------------------------------------------------------------------
// Helper functions
// ----------------------------------------------------------------------------
void printMessageType(char messageType) {
    // 'd' = data, 'm' = message, 'e' = error
    Serial.print("{");
    Serial.print(messageType);
    Serial.print("}");
}

// ----------------------------------------------------------------------------
// Setup functions
// ----------------------------------------------------------------------------
//// DC motors
void setupDC()
{
    // Set pins as outputs
    pinMode(enL, OUTPUT);
    pinMode(inL1, OUTPUT);
    pinMode(inL2, OUTPUT);
    pinMode(inR1, OUTPUT);
    pinMode(inR2, OUTPUT);
    pinMode(enR, OUTPUT);

    // Turn off motors
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, LOW);
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, LOW);
}

//// DC encoders
void setupEncoders()
{
    // Set pins as inputs
    pinMode(encoderLA, INPUT);
    pinMode(encoderLB, INPUT);
    pinMode(encoderRA, INPUT);
    pinMode(encoderRB, INPUT);

    // Set interrupts
    attachInterrupt(
        digitalPinToInterrupt(encoderLA), 
        encoderLeft, 
        RISING
    );
    attachInterrupt(
        digitalPinToInterrupt(encoderRA), 
        encoderRight,
        RISING
    );
}

// ----------------------------------------------------------------------------
// Main functions
// ----------------------------------------------------------------------------
//// Reconfigure variables
void reconfigure(int variable, u32 value) {
    if (variable == 0)
    { // Reconfigure verbose
        if (value == 0) {verbose = false;}
        else {verbose = true;}
        if (verbose) {strncpy(reconfigName, "verbose", numChars);}
    }
    else if (variable == 1)
    { // Reconfigure baud rate (restart serial monitor)
        baudRate = value;
        if (verbose) {strncpy(reconfigName, "baudRate", numChars);}
        Serial.flush();
        Serial.end();
        Serial.begin(baudRate);
        Serial.flush();
    }
    else if (variable == 2)
    { // Reconfigure incoming motor speed precision digits
        speedPrecision = value;
        if (verbose) {strncpy(reconfigName, "speedPrecision", numChars);}
    }
    else if (variable == 3)
    { // Reconfigure incoming reconfiguration precision digits
        configPrecision = value;
        if (verbose) {strncpy(reconfigName, "configPrecision", numChars);}
    }
    else if (variable == 4)
    { // Reconfigure proportional gain after dividing by power of configPrecision
        kP = (float) value/pow(10,configPrecision);
        if (verbose) {strncpy(reconfigName, "kP", numChars);}
    }
    else if (variable == 5)
    { // Reconfigure integral gain after dividing by power of configPrecision
        kI = (float) value/pow(10,configPrecision);
        if (verbose) {strncpy(reconfigName, "kI", numChars);}
    }
    else if (variable == 6)
    { // Reconfigure derivative gain after dividing by power of configPrecision
        kD = (float) value/pow(10,configPrecision);
        if (verbose) {strncpy(reconfigName, "kD", numChars);}
    }
    else if (variable == 7)
    { // Reconfigure filter coefficient of previous filtered value after dividing by power of configPrecision
        a = (float) value/pow(10,configPrecision);
        if (verbose) {strncpy(reconfigName, "a", numChars);}
    }
    else if (variable == 8)
    { // Reconfigure filter coefficient of current raw value after dividing by power of configPrecision
        b0 = (float) value/pow(10,configPrecision);
        if (verbose) {strncpy(reconfigName, "b0", numChars);}
    }
    else if (variable == 9)
    { // Reconfigure filter coefficient of previous raw value after dividing by power of configPrecision
        b1 = (float) value/pow(10,configPrecision);
        if (verbose) {strncpy(reconfigName, "b1", numChars);}
    }
    else if (variable == 10)
    { // Reconfigure encoder resolution (pulses per motor shaft revolution) after dividing by power of configPrecision
        ppr = (float) value/pow(10,configPrecision);
        if (verbose) {strncpy(reconfigName, "ppr", numChars);}
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
// void parseInteger(int **ptrArr, int idx, char *strtokIndx) {
//     int tempInt = atoi(strtokIndx);
//     *ptrArr[idx] = tempInt;
// }

//--// Parse speed float value from received integer
void parseIntToFloat(double **ptrArr, int idx, char *strtokIndx) {
    long tempInt = atol(strtokIndx);
    double tempFloat = (double) tempInt;
    *ptrArr[idx] = tempFloat/pow(10,speedPrecision);
}

//--// Parse command
void parseCommand() {
    char *strtokIndx; // this is used by strtok() as an index
    char **intPtrIndx; // this is used by strtoul() as an index

    // Receive command type
    strtokIndx = strtok(tempChars,",");
    commandType = atoi(strtokIndx);

    if (commandType == 0)
    { // Reconfigure value
        // Format: <messageType, variable, value>
        strtokIndx = strtok(NULL, ","); 
        reconfigVariable = atoi(strtokIndx);
        strtokIndx = strtok(NULL, ","); 

        reconfigValue = strtoul(strtokIndx, intPtrIndx, 10);
        reconfigure(reconfigVariable, reconfigValue);
    }
    else if (commandType == 1)
    { // Store specified motor speed
        // Format: <leftSpeed, rightSpeed>
        // Receive DC motor value
        strtokIndx = strtok(NULL, ",");
        parseIntToFloat(ptrSpeedArr, 0, strtokIndx);

        strtokIndx = strtok(NULL, ",");
        parseIntToFloat(ptrSpeedArr, 1, strtokIndx);
    }
    else if (commandType == 2)
    { // Stop command
        speedLeft = 0;
        speedRight = 0;

        printMessageType('m');
        Serial.print("<");
        Serial.print(arduinoName);
        Serial.println(": Stopping motors>");
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
        Serial.print("reconfig: ");
        Serial.print(reconfigName);
        Serial.print(" = ");
        Serial.print(reconfigValue);
        Serial.println(">");
    }
    else if (commandType == 1)
    { // Control
        Serial.print("control: ");
        for (int i = 0; i < 2; i++) 
        {
        Serial.print(*ptrSpeedArr[i]);
        if (i < 1){Serial.print(", ");}
        }
        Serial.println(">");
    }
}

//// DC motors
//--// Turn on motors
// void turnOnDC() 
// {
//     digitalWrite(inL1, HIGH);
//     digitalWrite(inL2, LOW);
//     digitalWrite(inR1, HIGH);
//     digitalWrite(inR2, LOW);
// }

//--// Turn off motors
void turnOffDC() 
{
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, LOW);
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, LOW);
}

//--// Handle motor reading, PID control, and motor output
void handleDC() 
{
    // Read the position in an atomic block to avoid potential misreads
    int leftPos = 0;
    float leftVel_2 = 0;
    int rightPos = 0;
    float rightVel_2 = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        leftPos = leftPos_i;
        rightPos = rightPos_i;
    }

    // Compute velocity
    long currT = micros();
    float deltaLeftT = ((float) (currT - leftPrevT)) / 1.0e6;
    float deltaRightT = ((float) (currT - rightPrevT)) / 1.0e6;
    float leftVel = ((float) (leftPos - leftPosPrev)) / deltaLeftT;
    float rightVel = ((float) (rightPos - rightPosPrev)) / deltaRightT;
    leftPosPrev = leftPos;
    rightPosPrev = rightPos;
    leftPrevT = currT;
    rightPrevT = currT;

    // Convert count/s to RPM
    float leftV = leftVel/ppr*60.0;
    float rightV = rightVel/ppr*60.0;

    // Low-pass filter
    leftV1Filt = a*leftV1Filt + b0*leftV + b1*leftVPrev;
    leftVPrev = leftV;

    rightV1Filt = a*rightV1Filt + b0*rightV + b1*rightVPrev;
    rightVPrev = rightV;

    // Apply PID control
    float leftU = pidInput( // Compute control signal
        speedLeft, leftV1Filt, deltaLeftT,
        kP, kI, kD,
        leftEIntegral, leftEPrev
    );

    float rightU = pidInput( // Compute control signal
        speedRight, rightV1Filt, deltaRightT,
        kP, kI, kD,
        rightEIntegral, rightEPrev
    );

    int leftDir = 1;
    int rightDir = 1;
    if (leftU<0) {
      leftDir = -1;
    }
    if (rightU<0) {
      rightDir = -1;
    }
    int leftPwr = (int) fabs(leftU);
    if (leftPwr > 255) {
      leftPwr = 255;
    }
    int rightPwr = (int) fabs(rightU);
    if (rightPwr > 255) {
      rightPwr = 255;
    }

    // Set motor speed
    setMotor(leftDir, leftPwr, enL, inL1, inL2);
    setMotor(rightDir, rightPwr, enR, inR1, inR2);

    // Serial.print(leftV1Filt); // Print values for serial plot debugging
    // Serial.print(" ");
    // Serial.print(rightV1Filt);
    // Serial.println();
}

//--// Set motor speed and direction
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
    analogWrite(pwm, pwmVal);
    if (dir == 1){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else if (dir == -1){
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    else{
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
}

//--// Read left encoder
void encoderLeft(){
    int b = digitalRead(encoderLB);
    int increment = 0;
    if (b>0){
        increment = -1;
    }
    else{
        increment = 1;
    }
    leftPos_i = leftPos_i + increment;
}

//--// Read right encoder
void encoderRight(){
    int b = digitalRead(encoderRB);
    int increment = 0;
    if (b>0){
        increment = 1;
    }
    else{
        increment = -1;
    }
    rightPos_i = rightPos_i + increment;
}

//--// Compute PID input
float pidInput(
    float target, float velFilt, float deltaT,
    float kP, float kI, float kD,
    float &eIntegral, float &ePrev
){
    // Compute error
    float e = target - velFilt;
    eIntegral = eIntegral + e*deltaT;
    float eDerivative = (e - ePrev)/deltaT;
    ePrev = e;

    // Calculate input to motor
    float u = kP*e + kI*eIntegral + kD*eDerivative;
    return u;
}

// ----------------------------------------------------------------------------
// Arduino executables
// ----------------------------------------------------------------------------
void setup() {
    // Serial
    Serial.begin(baudRate);
    printMessageType('m');
    Serial.print("<");
    Serial.print(arduinoName);
    Serial.println(": Initializing Arduino...>");

    // DC motors
    setupDC();
    setupEncoders();
}

void loop() {
    // Serial
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
        parseCommand();
        if (verbose) {showParsedCommand();}
        newData = false;
        lastRecvTime = millis();
    }

    // If no new instruction has been received for some time, turn off motors
    if (millis() - lastRecvTime > 2000) {
        turnOffDC();
    }

    // DC motor handling
    if (commandType == 0 || commandType == 2)
    { // Stop motors
        turnOffDC();
    }
    else if (commandType == 1)
    { // Control motor speed
        handleDC();
        delay(1);
    }
}

