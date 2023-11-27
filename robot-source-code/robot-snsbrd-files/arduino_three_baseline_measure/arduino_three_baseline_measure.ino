// Arduino control loop
// Hans Verdolaga
// MSc Mechatronics 2023
// MC-F23 Thesis

// --------------------------------------------
// Libraries
// --------------------------------------------
// Standard libraries
#include <Arduino.h>

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
unsigned long readTimeSec = 1; // Interval for reading data in seconds
unsigned long sendTimeMin = 0.1; // Interval for sending data in minutes
int long sendTime; // Variable time for sending data
int long sendTimePrev; // Previous time value
unsigned int sensorCount = 3; // Number of sensors

u32 baseline0 = 0;//2556858855; // Baseline for sensor 0
u32 baseline1 = 0;//2556858855; // Baseline for sensor 1
u32 baseline2 = 0;//2556858855; // Baseline for sensor 2

// Arduino board identifier - Sensor (and servo) board
char arduinoName[7] = "SNSBRD"; // 3 characters + null terminator

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
        Serial.println(" SGP failed");
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
    TCA.closeChannel(TCA_CHANNEL_0);

    // Sensor 1
    TCA.openChannel(TCA_CHANNEL_1);
    setupIndSensor(1);
    TCA.closeChannel(TCA_CHANNEL_1);

    // Sensor 2
    TCA.openChannel(TCA_CHANNEL_2);
    setupIndSensor(2);
    TCA.closeChannel(TCA_CHANNEL_2);
}

// --------------------------------------------
// Main functions
// --------------------------------------------
//// Sensors
//--// Read individual sensor
void readIndSensor(u32 *dataArray, int sensorIdx)
{
    s16 err = 0;
    u16 tvoc_ppb, co2_eq_ppm;
    u32 baseline;
    //u16 scaled_ethanol_signal, scaled_h2_signal;

    // Read sensor
    // err = sgp_measure_signals_blocking_read(
    //     &scaled_ethanol_signal, &scaled_h2_signal);
    err = sgp_measure_iaq_blocking_read(&tvoc_ppb, &co2_eq_ppm);
    sgp_get_iaq_baseline(&baseline);

    if (err == STATUS_OK) {
        // dataArray[elementIdx + sensorIdx] = scaled_ethanol_signal;
        // dataArray[sensorIdx] = tvoc_ppb;
        // dataArray[sensorIdx + 1] = co2_eq_ppm;
        dataArray[sensorIdx] = baseline;
    } else {
        // dataArray[3*sensorIdx] = 0;
        // dataArray[3*sensorIdx + 1] = 0;
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
void sendData(u32 *dataArray)
{
    // printMessageType('d');
    // Serial.print("<,");
    for (int i = 0; i < sensorCount; i++) {
        Serial.print(dataArray[i]);
        if (i < (sensorCount - 1)) {Serial.print(",");}
    }
    // Serial.println(">");
}

//--// Read all sensors
void readSensors()
{
    sendTime = millis();
    u32 dataArray[sensorCount];
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

    // If 5 minutes have passed, send data
    if (sendTime - sendTimePrev >= sendTimeMin * 60 * 1000) {
        sendTimePrev = sendTime;
        sendData(dataArray);
    }
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

    // Set previous time
    sendTimePrev = millis();
}

void loop() {
    // Read sensors
    readSensors();

    // Wait for next reading
    delay(readTimeSec * 1000);
}
