// ROTRIX Application Firmware: Multi-Sensor Integration Source Code
//
// Copyright (c) 2025 REUDE Technologies Pvt. Ltd.
// MIT License 
//
// This source code provides a starting point for integrating multiple sensors and actuators with the ROTRIX platform. 
// Users can extend or modify the code to suit their specific sensor and hardware requirements. 
//
// Disclaimer: This code is provided as a sample for educational and prototyping purposes only. 
// Users are responsible for adapting the code to their specific needs and testing it thoroughly before deployment. 
// REUDE Technologies Pvt. Ltd. shall not be held liable for any mishappenings, damages, or malfunctions arising from the use of this code.
// 
// GitHub: https://github.com/REUDE-Technologies/REUDE_ROTRIX.git
//
// Prerequisites: 
// Before using this sample code, ensure the following additional libraries are installed in your Arduino IDE,
// Servo - For controlling motors via ESC.
// HX711 - For processing data from load cell amplifiers.
// Adafruit_MLX90640 - For interfacing with the MLX90640 thermal camera.
// BreezyArduCAM - For handling RGB camera data.
// Wire - For I2C communication.
// SPI - For SPI communication with the RGB camera.
//
// This code supports the following hardware components:
// Load cells with HX711 amplifiers, RPM Sensor, voltage & current sensors
// and is specific to the following sensors:
// Arducam 2 MP Mini module with OV2640 sensor & Adafruit MLX90640 IR thermal camera
//
// Customizing for Your Needs:
// Update the pin assignments and variable values to match your hardware configuration.
// Adjust calibration factors (e.g., for load cells) as per your sensors.
// Extend the code by adding support for new sensors or additional processing.

#include <Servo.h>
#include <Adafruit_MLX90640.h>
#include <HX711.h>
#include <Wire.h>
#include <BreezyArduCAM.h>
#include <SPI.h>

/* Device ID */
String device_id = "REU_MTB_001"                    

/* No. of values received from software */
const int numValues = 4; 

/* Initializing ESC variables with standard PWM values */
unsigned int ESC_min;                                 // Minimum ESC PWM value (µs)
unsigned int ESC_max;                                 // Maximum ESC PWM value (µs)

/* Motor setup */
Servo mot;
const int MOTOR_SIGNAL_PIN;                           // Motor - ESC signal pin  
unsigned int current_PWM = ESC_min;
unsigned int input_PWM = ESC_min;

/* HX711 amplifier - load cell variables setup based on HX711 library */
HX711 scale1;
HX711 scale2;
const int LOADCELL1_DOUT_PIN;                         // Thrust loadcell - amplifier pins
const int LOADCELL1_SCK_PIN;
const int LOADCELL2_DOUT_PIN;                         // Torque loadcell - amplifier pins
const int LOADCELL2_SCK_PIN;
long calibration_factor_1;                            // Thrust loadcell calibration value
long calibration_factor_2;                            // Torque loadcell calibration value

/* RPM sensor variables */
byte PulsesPerRevolution;                             // number of propeller blades
const unsigned long ZeroTimeout = 100000;          
volatile unsigned long LastTimeWeMeasured;
volatile unsigned long PeriodBetweenPulses = ZeroTimeout + 1000;
volatile unsigned long PeriodAverage = ZeroTimeout + 1000;
unsigned int PulseCounter = 1;
unsigned long PeriodSum;
unsigned int AmountOfReadings = 1;
unsigned int ZeroDebouncingExtra;
unsigned long FrequencyRaw;                        
unsigned long FrequencyReal;
unsigned long RPM;
unsigned long LastTimeCycleMeasure = LastTimeWeMeasured;
unsigned long CurrentMicros = micros();
const int RPM_SENSOR_PIN;                            // RPM sensor signal pin

/* Voltage & current sensor variables */
float sensorValue_volt;
float batt_volt;
float sensorValue_current;
float batt_current;

const int CURRENT_SENSOR_PIN;                        // current sensor analog signal pin
const float MAX_CURRENT;                             // current sensor maximum value (A) (range: 0-200) 
const float divider_current;                         // current sensor divider value 
const int VOLTAGE_SENSOR_PIN;                        // voltage sensor analog signal pin
const float MAX_VOLTAGE;                             // voltage sensor maximum value (V) (range: 0-58)
const float divider_volt;                            // voltage sensor divider value

/* Thermal camera variables setup based on MLX90640 library */
Adafruit_MLX90640 mlx;
float frame[32*24];                                  // buffer for full frame of temperatures
float Ta;                                            // Ambient temperature value           
float t;                                      

/* RGB camera variables*/
static const int CS;                                 // RGB camera Chip Select (CS) pin                      
Serial_ArduCAM_FrameGrabber fg;
ArduCAM_Mini_2MP myCam(CS, &fg);
const int width;                                     // RGB camera set resolution
const int height;                                    // RGB camera set resolution
const int markers = 4;                               // no. of markers in RGB data
const int arraySize = (width * height * 3) + markers; 
unsigned int array[arraySize];

const byte START_MARKER[] = {0xAA, 0xBB};            // Markers to mark start of an array
const byte END_MARKER[] = {0xDD, 0xEE};              // Markers to mark end of an array
int check = 1;

void setup() {
    Serial.begin(115200);
    delay(100);

    /* sensor pins assignment and setup */
    attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_PIN), Pulse_Event, RISING); 
    RPM = 0; 

    pinMode(CURRENT_SENSOR_PIN, INPUT);
    pinMode(VOLTAGE_SENSOR_PIN, INPUT);
    batt_current = 0;
    batt_volt = 0;

    mot.attach(MOTOR_SIGNAL_PIN, ESC_min, ESC_max);
    mot.writeMicroseconds(ESC_min);

    scale1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
    scale1.set_scale(calibration_factor_1);
    scale1.tare();                                       //Assuming there is no weight on the scale at start up
    scale2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
    scale2.set_scale(calibration_factor_2);
    scale2.tare();

    if (! mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Thermal camera error 1");
    while (1) delay(10);
    }
    mlx.setMode(MLX90640_CHESS);
    mlx.setResolution(MLX90640_ADC_18BIT);
    mlx.setRefreshRate(MLX90640_2_HZ);

    Wire.begin();
    SPI.begin();
    myCam.beginJpeg320x240();                            // Start the camera image size  

    delay(100);
}

void loop() {
    if (Serial.available() > 0) {
        String inputString = Serial.readStringUntil('\n');
        char inputChars[inputString.length() + 1];
        inputString.toCharArray(inputChars, inputString.length() + 1);
        char *token = strtok(inputChars, ",");
        int index = 0;
        while (token != NULL && index < numValues) {
            int parsedValue = atof(token); 
            switch (index) {
            case 0: PulsesPerRevolution = parsedValue; break;
            case 1: ESC_min = parsedValue; break;
            case 2: ESC_max = parsedValue; break;
            case 3: input_PWM = parsedValue; break;
            }
        index++;
        token = strtok(NULL, ","); 
        }
    }
    if (check == 1) {
    mot.attach(4, ESC_min, ESC_max);
    mot.writeMicroseconds(ESC_min);
    count ++;
    }

    if ((input_PWM > (ESC_min-1)) && (input_PWM < (ESC_max+1)))
    {
        if(input_PWM < current_PWM) {
            for(int i = current_PWM; i >= input_PWM; i -= 1)
            {
                mot.writeMicroseconds(i);
                delay(20);
            }
        }
        else if(input_PWM >= current_PWM){
            for(int i = current_PWM; i <= input_PWM; i += 1)
            {
                mot.writeMicroseconds(i);
                delay(20);
            }
        }
    current_PWM = input_PWM;
    } 
  
    /* Voltage & current sensor module calculations */
    sensorValue_volt = analogRead(VOLTAGE_SENSOR_PIN);
    sensorValue_current = analogRead(CURRENT_SENSOR_PIN);
    batt_voltage = sensorValue_volt * (3.3 / 1024.0);
    batt_voltage = batt_voltage * divider_volt;
    batt_current = sensorValue_current * (3.3 / 1024.0);  
    batt_current = batt_current * divider_current;

    /* Frame capture from thermal camera */
    if (mlx.getFrame(frame) != 0) {
        Serial.println("Thermal camera error 2");
        return;
    }
    /* Ambient temperature measurement from thermal camera */
    Ta = mlx.getTa(false); 

    /* Motor RPM calculation */
    LastTimeCycleMeasure = LastTimeWeMeasured;
    CurrentMicros = micros();
    if (CurrentMicros < LastTimeCycleMeasure) {
        LastTimeCycleMeasure = CurrentMicros;
    }
    FrequencyRaw = 10000000000 / PeriodAverage;  // in microseconds
    if (PeriodBetweenPulses > ZeroTimeout - ZeroDebouncingExtra || CurrentMicros - LastTimeCycleMeasure > ZeroTimeout - ZeroDebouncingExtra) {
        FrequencyRaw = 0;  // Set frequency as 0.
        ZeroDebouncingExtra = 2000;
    } else {
        ZeroDebouncingExtra = 0;
    }
    FrequencyReal = FrequencyRaw/10000;
    RPM = (FrequencyRaw / PulsesPerRevolution) * 60;
    RPM = RPM / 10000;

    /* Load cell data capture */
    if(scale1.is_ready() && scale2.is_ready()) {
        float weight1 = scale1.get_units();
        float weight2 = scale2.get_units();    
    } 
    else {
        Serial.println("HX711 not ready. Check connections");
    }

    /* RGB camera data capture */
    array = myCam.capture();
    
    /* Sensor value outputs to software */
    Serial.print(device_id);
    Serial.print(",");
    Serial.print(current_PWM);
    Serial.print(",");
    Serial.print(RPM);
    Serial.print(",");
    Serial.print(batt_current);
    Serial.print(",");
    Serial.print(batt_volt);
    Serial.print(",");
    Serial.print(weight1);
    Serial.print(",");
    Serial.print(weight2);
    Serial.print(",");
    Serial.print(Ta); 
    Serial.print(",");
    Serial.print("0x");
    Serial.print(START_MARKER[0], HEX);
    Serial.print(",");
    Serial.print("0x");
    Serial.print(START_MARKER[1], HEX);
    Serial.print(",");
    for (uint8_t h=0; h<24; h++) {
        for (uint8_t w=0; w<32; w++) {
            t = frame[h*32 + w];
            Serial.print(t, 2);
            Serial.print(",");
        }
    }
    Serial.print(",");
    Serial.print("0x");
    Serial.print(END_MARKER[0], HEX);
    Serial.print(",");
    Serial.print("0x");
    Serial.print(END_MARKER[1], HEX);
    Serial.print(",");
    Serial.print("0x");
    Serial.print(START_MARKER[0], HEX);
    Serial.print(",");
    Serial.print("0x");
    Serial.print(START_MARKER[1], HEX);
    Serial.print(",");
    for (int i = 0; i < imgsize; i++) {    
        if((i>1) && (i<=(imgsize-2)))
        {
            Serial.print("0x");
            if (array[i] < 0x10) {
                Serial.print("0");
            }
            Serial.print(array[i], HEX);
            if (i < arraySize - 1) {
                Serial.print(", ");
            }
        }
    }
    Serial.print(",");
    Serial.print("0x");
    Serial.print(END_MARKER[0], HEX);
    Serial.print(",");
    Serial.print("0x");
    Serial.print(END_MARKER[1], HEX);
    Serial.println();
    delay(500);
}

/* Interrupt event to calculate motor RPM */
void Pulse_Event() {
    PeriodBetweenPulses = micros() - LastTimeWeMeasured;
    LastTimeWeMeasured = micros();
    if (PulseCounter >= AmountOfReadings)  {
        PeriodAverage = PeriodSum / AmountOfReadings;
        PulseCounter = 1;
        PeriodSum = PeriodBetweenPulses;
        int RemapedAmountOfReadings = map(PeriodBetweenPulses, 40000, 5000, 1, 10);
        RemapedAmountOfReadings = constrain(RemapedAmountOfReadings, 1, 10);
        AmountOfReadings = RemapedAmountOfReadings;
    } else {
        PulseCounter++;
        PeriodSum = PeriodSum + PeriodBetweenPulses;
    }
}

