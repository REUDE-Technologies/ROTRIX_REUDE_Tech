#include <Keyboard.h>
#include <KeyboardLayout.h>
#include <Keyboard_da_DK.h>
#include <Keyboard_de_DE.h>
#include <Keyboard_es_ES.h>
#include <Keyboard_fr_FR.h>
#include <Keyboard_hu_HU.h>
#include <Keyboard_it_IT.h>
#include <Keyboard_pt_PT.h>
#include <Keyboard_sv_SE.h>

#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
/*#include <Adafruit_MLX90640.h>

*/

/*
// HX711 circuit wiring
*/
// Load Cell 1
const int LOADCELL1_DOUT_PIN = 12;
const int LOADCELL1_SCK_PIN = 13;
long calibration_factor = 96400.0;  // 10kg load cell
long OFFSET_1 = 0;  // Tare value for load cell 1

// Load Cell 2
const int LOADCELL2_DOUT_PIN = 16;
const int LOADCELL2_SCK_PIN = 17;
long calibration_factor_2 = 96400.0;  // 10kg load cell
long OFFSET_2 = 0;  // Tare value for load cell 2

unsigned int hx711_fail = 0;

//current & voltage sensor variables
float sensorValue_volt;
float arduino_voltage;
float voltage;
float batt_volt;

// Temperature sensor variables
float temperature;

// Dallas Temperature sensor setup
#define ONE_WIRE_BUS_1 13
#define ONE_WIRE_BUS_2 5
OneWire oneWire1(ONE_WIRE_BUS_1);
OneWire oneWire2(ONE_WIRE_BUS_2);
DallasTemperature sensors1(&oneWire1);
DallasTemperature sensors2(&oneWire2);
/*
float sensorValue_current;
float batt_current;

// Define the analog pin that the current sensor module is connected to
const int CURRENT_SENSOR_PIN = A1;
// Define the maximum current that the current sensor module can measure
const float MAX_CURRENT = 200; // Amps 
*/
// Define the analog pin that the voltage sensor module is connected to
const int VOLTAGE_SENSOR_PIN = A2;
// Define the maximum volts that the voltage sensor module can measure
const float MAX_VOLTAGE = 51.8; // volts
// Define the voltage divider value
const float divider_volt = 15.7; //



const int numValues = 4; // Number of expected integer values

// Customize here pulse lengths as needed
int ESC_min = 1000;    // Minimum pulse length in µs
int ESC_max = 2000;    // Maximum pulse length in µs

Servo mot;
unsigned int current_PWM = ESC_min;
unsigned int input_PWM = ESC_min;

// RPM sensor variables
byte PulsesPerRevolution = 2;     // number of blades
const unsigned long ZeroTimeout = 100000;     // in microseconds

volatile unsigned long LastTimeWeMeasured;
volatile unsigned long PeriodBetweenPulses = ZeroTimeout + 1000;
volatile unsigned long PeriodAverage = ZeroTimeout + 1000;
unsigned long FrequencyRaw;       // frequency of obstacle detected
unsigned long FrequencyReal;
unsigned long RPM;
unsigned int PulseCounter = 1;
unsigned long PeriodSum;

unsigned long LastTimeCycleMeasure = LastTimeWeMeasured;
unsigned long CurrentMicros = micros();
unsigned int AmountOfReadings = 1;
unsigned int ZeroDebouncingExtra;    // to avoid signal bouncing effects



// Define the analog pin that the current sensor module is connected to
const int RPM_SENSOR_PIN = 2;
const int width = 32;
const int height = 24;
const int arraySize = width * height * 3;

// Real sensor variables (no dummy arrays)
float batt_current = 0.0;  // No current sensor connected
float weight1 = 0.0;       // Load cell 1 value
float weight2 = 0.0;       // Load cell 2 value
float temperature1;
float temperature2;
float temperature3;
float temperature4;
float temperature5;
float temperature6;
float Ta;

// Simple HX711 read function
long readHX711(int dout_pin, int sck_pin) {
  long value = 0;
  
  // Wait for chip to be ready
  while (digitalRead(dout_pin) == HIGH);
  
  // Read 24 bits
  for (int i = 0; i < 24; i++) {
    digitalWrite(sck_pin, HIGH);
    value = value << 1;
    if (digitalRead(dout_pin)) value++;
    digitalWrite(sck_pin, LOW);
  }
  
  // Set gain to 128 (1 extra pulse)
  digitalWrite(sck_pin, HIGH);
  digitalWrite(sck_pin, LOW);
  
  // Convert to signed value
  if (value & 0x800000) {
    value |= (long) ~0xFFFFFF;
  }
  
  return value;
}

void setup() {
  Serial.begin(115200);
  delay(100);

// sensor pin assignment 
  pinMode(RPM_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_SENSOR_PIN), Pulse_Event, FALLING);
  /*pinMode(CURRENT_SENSOR_PIN, INPUT);
  pinMode(VOLTAGE_SENSOR_PIN, INPUT);*/
  mot.attach(4, ESC_min, ESC_max);
  mot.writeMicroseconds(ESC_min);
  
  // Initialize Dallas Temperature sensors
  sensors1.begin();
  sensors2.begin();

  // Initialize HX711 Load Cells
  pinMode(LOADCELL1_DOUT_PIN, INPUT);
  pinMode(LOADCELL1_SCK_PIN, OUTPUT);
  pinMode(LOADCELL2_DOUT_PIN, INPUT);
  pinMode(LOADCELL2_SCK_PIN, OUTPUT);
  delay(100);
  
  // Get tare values
  long sum1 = 0, sum2 = 0;
  for (int i = 0; i < 10; i++) {
    sum1 += readHX711(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
    sum2 += readHX711(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
    delay(10);
  }
  OFFSET_1 = sum1 / 10;
  OFFSET_2 = sum2 / 10;

// parameter initialisation
  RPM = 0;

  Serial.println("Setup Done");
  delay(100);
}

void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    char inputChars[inputString.length() + 1];
    inputString.toCharArray(inputChars, inputString.length() + 1);
    // Parse the input using strtok()
    char *token = strtok(inputChars, ",");
    int index = 0;
    while (token != NULL && index < numValues) {
      int parsedValue = atof(token); // Convert token to an integer
      switch (index) {
        case 0: PulsesPerRevolution = parsedValue; break;
        case 1: ESC_min = parsedValue; break;
        case 2: ESC_max = parsedValue; break;
        case 3: input_PWM = parsedValue; break;
      }
      index++;
      token = strtok(NULL, ","); // Get the next token
    }
  }
    // mot.attach(4, ESC_min, ESC_max);
    // mot.writeMicroseconds(ESC_min);
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
  

  // Read the voltage from the voltage & current sensor module
  sensorValue_volt = analogRead(VOLTAGE_SENSOR_PIN);
  // sensorValue_current = analogRead(CURRENT_SENSOR_PIN); // commented: current sensor not in use
  // Convert the voltage sensor input voltage to battery voltage value (Mega default 5V ref)
  voltage = sensorValue_volt * (5.0 / 1024.0);
  //voltage = arduino_voltage * (3.3 / 5.0);
  batt_volt = voltage * divider_volt;

  // Read Dallas Temperature sensors
  sensors1.requestTemperatures();
  sensors2.requestTemperatures();
  temperature2 = sensors1.getTempCByIndex(0);  // First temperature sensor on pin 5
  temperature3 = sensors2.getTempCByIndex(0);  // Second temperature sensor on pin 6
  
  // Set other temperature variables to 0
  temperature1 = 0.0;
  temperature4 = 0.0;
  temperature5 = 0.0;
  temperature6 = 0.0;
  Ta = 0.0;

  // Read weight from both Load Cells (simple calculation)
  long raw1 = readHX711(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
  long raw2 = readHX711(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
  weight1 = (float)(raw1 - OFFSET_1) / calibration_factor;
  weight2 = (float)(raw2 - OFFSET_2) / calibration_factor_2;
  
  // DEBUG: Uncomment to see raw values and help with calibration
  // Serial.print("RAW1: "); Serial.print(raw1);
  // Serial.print(" | OFFSET1: "); Serial.print(OFFSET_1);
  // Serial.print(" | DIFF1: "); Serial.println(raw1 - OFFSET_1);

/*// Convert the current sensor input voltage to battery current value
  batt_current = (sensorValue_current / 1024.0) * 200;  

  //get frame data from thermal camera
  if (mlx.getFrame(frame) != 0) {
    Serial.println("Thermal camera error 2");
    return;
  }
  //Measure ambient temperature
  Ta = mlx.getTa(false); // false = no new frame capture

   // load cell data collection
  
  if(scale1.is_ready() && scale2.is_ready()) {
    float weight1 = scale1.get_units();
    float weight2 = scale2.get_units();    
  } 
  else {
    Serial.println("HX711 not ready. Check connections");
  }*/
  
  /*if(scale2.is_ready()) {
    float weight2 = scale2.get_units();
  } 
  else {
    Serial.println("Loadcell error");
  }
*/

  // calculate motor RPM
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
  
  Serial.print("UAT002");
  Serial.print(",");
  Serial.print(current_PWM);
  Serial.print(",");
  Serial.print(RPM);
  Serial.print(",");
  Serial.print(batt_current);
  Serial.print(",");
  Serial.print(batt_volt, 2);
  Serial.print(",");
  Serial.print(weight2);
  Serial.print(",");
  Serial.print(weight1);
  Serial.print(",");
  Serial.print(temperature1, 1);
  Serial.print(",");
  Serial.print(temperature2, 1);
  Serial.print(",");
  Serial.print(temperature3, 1);
  Serial.print(",");
  Serial.print(temperature4, 1);
  Serial.print(",");
  Serial.print(temperature5, 1);
  Serial.print(",");
  Serial.print(temperature6, 1);
  Serial.print(",");
  Serial.print(Ta, 1);
  for (uint8_t h=0; h<24; h++) {
      for (uint8_t w=0; w<32; w++) {
         int t = 0;
         //Serial.print(t, 2);
         //Serial.print(",");
      }
  }
  //Serial.print(",");
  for (int i = 0; i < arraySize; i++) {  
        //Serial.print("0x00");
        if ( i< arraySize - 1) {
        //Serial.print(","); // Add delimiter
        }
  }
  Serial.println();
  delay(500);
}

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