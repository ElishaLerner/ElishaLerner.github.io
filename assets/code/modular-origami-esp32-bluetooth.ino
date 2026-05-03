#include <RotaryEncoder.h>
#include <PID_v1.h>
#include <math.h>
#include <BluetoothSerial.h>
#include "driver/ledc.h"
#include <Arduino.h>

BluetoothSerial SerialBT;

// Thermistor constants
#define VCC 3.3f
#define R_FIXED 10000.0f
#define BETA 3950.0f
#define T0 298.15f
#define R0 10000.0f

// Thermistor variables
double tempC = 0;
const int th1 = 15;     //mosfet pin
const int adcPin = 26;  //read pin

// Thermistor 2 variables
double tempC2 = 0;
const int th2 = 2;       // Second MOSFET pin (PWM)
const int adcPin2 = 34;  // Second thermistor analog input




// Motor pins (ESP32 remapped)
const int M2En = 23, M2Ph = 22;
const int M1En = 21, M1Ph = 19;
const int M3En = 18, M3Ph = 5;

// Motor direction signals
double Ph2Pos = 255, Ph2Neg = 0;
double Ph1Pos = 255, Ph1Neg = 0;
double Ph3Pos = 0, Ph3Neg = 255;

// Movement parameters
double zerospeed = 30;
double Error = 50;
double Espeed = 30;
double pollc = 0;

// Encoder resolution factor
const int mod = 380 * 12 / 15.9;

// Encoder pins (ESP32 remapped)
#define ENCODER2_PIN_A 14
#define ENCODER2_PIN_B 27
#define ENCODER1_PIN_A 25
#define ENCODER1_PIN_B 33
#define ENCODER3_PIN_A 32
#define ENCODER3_PIN_B 35



double En1read = 0;
double En2read = 0;
double En3read = 0;

// Serial input parsing
const int maxLength = 100;
char inputString[maxLength];
int intArray[10];
int arraySize = 0;

// PID control
double PID1;
double Kp = 2, Ki = 0.5, Kd = 3;
double set1 = 0;
PID temp1_1(&tempC, &PID1, &set1, Kp, Ki, Kd, DIRECT);

// PID control for second thermistor
double PID2;
double set2 = 0;
PID temp2_2(&tempC2, &PID2, &set2, Kp, Ki, Kd, DIRECT);

// Gait control
enum GaitType { NONE,
                SHIMMY,
                CRUTCH,
                CRAWL };
GaitType currentGait = NONE;

bool demoMode = false;
bool demoStarted = false;
unsigned long demoStartTime = 0;
int demoStep = 0;

const int stepsShimmy[][3] = {
  { 10, 0, 10 },
  { 10, 0, 12 },
  { 11, 0, 10 },
};

const int stepsCrutch[][3] = {
  { 4, 0, 4 },
  { 8, 0, 8 },
  { 8, 12, 8 },
  { 4, 12, 4 },
};

const int stepsCrawl[][3] = {
  { 5, 0, 5 },
  { 5, 10, 5 },
  { 0, 0, 0 },
};

const int* currentSteps = nullptr;
int totalSteps = 0;

// Encoders
RotaryEncoder encoder1(ENCODER1_PIN_A, ENCODER1_PIN_B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder encoder2(ENCODER2_PIN_A, ENCODER2_PIN_B, RotaryEncoder::LatchMode::TWO03);
RotaryEncoder encoder3(ENCODER3_PIN_B, ENCODER3_PIN_A, RotaryEncoder::LatchMode::TWO03);  // Reversed intentionally

IRAM_ATTR void checkPosition() {
  encoder1.tick();  // just call tick() to check the state.
  encoder2.tick();  // just call tick() to check the state.
  encoder3.tick();  // just call tick() to check the state.
}


void setup() {
  SerialBT.begin("Oribot2");
  analogReadResolution(12);

  // Setup PWM channels
  ledcSetup(0, 5000, 8);
  ledcAttachPin(M1En, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(M1Ph, 1);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(M2En, 2);
  ledcSetup(3, 5000, 8);
  ledcAttachPin(M2Ph, 3);
  ledcSetup(4, 5000, 8);
  ledcAttachPin(M3En, 4);
  ledcSetup(5, 5000, 8);
  ledcAttachPin(M3Ph, 5);
  ledcSetup(6, 5000, 8);
  ledcAttachPin(th1, 6);
  ledcSetup(7, 5000, 8);
  ledcAttachPin(th2, 7);

  temp2_2.SetMode(AUTOMATIC);




  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_B), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_B), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_PIN_A), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_PIN_B), checkPosition, CHANGE);

  temp1_1.SetMode(AUTOMATIC);
  delay(100);
}

void loop() {



  if (encoder1.getPosition() != En1read) {
    SerialBT.print("Motor 1: ");
    SerialBT.println(encoder1.getPosition());
  }
  if (encoder2.getPosition() != En2read) {
    SerialBT.print("Motor 2: ");
    SerialBT.println(encoder2.getPosition());
  }
  if (encoder3.getPosition() != En3read) {
    SerialBT.print("Motor 3: ");
    SerialBT.println(encoder3.getPosition());
  }

  if (SerialBT.available() > 0) {
    int length = SerialBT.readBytesUntil('\n', inputString, maxLength);
    inputString[length] = '\0';

    if (strcmp(inputString, "SHIMMY") == 0) {
      currentGait = SHIMMY;
      currentSteps = &stepsShimmy[0][0];
      totalSteps = sizeof(stepsShimmy) / sizeof(stepsShimmy[0]);
      demoMode = true;
      demoStarted = false;
      SerialBT.println("Gait: SHIMMY mode started");
    } else if (strcmp(inputString, "CRUTCH") == 0) {
      currentGait = CRUTCH;
      currentSteps = &stepsCrutch[0][0];
      totalSteps = sizeof(stepsCrutch) / sizeof(stepsCrutch[0]);
      demoMode = true;
      demoStarted = false;
      SerialBT.println("Gait: CRUTCH mode started");
    } else if (strcmp(inputString, "CRAWL") == 0) {
      currentGait = CRAWL;
      currentSteps = &stepsCrawl[0][0];
      totalSteps = sizeof(stepsCrawl) / sizeof(stepsCrawl[0]);
      demoMode = true;
      demoStarted = false;
      SerialBT.println("Gait: CRAWL mode started");
    } else if (strcmp(inputString, "STOP") == 0) {
      demoMode = false;
      currentGait = NONE;
      SerialBT.println("Demo mode stopped");
    } else {
      convertStringToIntArray(inputString, intArray, arraySize);
      SerialBT.println("Converted array:");
      for (int i = 0; i < arraySize; i++) {
        if (i < 3) {
          intArray[i] = constrain(intArray[i], -20, 20);
        }
        SerialBT.print(intArray[i]);
        SerialBT.print(" ");
      }
      SerialBT.println();
    }
  }

  // DEMO MODE automation
  if (demoMode && currentSteps != nullptr) {
    const int stepDelay = 2000;
    if (!demoStarted) {
      demoStartTime = millis();
      demoStep = 0;
      demoStarted = true;
    }
    if (millis() - demoStartTime > stepDelay) {
      for (int i = 0; i < 3; i++) {
        intArray[i] = constrain(*(currentSteps + demoStep * 3 + i), 0, 24);
      }
      demoStep++;
      if (demoStep >= totalSteps) demoStep = 0;
      demoStartTime = millis();
    }
  }

  // Motor control

  moveMotor(1, encoder1.getPosition(), intArray[0], M1En, M1Ph, Ph1Pos, Ph1Neg, 0, 1);
  moveMotor(2, encoder2.getPosition(), intArray[1], M2En, M2Ph, Ph2Pos, Ph2Neg, 2, 3);
  moveMotor(3, encoder3.getPosition(), intArray[2], M3En, M3Ph, Ph3Pos, Ph3Neg, 4, 5);

  En1read = encoder1.getPosition();
  En2read = encoder2.getPosition();
  En3read = encoder3.getPosition();

  // TEMP CONTROL
  int adcValue = analogRead(adcPin);
  float Vout = (adcValue / 4095.0f) * VCC;
  float R_thermistor = R_FIXED * (VCC / Vout - 1.0f);
  float tempK = 1.0f / (1.0f / T0 + log(R_thermistor / R0) / BETA);
  tempC = tempK - 273.15f;

  set1 = intArray[3];
  temp1_1.Compute();
  PID1 = constrain(PID1, 0, 40);
  if (set1 != 0) {
    ledcWrite(6, PID1);
  } else {
    ledcWrite(6, 0);
  }


  // TEMP CONTROL for second thermistor
  int adcValue2 = analogRead(adcPin2);
  float Vout2 = (adcValue2 / 4095.0f) * VCC;
  float R_thermistor2 = R_FIXED * (VCC / Vout2 - 1.0f);
  float tempK2 = 1.0f / (1.0f / T0 + log(R_thermistor2 / R0) / BETA);
  tempC2 = tempK2 - 273.15f;

  set2 = intArray[4];
  temp2_2.Compute();
  PID2 = constrain(PID2, 0, 40);
  if (set2 != 0) {
    ledcWrite(7, PID2);
  } else {
    ledcWrite(7, 0);
  }

  if (++pollc >= 5000) {
    SerialBT.print("Set temp1 = ");
    SerialBT.print(set1);
    SerialBT.print(" | Temp1: ");
    SerialBT.print(tempC, 2);
    SerialBT.print(" °C || Set temp2 = ");
    SerialBT.print(set2);
    SerialBT.print(" | Temp2: ");
    SerialBT.print(tempC2, 2);
    SerialBT.println(" °C");
    pollc = 0;
  }
}

void moveMotor(int motor, long pos, int target_mm, int enPin, int phPin, int phPos, int phNeg, int enChannel, int phChannel) {
  long targetPos = target_mm * mod;
  long error = targetPos - pos;

  if (error > Error) {
    ledcWrite(enChannel, 255);
    ledcWrite(phChannel, phPos);
  } else if (error > 15) {
    ledcWrite(enChannel, Espeed);
    ledcWrite(phChannel, phPos);
  } else if (error < -Error) {
    ledcWrite(enChannel, 255);
    ledcWrite(phChannel, phNeg);
  } else if (error < -15) {
    ledcWrite(enChannel, Espeed);
    ledcWrite(phChannel, phNeg);
  } else {
    ledcWrite(enChannel, 0);
  }
}

void convertStringToIntArray(const char* str, int* arr, int& size) {
  int i = 0, num = 0, idx = 0;
  bool isNegative = false;
  bool inNumber = false;

  while (str[i] != '\0') {
    if (str[i] == ' ') {
      if (inNumber) {
        if (isNegative) num = -num;
        arr[idx++] = num;
        num = 0;
        isNegative = false;
        inNumber = false;
      }
    } else if (str[i] == '-') {
      isNegative = true;
      inNumber = true;
    } else if (str[i] >= '0' && str[i] <= '9') {
      num = num * 10 + (str[i] - '0');
      inNumber = true;
    }
    i++;
  }

  if (inNumber) {
    if (isNegative) num = -num;
    arr[idx++] = num;
  }
  size = idx;
}