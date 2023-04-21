#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Main Screen/Profile Select
// <Prof 0>
// MAX 255C
// DUR 255s
// NTC  25C

//Profile Menu
//  Prof 0
// [Run   ]
//  Edit
//  Cancel

//Profile Running
//  Prof 0
// 125/150C
//  50/240s
// [Cancel]

//Profile Editing
//  Pnt  0
// <  255C>
//    255s
//  Save

// Text area 8x4chars, Graph area 80x32px
// Physical buttons: <, Ok, >

//#define serialDebug
//#define serialVerbose

#define buttonPinL 27
#define buttonPinM 26
#define buttonPinR 25

#define touchButtonL T7
#define touchButtonM T8
#define touchButtonR T9

int touchThresholdM;
int touchThresholdL;
int touchThresholdR;

#define heaterPowerPin 17
#define heaterPeriodms 500

#define NTCADCPin 4
#define NTCPowerPin 16
#define NTCReference1Pin 2
#define NTCReference2Pin 13

#define supplyMillivolts 3335 // Make sure this is accurate to esp regulator voltage
#define ADCResolution 4096
#define samples 5

#define referenceResistance1 100
#define referenceResistance2 1000
#define thresholdTemperature 100

#define thermistorReferenceTemperature 25
#define thermistorReferenceResistance 10000
#define beta 3425

#define OLEDSDA 12
#define OLEDSCL 14

enum systemState{
  SELECT=0,
  MENU=1,
  RUN=2,
  EDIT=3,
  EDITPNTTEMP=4,
  EDITPNTTIME=5,
  EDITSELPNT=6
};

systemState state = SELECT;
int buttonState;
unsigned long buttonTime;
int selectedProfile = 0; //0-9
int selectedPoint = 0; //0-9
int selectedRow = 0; //0-3
bool selected = true;

enum dataType{
  PROFILE_TEMP=0,
  PROFILE_TIME=1,
  POINT_TEMP=2,
  POINT_TIME=3
};

char data[200]; //10 profiles * 10 points * 2 chars (temperature & duration)

bool heaterState;

float lastTemperature;

double lastError;
unsigned long previousTime;

#define kP 25
#define kI 25
#define kD 75

double iTerm;

unsigned long heaterPreviousTime, heaterSwitchTime;

unsigned long startTime;

void heaterPower(bool power = false) {
  if (power && !heaterState) {
    pinMode(heaterPowerPin, OUTPUT);
    digitalWrite(heaterPowerPin, LOW);
  } else if (!power && heaterState) {
    pinMode(heaterPowerPin, INPUT);
  }

  heaterState = power;
}

float readTemperature() {
  bool temperatureRange = lastTemperature > thresholdTemperature;

  if (temperatureRange) {
    pinMode(NTCReference1Pin, OUTPUT);
    digitalWrite(NTCReference1Pin, LOW);
  } else {
    pinMode(NTCReference2Pin, OUTPUT);
    digitalWrite(NTCReference2Pin, LOW);
  }
  pinMode(NTCPowerPin, OUTPUT);
  digitalWrite(NTCPowerPin,HIGH);

  delay(5);

  //float rawADCValue = 0;

  //for (int i = 0; i < samples; i++) {
  //  rawADCValue += analogRead(NTCADCPin);
  //  delay(10);
  //}

  float ADCmVValue = 0;

  for (int i = 0; i < samples; i++) {
    ADCmVValue += analogReadMilliVolts(NTCADCPin);
    delay(2);
  }

  digitalWrite(NTCPowerPin,LOW);
  pinMode(NTCPowerPin, INPUT);
  if (temperatureRange) {
    pinMode(NTCReference1Pin, INPUT);
  } else {
    pinMode(NTCReference2Pin, INPUT);
  }

  //rawADCValue /= samples;
  //float ADCValue2 = map(rawADCValue, 105, 1980, 248, 2296); // Try to account for esp32 nonlinear ADC
  //float referenceResistanceB = temperatureRange ? referenceResistance1 : referenceResistance2;
  //float NTCResistance2 = ADCResolution * referenceResistanceB / ADCValue2 - referenceResistanceB;
  //float temperature2 = 1.0 / (1.0 / (thermistorReferenceTemperature + 273.15) + log(NTCResistance2 / thermistorReferenceResistance) / beta) - 273.15;

  ADCmVValue /= samples;
  float ADCValue = ADCmVValue * ADCResolution / supplyMillivolts; // millivolt reading is probably calibrated, remap back to counts
  float referenceResistance = temperatureRange ? referenceResistance1 : referenceResistance2;
  float NTCResistance = ADCResolution * referenceResistance / ADCValue - referenceResistance;
  float temperature = 1.0 / (1.0 / (thermistorReferenceTemperature + 273.15) + log(NTCResistance / thermistorReferenceResistance) / beta) - 273.15;
  temperature -= temperatureRange ? 10.0 : 0.0;
  lastTemperature = temperature;

#ifdef serialVerbose
  //Serial.print("rawADC: ");
  //Serial.println(rawADCValue);
  //Serial.print("referenceResistance: ");
  //Serial.println(referenceResistanceB);
  //Serial.print("Thermistor resistance (Ohms): ");
  //Serial.println(NTCResistance2);
  //Serial.print("Temperature (C): ");
  //Serial.println(temperature2);
  Serial.print("mVADC: ");
  Serial.println(ADCmVValue);
  Serial.print("referenceResistance: ");
  Serial.println(referenceResistance);
  Serial.print("Thermistor resistance (Ohms): ");
  Serial.println(NTCResistance);
  Serial.print("Temperature (C): ");
  Serial.println(temperature);
#endif

  return temperature;
}

double updateHeaterPID(double targetTemperature = 0) {
  unsigned long currentTime = millis();
  double elapsedTime = (currentTime - previousTime) / 300.0;
  previousTime = currentTime;

  if (targetTemperature == 0) {
    iTerm = 0;
    lastError = 0;
    return 0;
  }

  double error = targetTemperature - lastTemperature;

  // Proportional
  double pTerm = error;

  // Integral
  if (error > -5 && error < 5)
    iTerm += error * elapsedTime;
  
  // Anti-wind-up
  iTerm = constrain(iTerm, 0, 255.0 / kI);

  // Derivative
  double dTerm = (error - lastError) / elapsedTime; //max 5deg/sec -> -5

  // Calculate PID
  double PIDValue = kP * pTerm + kI * iTerm + kD * dTerm;

  // Deadband
  PIDValue = constrain(PIDValue, 0, 255) / 255.0;

  lastError = error;

  Serial.print("Heater Debug: ");
  Serial.print(PIDValue);
  Serial.print(",");
  Serial.print(pTerm);
  Serial.print(",");
  Serial.print(iTerm);
  Serial.print(",");
  Serial.println(dTerm);

#ifdef serialDebug
  Serial.print("Heater Debug: ");
  Serial.print(targetTemperature);
  Serial.print(",");
  Serial.print(PIDValue);
  Serial.print(",");
  Serial.println(lastTemperature);
#endif

#ifdef serialVerbose
  Serial.print("Target Temperature (C): ");
  Serial.println(targetTemperature);
  Serial.print("Heater State / PID Value (%): ");
  Serial.println(PIDValue * 100.0);
  Serial.print("Current Temperature (C): ");
  Serial.println(lastTemperature);
#endif

  return PIDValue;
}

void updateHeater(double value = 0) {
  unsigned long currentTime = millis();

  if (currentTime > heaterPreviousTime + heaterPeriodms) {
    heaterPreviousTime = currentTime;
    heaterSwitchTime = currentTime + heaterPeriodms * value;

    if (value > 0) {
      heaterPower(true);
    } else {
      heaterPower(false);
    }
  }

  if (currentTime > heaterSwitchTime) {
    heaterPower(false);
  }
}

void loadData() {
  for (int i = 0; i < 200; i++) {
    data[i] = EEPROM.read(i);
  }
}

void saveData(int profile) {
  for (int i = 0; i < 20; i++) {
    EEPROM.write(20 * profile + i, data[20 * profile + i]);
  }
  EEPROM.commit();
}

int getData(dataType type, int profile, int point = 0) {
  int temp = 0;
  switch (type) {
    case PROFILE_TEMP:
      for (int i = 0; i < 10; i++) {
        temp = max(temp, (int)data[profile * 20 + i * 2]);
      }
      return temp;
    case PROFILE_TIME:
      for (int i = 0; i < 10; i++) {
        temp += data[profile * 20 + i * 2 + 1];
      }
      return temp;
    case POINT_TEMP:
      return data[profile * 20 + point * 2];
    case POINT_TIME:
      return data[profile * 20 + point * 2 + 1];
  }
  return 0;
}

double getTargetTemperature(int profile, double time) {
  if ((int)time > getData(PROFILE_TIME, profile)) {
    return 0;
  }

  long temp1 = 0;
  long temp2 = 0;
  long time1 = 0;
  long time2 = 0;

  for (int i = 0; i < 10; i++) {
    if ((long)(time * 100) > time2) {
      time1 = time2;
      temp1 = temp2;
      time2 += data[profile * 20 + i * 2 + 1] * 100;
      temp2 = data[profile * 20 + i * 2] * 100;
    }
  }

  if (time1 == time2) {
    return temp1/100.0;
  }

#ifdef serialDebug
  Serial.print("Target Debug: ");
  Serial.print(time);
  Serial.print(",");
  Serial.print(time1/100);
  Serial.print(",");
  Serial.print(temp1/100);
  Serial.print(",");
  Serial.print(time2/100);
  Serial.print(",");
  Serial.print(temp2/100);
  Serial.print(",");
  Serial.println(map((long)(time * 100), time1, time2, temp1, temp2)/100.0);
#endif

  return map((long)(time * 100), time1, time2, temp1, temp2)/100.0;
}

void updateDisplay() {
  unsigned long currentTime = millis();

  display.clearDisplay();

  display.setCursor(0,selectedRow * 8);
  if (selected) {
    display.print("<      >");
  } else {
    display.print("[      ]");
  }

  int maxTemp = max(getData(PROFILE_TEMP, selectedProfile), 100);
  int maxTime = max(getData(PROFILE_TIME, selectedProfile), 60);
  int temp1 = 0;
  int temp2 = 0;
  int time1 = 0;
  int time2 = 0;

  for (int i = 0; i < 10; i++) {
    time1 = time2;
    temp1 = temp2;
    time2 += data[selectedProfile * 20 + i * 2 + 1];
    temp2 = data[selectedProfile * 20 + i * 2];
    int x1 = 80*time1/maxTime + 48;
    int y1 = 32 - (32*temp1/maxTemp);
    int x2 = 80*time2/maxTime + 48;
    int y2 = 32 - (32*temp2/maxTemp);
    display.drawLine(x1, y1, x2, y2, true);
  }

  if (startTime > 0) {
    display.drawFastHLine(48, 32 - (32*lastTemperature/maxTemp), 80, true);
    display.drawFastVLine(80*(currentTime - startTime)/maxTime / 1000 + 48, 0, 32, true);
  }

  switch (state) {
    case SELECT:
      display.setCursor(6,0);
      display.print("Prof");
      display.setCursor(36,0);
      display.printf("%d", selectedProfile);
      display.setCursor(0,8);
      display.printf("MAX %3dC", getData(PROFILE_TEMP, selectedProfile));
      display.setCursor(0,16);
      display.printf("DUR %3ds", min(getData(PROFILE_TIME, selectedProfile), 999));
      display.setCursor(0,24);
      display.printf("NTC %3dC", (int)lastTemperature);
      break;
    case MENU:
      display.setCursor(6,0);
      display.print("Prof");
      display.setCursor(36,0);
      display.printf("%d", selectedProfile);
      display.setCursor(6,8);
      display.print("Run");
      display.setCursor(6,16);
      display.print("Edit");
      display.setCursor(6,24);
      display.print("Cancel");
      break;
    case RUN:
      display.setCursor(6,0);
      display.print("Prof");
      display.setCursor(36,0);
      display.printf("%d", selectedProfile);
      display.setCursor(0,8);
      display.printf("%3d/%3dC", (int)lastTemperature, (int)getTargetTemperature(selectedProfile, (currentTime - startTime) / 1000.0));
      display.setCursor(0,16);
      display.printf("%3d/%3ds", (int)min((currentTime - startTime) / 1000, 999UL), min(getData(PROFILE_TIME, selectedProfile), 999));
      display.setCursor(6,24);
      display.print("Cancel");
      break;
    case EDIT:
    case EDITPNTTEMP:
    case EDITPNTTIME:
    case EDITSELPNT:
      display.setCursor(6,0);
      display.print("Pnt");
      display.setCursor(36,0);
      display.printf("%d", selectedPoint);
      display.setCursor(18,8);
      display.printf("%3dC", getData(POINT_TEMP, selectedProfile, selectedPoint));
      display.setCursor(18,16);
      display.printf("%3ds", getData(POINT_TIME, selectedProfile, selectedPoint));
      display.setCursor(6,24);
      display.print("Save");
      break;
  }

  display.display();
}

void updateButtons() {
  switch (buttonState) {
    case 1:
      switch (state) {
        case SELECT:
          selectedProfile = max(selectedProfile - 1, 0);
          break;
        case MENU:
          selectedRow = max(selectedRow - 1, 1);
          break;
        case RUN:
          break;
        case EDIT:
          selectedRow = max(selectedRow - 1, 0);
          break;
        case EDITPNTTEMP:
          data[20 * selectedProfile + 2 * selectedPoint] = max(data[20 * selectedProfile + 2 * selectedPoint] - 1, 0);
          break;
        case EDITPNTTIME:
          data[20 * selectedProfile + 2 * selectedPoint + 1] = max(data[20 * selectedProfile + 2 * selectedPoint + 1] - 1, 0);
          break;
        case EDITSELPNT:
          selectedPoint = max(selectedPoint - 1, 0);
          break;
      }
      break;
    case 2:
      switch (state) {
        case SELECT:
          state = MENU;
          selectedRow = 1;
          selected = false;
          break;
        case MENU:
          switch (selectedRow) {
            case 1:
              state = RUN;
              selectedRow = 3;
              selected = false;
              startTime = millis();
              break;
            case 2:
              state = EDITSELPNT;
              selectedRow = 0;
              selected = true;
              break;
            case 3:
              state = SELECT;
              selectedRow = 0;
              selected = true;
              break;
          }
          break;
        case RUN:
          state = SELECT;
          selectedRow = 0;
          selected = true;
          startTime = 0;
          break;
        case EDIT:
          switch (selectedRow) {
            case 0:
              state = EDITSELPNT;
              selected = true;
              break;
            case 1:
              state = EDITPNTTEMP;
              selected = true;
              break;
            case 2:
              state = EDITPNTTIME;
              selected = true;
              break;
            case 3:
              state = MENU;
              selectedRow = 1;
              selected = false;
              saveData(selectedProfile);
              break;
          }
          break;
        case EDITPNTTEMP:
        case EDITPNTTIME:
        case EDITSELPNT:
          state = EDIT;
          selected = false;
          break;
      }
      break;
    case 4:
      switch (state) {
        case SELECT:
          selectedProfile = min(selectedProfile + 1, 9);
          break;
        case MENU:
          selectedRow = min(selectedRow + 1, 3);
          break;
        case RUN:
          break;
        case EDIT:
          selectedRow = min(selectedRow + 1, 3);
          break;
        case EDITPNTTEMP:
          data[20 * selectedProfile + 2 * selectedPoint] = min(data[20 * selectedProfile + 2 * selectedPoint] + 1, 255);
          break;
        case EDITPNTTIME:
          data[20 * selectedProfile + 2 * selectedPoint + 1] = min(data[20 * selectedProfile + 2 * selectedPoint + 1] + 1, 255);
          break;
        case EDITSELPNT:
          selectedPoint = min(selectedPoint + 1, 9);
          break;
      }
    default:
      break;
  }
}

void readButtons() {
  #ifdef serialVerbose
    Serial.printf("Touch Value L/M/R: %3d/%3d/%3d\n", touchRead(touchButtonL), touchRead(touchButtonM), touchRead(touchButtonR));
  #endif

  //buttonState |= !digitalRead(buttonPinL) | (!digitalRead(buttonPinM) << 1) | (!digitalRead(buttonPinR) << 2);
  buttonState |= (touchRead(touchButtonL) < touchThresholdL) | ((touchRead(touchButtonM) < touchThresholdM) << 1) | ((touchRead(touchButtonR) < touchThresholdR) << 2);
  if (buttonState == 0) {
    buttonTime = 0;
  } else if (buttonTime == 0) {
    buttonTime = millis();
    updateButtons();
  } else if (millis() > buttonTime + 500) {
    updateButtons();
  }
  buttonState = 0;
}

void IRAM_ATTR buttonIntterupt() {
  //buttonState |= !digitalRead(buttonPinL) | (!digitalRead(buttonPinM) << 1) | (!digitalRead(buttonPinR) << 2);
  buttonState |= (touchRead(touchButtonL) < touchThresholdL) | ((touchRead(touchButtonM) < touchThresholdM) << 1) | ((touchRead(touchButtonR) < touchThresholdR) << 2);
}

void setup() {
  analogReadResolution(12);
  Serial.begin(115200);
#ifdef serialDebug
  Serial.begin(115200);
#else
#ifdef serialVerbose
  Serial.begin(115200);
#endif
#endif
  Wire.begin(OLEDSDA, OLEDSCL, 400000U);
  EEPROM.begin(200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
#ifdef serialDebug
    Serial.println(F("SSD1306 allocation failed"));
#endif
    for(;;);
  }

  display.clearDisplay();
  display.display();
  display.setTextColor(true);

  //pinMode(buttonPinL, INPUT_PULLUP);
  //pinMode(buttonPinM, INPUT_PULLUP);
  //pinMode(buttonPinR, INPUT_PULLUP);

  //attachInterrupt(buttonPinL, buttonIntterupt, FALLING);
  //attachInterrupt(buttonPinM, buttonIntterupt, FALLING);
  //attachInterrupt(buttonPinR, buttonIntterupt, FALLING);

  for (int i = 0; i < 10; i++) {
    touchThresholdL += touchRead(touchButtonL);
    touchThresholdM += touchRead(touchButtonM);
    touchThresholdR += touchRead(touchButtonR);
  }

  touchThresholdL /= 10;
  touchThresholdM /= 10;
  touchThresholdR /= 10;

  touchThresholdL -= 15;
  touchThresholdM -= 15;
  touchThresholdR -= 15;

#ifdef serialDebug
  Serial.printf("Touch Threshold L/M/R: %3d/%3d/%3d\n", touchThresholdL, touchThresholdM, touchThresholdR);
#endif

  touchAttachInterrupt(touchButtonL, buttonIntterupt, touchThresholdL);
  touchAttachInterrupt(touchButtonR, buttonIntterupt, touchThresholdR);
  touchAttachInterrupt(touchButtonM, buttonIntterupt, touchThresholdM);

  loadData();
}

void loop() {
  readButtons();

  readTemperature();

  unsigned long currentTime = millis();
  double time = 0;

  if (startTime > 0) {
    time = (currentTime - startTime) / 1000.0;

    if ((int)time > getData(PROFILE_TIME, selectedProfile)) {
      state = SELECT;
      selectedRow = 0;
      selected = true;
      startTime = 0;
      time = 0;
    }
  }

  double temp = getTargetTemperature(selectedProfile, time);
  double value = updateHeaterPID(temp);
  updateHeater(value);

  updateDisplay();
}