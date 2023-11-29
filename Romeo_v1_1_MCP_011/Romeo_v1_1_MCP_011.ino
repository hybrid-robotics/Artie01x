/*
  Program:      4WD Rover (DFRobot Baron Rover) Master Control Program (MCP)
  Date:         28-Jun-2014
  Version:      0.1.1 ALPHA

  Platform:     DFRobot Romeo v1.1 Microcntroller (Arduino Uno compatible)

  Purpose:      To have FUN with a little 4WD rover - can control the Rover over a serial link.

                                                  Change Log
                -------------------------------------------------------------------------------
                v0.1.0 ALPHA 27-Jun-2014:
                Initial build from example code on the DFRobot Wiki

                Added ms delay parameter to all motor functions to provide a delayed exit before
                  the next function.
                -------------------------------------------------------------------------------
                v0.1.1 ALPHA 28-Jun-2014:
                I've modified all my SSC-32 servo routines to work with the Arduino Servo library.

                I've cut the header file down to just what is needed here.
                -------------------------------------------------------------------------------

  Dependencies: Adafruit libraries:
                  Adafruit_Sensor, Adafruit_TMP006, and Adafruit_TCS34725, Adafruit_LEDBackpack,
                  Adafruit_GFX libraries

                Hybotics libraries:
                  None (Yet)

                Other libraries:
                  RTClib for the DS1307 (Adafruit's version)

  Comments:     Credit is given, where applicable, for code I did not originate.

  Copyright (c) 2014 Dale A. Weber <hybotics.pdx@gmail.com, @hybotics on App.Net and Twitter>
*/
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <BMSerial.h>

#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>

/*
  Additional sensors
*/
#include <Adafruit_TMP006.h>
#include <Adafruit_TCS34725.h>

#include "Romeo_v1_1_MCP_011.h"

//  Standard PWM DC control
int E1 = 5;                           //  M1 Speed Control
int M1 = 4;                           //  M1 Direction Control

int E2 = 6;                           //  M2 Speed Control
int M2 = 7;                           //  M2 Direction Control

/********************************************************************/
/*  Bitmaps for the drawBitMap() routines               */
/********************************************************************/

static const uint8_t PROGMEM
  hpa_bmp[] = {
    B10001110,
    B10001001,
    B11101110,
    B10101000,
    B00000100,
    B00001010,
    B00011111,
    B00010001
  },

  c_bmp[] = {
    B01110000,
    B10001000,
    B10000000,
    B10001000,
    B01110000,
    B00000000,
    B00000000,
    B00000000
  },

  f_bmp[] = {
    B11111000,
    B10000000,
    B11100000,
    B10000000,
    B10000000,
    B00000000,
    B00000000,
    B00000000
  },

  m_bmp[] = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B11101110,
    B10111010,
    B10010010,
    B10000010
  },

  date_bmp[] = {
    B10110110,
    B01001001,
    B01001001,
    B00000100,
    B00000100,
    B01111100,
    B10000100,
    B01111100
  },

  year_bmp[] = {
    B00000000,
    B10001000,
    B10001000,
    B01110000,
    B00101011,
    B00101100,
    B00101000,
    B00000000
  },

  am_bmp[] = {
    B01110000,
    B10001010,
    B10001010,
    B01110100,
    B00110110,
    B01001001,
    B01001001,
    B01001001
  },

  pm_bmp[] = {
    B01111100,
    B10000010,
    B11111100,
    B10000000,
    B10110110,
    B01001001,
    B01001001,
    B01001001
  },

  allon_bmp[] = {
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B11111111
  };

/************************************************************/
/*  Initialize global variables               */
/************************************************************/

/*
  These variables control the display of various information
    on the seven segment and matrix displays.
*/

//  Date display
boolean displayDate = true;
uint8_t dateMinuteCount = 0;

//  Time display
boolean displayTime = true;
uint8_t timeMinuteCount = 0;

//  Temperature display
boolean displayTemperature = true;
uint8_t temperatureMinuteCount = 0;

/*
  Time control variables
*/
uint8_t currentMinute = 0;
uint8_t lastMinute = -1;
long minuteCount = 0;           //  Count the time, in minutes, since we were last restarted

//  Enable run once only loop code to run
bool firstLoop = true;

//  True when the robot has not moved after an area scan
bool hasNotMoved = true;

//  This will always have the name of the last routine executed before an error
String lastRoutine;

//  Total number of area readings taken, or -1 if data is not valid
int nrAreaReadings;

//  Sharp GP2Y0A21YK0F IR range sensor readings
float ir[MAX_NUMBER_IR];

//  Area scan readings
AreaScanReading areaScan[MAX_NUMBER_AREA_READINGS];
bool areaScanValid = false;

/************************************************************/
/*  Initialize Objects                    */
/************************************************************/

Adafruit_TCS34725 rgb = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TMP006 heat = Adafruit_TMP006();

RTC_DS1307 clock;

//  Support for multiple 7 segment displays
Adafruit_7segment sevenSeg[MAX_NUMBER_7SEG_DISPLAYS];

Adafruit_8x8matrix matrix8x8 = Adafruit_8x8matrix();

/*
  Setup all our serial devices
*/

//  Hardware Serial0: Console and debug (replaces Serial.* routines)
BMSerial console(SERIAL_CONSOLE_RX_PIN, SERIAL_CONSOLE_TX_PIN);

//  Define the servo object for the pan servo
StandardServo mainPan;

Servo test;

/********************************************************************
  Basic movement routines - I've added the ms (millisecond) parameter 
    to provide away to time how long a move is.
/********************************************************************/

//  Stop 
void stop (void) {
  digitalWrite(E1, LOW);  
  digitalWrite(E2, LOW);     
}

//  Move forward
void forward (char a, char b, short ms = 0) {
  analogWrite (E1, a);                //  PWM Speed Control
  digitalWrite(M1, HIGH);   

  analogWrite (E2, b);   
  digitalWrite(M2, HIGH);

  if (ms > 0) {
    delay(ms);
  }
}

//  Move backward 
void reverse (char a, char b, short ms = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, LOW);  

  analogWrite (E2, b);   
  digitalWrite(M2, LOW);

  if (ms > 0) {
    delay(ms);
  }
}

//  Turn Left
void turnLeft (char a, char b, short ms = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, LOW);   

  analogWrite (E2, b);   
  digitalWrite(M2, HIGH);

  if (ms > 0) {
    delay(ms);
  }
}

//  Turn Right
void turnRight (char a, char b, short ms = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, HIGH);   

  analogWrite (E2, b);   
  digitalWrite(M2, LOW);

  if (ms > 0) {
    delay(ms);
  }
}

/*
  Display the TCS34725 RGB color sensor readings
*/
void displayColorSensorData (ColorSensor *colorData) {
  lastRoutine = String(F("displayColorSensorData"));

  console.println(F("Color Sensor Data:"));
  console.println();

  console.print(F("Color Temperature: "));
  console.print(colorData->colorTempC, DEC);
  console.print(F(" K - Lux: "));
  console.print(colorData->lux, DEC);
  console.print(F(" - Red: "));
  console.print(colorData->red, DEC);
  console.print(F(" Green: "));
  console.print(colorData->green, DEC);
  console.print(F(" Blue: "));
  console.print(colorData->blue, DEC);
  console.print(F(" C: "));
  console.println(colorData->c, DEC);
  console.println(F("."));
}

/*
  Display the TMP006 heat sensor readings
*/
void displayHeatSensorData (HeatSensor *heatData) {
  lastRoutine = String(F("displayHeatSensorData"));

  float objCelsius = heatData->objectTempC;
  float objFahrenheit = toFahrenheit(objCelsius);
  float dieCelsius = heatData->dieTempC;
  float dieFahrenheit = toFahrenheit(dieCelsius);

  console.println(F("Heat Sensor Data:"));
  console.println();

  console.print(F("Object Temperature: "));
  console.print(objFahrenheit);
  console.print(F(" F, "));
  console.print(objCelsius);
  console.println(F(" C."));
  console.print(F("Die Temperature: "));
  console.print(dieFahrenheit);
  console.print(F(" F, "));
  console.print(dieCelsius);
  console.println(F(" C."));
}

/*
    Display the GP2Y0A21YK0F IR sensor readings (cm)
*/
void displayIR (void) {
  uint8_t sensorNr = 0;

  lastRoutine = String("displayIR");
  
  console.println(F("IR Sensor readings:"));

  while (sensorNr < MAX_NUMBER_IR) {
    console.print(F("IR #"));
    console.print(sensorNr + 1);
    console.print(F(" range = "));
    console.print(ir[sensorNr]);
    console.println(F(" cm"));

    sensorNr += 1;
  }

  console.println();
}

ColorSensor readColorSensor (void) {
  ColorSensor colorData;

  rgb.getRawData(&colorData.red, &colorData.green, &colorData.blue, &colorData.c);
  colorData.colorTempC = rgb.calculateColorTemperature(colorData.red, colorData.green, colorData.blue);
  colorData.lux = rgb.calculateLux(colorData.red, colorData.green, colorData.blue);

  return colorData;
}

HeatSensor readHeatSensor (void) {
  HeatSensor heatData;

  heatData.dieTempC = heat.readDieTempC();
  heatData.objectTempC = heat.readObjTempC();

  return heatData;
}

/*
  Read distance in cm from a Sharp GP2Y0A21YK0F IR sensor
*/
float readSharpGP2Y0A21YK0F (byte sensorNr) {
  byte pin = sensorNr + IR_PIN_BASE;
  int reading = analogRead(pin);
  float distance = (6762.0 / (reading - 9)) - 4;

  lastRoutine = String(F("readSharpGP2Y0A21YK0F"));

  return distance;
}

/********************************************************************/
/*  Servo routines                                                  */
/********************************************************************/

/*
    Move a servo by pulse width in ms (500ms - 2500ms) - Modified to use HardwareSerial2()
*/
uint16_t moveServoPw (StandardServo *servo, int servoPosition) {
  uint16_t errorStatus = 0;
  char asciiCR = 13;
  int position = servoPosition + servo->offset;

  lastRoutine = String(F("moveServoPw"));

  servo->error = 0;
  
  if ((position >= servo->minPulse) && (position <= servo->maxPulse)) {
    if (servo->maxDegrees == 180) {
      servo->angle += 90;
    }
  }

  if ((position < servo->minPulse) || (position > servo->maxPulse)) {
    errorStatus = 201;
    processError(errorStatus, F("Servo pulse is out of range"));
  } else {
    //  Move the servo
    console.print("(");
    console.print(lastRoutine);
    console.print(") Moving the ");
    console.print(servo->descr);
    console.print(" servo to position ");
    console.print(servoPosition);
    console.println(" uS..");

    servo->servo.writeMicroseconds(position);
  }

  if (errorStatus != 0) {
    servo->error = errorStatus;
  }

  return errorStatus;
}

/*
    Move a servo by degrees (-90 to 90) or (0 - 180) - Modified to use BMSerial
*/
uint16_t moveServoDegrees (StandardServo *servo, int servoDegrees) {
  uint16_t servoPulse;

  uint16_t errorStatus = 0;
  String errorMsg;

  lastRoutine = String(F("moveServoDegrees"));

  servo->error = 0;
  
  //  Convert degrees to ms for the servo move
  if (servo->maxDegrees == 90) {
    servoPulse = SERVO_CENTER_MS + (servoDegrees * 11);
  } else if (servo->maxDegrees == 180) {
    servoPulse = SERVO_CENTER_MS + ((servoDegrees -90) * 11);
  } else {
    errorStatus = 202;
  }

  if (errorStatus != 0) {
    processError(errorStatus, F("Servo position (degrees) is invalid"));
  } else {
    if ((servoPulse >= servo->minPulse) && (servoPulse <= servo->maxPulse)) {
      errorStatus = moveServoPw(servo, servoPulse);

      if (errorStatus != 0) {
        processError(errorStatus, "Could not move the " + servo->descr + " servo");
      }
    }
  }

  return errorStatus;
}

DistanceObject findDistanceObjects () {
  uint8_t readingNr;

  DistanceObject distObj = {0, 0, 0, 0, 0, 0, 0, 0};

  console.println(F("Finding the closest and farthest objects.."));

  //  Find the closest and farthest objects
  for (readingNr = 0; readingNr <= nrAreaReadings; readingNr++) {
    //  Check for the closest object
    if (areaScan[readingNr].ir <=  areaScan[distObj.closestIR].ir) {
      distObj.closestIR = readingNr;
      distObj.closestPosIR = areaScan[readingNr].positionDeg;
    }

    if (areaScan[readingNr].ir > areaScan[distObj.farthestIR].ir) {
      distObj.farthestIR = readingNr;
      distObj.farthestPosIR = areaScan[readingNr].positionDeg;
    }
  }

  return distObj;
}

/*
  Scan an arc of up to 180 degrees, and take sensor readings at each angle increment
*/
uint16_t scanArea (StandardServo *pan, int startDeg, int stopDeg, int incrDeg) {
  uint16_t errorStatus = 0;
  uint16_t readingNr = 0, nrReadings = 0;
  int positionDeg = 0;
  int totalRangeDeg = 0;

  lastRoutine = String(F("scanArea"));

  //  Check the parameters
  if (startDeg > stopDeg) {
    //  Start can't be greater than stop
    errorStatus = 401;
  } else if (((pan->maxDegrees == 90) && ((startDeg < -90) || (stopDeg > 90))) || ((pan->maxDegrees == 180) && ((startDeg < 0) || (stopDeg > 180)))) {
    //  One or more parameters is outside of the valid range
    errorStatus = 402;
  } else if ((startDeg < pan->minPulse) || (stopDeg > pan->maxPulse)) {
    //  Out of range for the pan servo
    errorStatus = 403;
  } else {
    //  Calculate the total range, in degrees
    totalRangeDeg = abs(stopDeg - startDeg);

    //  Calculate the number of readings we need room for
    nrReadings = totalRangeDeg / incrDeg;

    //  More error checking
    if (totalRangeDeg > 180) {
      //  Servos can only move up to 180 degrees
      errorStatus = 404;
    } else if (nrReadings > MAX_NUMBER_AREA_READINGS) {
      //  We can't take this many readings
      errorStatus = 405;
    } else if (incrDeg > totalRangeDeg) {
      //  Increment is too large for our range
      errorStatus = 406;
    } else {
      /*
        Continue normal processing
      */

      //  Stop, so we can do this scan
      stop();

      readingNr = 0;

      console.println(F("Scanning the area.."));

      for (positionDeg = startDeg; positionDeg < stopDeg; positionDeg += incrDeg) {
        errorStatus = moveServoDegrees(pan, positionDeg);

        if (errorStatus != 0) {
          processError(errorStatus, "Could not move the " + pan->descr + " servo");
          break;
        } else {
          //  Delay to let the pan/tilt stabilize after moving it
          delay(1500);

          //  Take a reading from the pan sensor in cm
          areaScan[readingNr].ir = readSharpGP2Y0A21YK0F(IR_FRONT_CENTER);
          areaScan[readingNr].positionDeg = positionDeg;

          if (HAVE_COLOR_SENSOR) {
            areaScan[readingNr].color = readColorSensor();
          }

          if (HAVE_HEAT_SENSOR) {
            areaScan[readingNr].heat = readHeatSensor();
          }

          readingNr += 1;
        }
      }

      //  Send the pan servo back to home position
      errorStatus = moveServoPw(pan, pan->homePos);
    }
  }

  if (errorStatus != 0) {
    processError(errorStatus, F("Could not complete the area scan"));
    nrAreaReadings = -1;
    areaScanValid = false;
  } else {
    //  Robot has not moved
    hasNotMoved = true;

    //  Set the number of readings taken
    nrAreaReadings = readingNr;

    //  This area scan is valid
    areaScanValid = true;
  }

  return errorStatus;
}

/*
  Turn towards the farthest detected object
*/
uint16_t turnToFarthestObject (DistanceObject *distObj, StandardServo *pan) {
  uint16_t errorStatus = 0;

  if (distObj->farthestPosPING < 0) {
    //  Turn to the right
    turnRight(50, 50, 250);
    forward(50, 50);
  } else if (distObj->farthestPosPING > 0) {
    //  Turn to the left
    turnLeft(50, 50, 250);
    forward(50, 50);
  } else {
    //  Backup and scan again
    stop();
    reverse(50, 50, 250);
    stop();

    errorStatus = scanArea(pan, -90, 90, AREA_SCAN_DEGREE_INCREMENT);
  }

  return errorStatus;
}

/********************************************************************/
/*  Miscellaneous routines                                          */
/********************************************************************/

/*
    Pulses a digital pin for a duration in ms
*/
void pulseDigital(int pin, int duration) {
  lastRoutine = String(F("pulseDigital"));

  digitalWrite(pin, HIGH);      // Turn the ON by making the voltage HIGH (5V)
  delay(duration);          // Wait for duration ms
  digitalWrite(pin, LOW);       // Turn the pin OFF by making the voltage LOW (0V)
  delay(duration);          // Wait for duration ms
}

/*
    Trim trailing zeros from a numeric string
*/
String trimTrailingZeros (String st) {
  uint8_t newStrLen = 0;
  String newStr = st;

  lastRoutine = String(F("trimTrailingZeros"));

  newStrLen = newStr.length();

  while (newStr.substring(newStrLen - 1) == "0") {
    newStrLen -= 1;
    newStr = newStr.substring(0, newStrLen);
  }

  return newStr;
}

/*
  Convert a temperature in Celsius to Fahrenheit
*/
float toFahrenheit (float celsius) {
  lastRoutine = String(F("toFahrenheit"));

  return (celsius * 1.8) + 32;
}

/*
    Process error conditions
*/
void processError (byte errCode, String errMsg) {
  console.print(F("Error in routine '"));
  console.print(lastRoutine);
  console.print(F("', Code: "));
  console.print(errCode);
  console.print(F(", Message: "));
  console.print(errMsg);
  console.println(F("!"));
}

/*
  Wait for a bit to allow time to read the Console Serial Monitor log
*/
void wait (uint8_t nrSeconds, String text = "") {
  uint8_t count;

  lastRoutine = String(F("wait"));

  console.print(F("Waiting"));

  if (text != "") {
    console.print(F(" for "));
    console.print(text);
  }

  for (count = 0; count < nrSeconds; count++) {
    console.print(F("."));
    delay(1000);
  }

  console.println();
}

/*
  Show announcement message on the desired port.
*/
void announce (BMSerial *port) {
  port->println();
  port->print("4WD Rover Master Control Program (MCP), version ");
  port->print(BUILD_VERSION);
  port->print(" on ");
  port->println(BUILD_DATE);
  port->print("  for the ");
  port->print(BUILD_BOARD);
  port->println(".");
  port->println();

  port->println("Keyboard control is open..");
}

/*
  Clear all the seven segment and matrix displays
*/
void clearDisplays (void) {
  uint8_t nrDisp = 0;

  while (nrDisp < MAX_NUMBER_7SEG_DISPLAYS) {
    sevenSeg[nrDisp].clear();
    sevenSeg[nrDisp].drawColon(false);
    sevenSeg[nrDisp].writeDisplay();

    nrDisp += 1;
  }

  matrix8x8.clear();
  matrix8x8.writeDisplay();
}

/*
  Test all the displays
*/
void testDisplays (uint8_t totalDisplays) {
  uint8_t nrDisp = 0;

  console.println(F("Testing All Displays"));

  while (nrDisp < totalDisplays) {
    sevenSeg[nrDisp].print(8888);
    sevenSeg[nrDisp].drawColon(true);
    sevenSeg[nrDisp].writeDisplay();

    nrDisp += 1;
  }

  matrix8x8.drawBitmap(0, 0, allon_bmp, 8, 8, LED_ON);
  matrix8x8.writeDisplay();

  delay(2000);

  clearDisplays();
}

/*
  Check for serial port manual commands
*/
void checkSerial (short leftSpd, short rightSpd) {
  char command = Serial.read();

  if (command != -1) {
    switch(command) {
      case 'w':                     //  Move Forward
      case 'W':
        forward(leftSpd, rightSpd);
        break;

      case 's':                     //  Move Backward
      case 'S':
        reverse(leftSpd, rightSpd);
        break;

      case 'a':                     //  Turn Left
      case 'A':
        turnLeft(leftSpd, rightSpd);
        break;      

      case 'd':                     //  Turn Right
      case 'D':
        turnRight(leftSpd, rightSpd);
        break;

      case 'z':
      case 'Z':
        Serial.println("Hello");
        break;

      case 'x':
      case 'X':
        stop();
        break;

      default:
        Serial.println("Invalid command received!");
        stop();
        break;
    }
  } else {
    stop();
  }
}

/*
  Initialize displays

  Multiple 7 segment displays will be supported. The displays
    should be on the breadboard, starting at the right with
    the lowest addressed display and going to the left.

*/
void initDisplays (uint8_t totalDisplays) {
  uint8_t nrDisp = 0;
  uint8_t address;

  console.println(F("Initializing Displays.."));

  while (nrDisp < totalDisplays) {
    sevenSeg[nrDisp] = Adafruit_7segment();
    address = SEVEN_SEG_BASE_ADDR + nrDisp;
    sevenSeg[nrDisp].begin(address);
    sevenSeg[nrDisp].setBrightness(5);
    sevenSeg[nrDisp].drawColon(false);

    nrDisp += 1;
  }

  /*
    The matrix display address is one higher than the last
      seven segment display, based on the number of seven
      seven segment displays that are configured.
  */
  matrix8x8.begin(MATRIX_DISPLAY_ADDR);
  matrix8x8.setBrightness(5);
  matrix8x8.setRotation(3);
}

/*
  Initialize sensors
*/
uint16_t initSensors (void) {
  uint16_t errorStatus = 0;

  lastRoutine = String(F("initSensors"));

  console.println(F("Initializing Sensors.."));

  if ((errorStatus == 0) && (HAVE_COLOR_SENSOR)) {
    //  Initialize the TCS3725 RGB Color sensor (Adafruit)
    if (! rgb.begin()) {
      errorStatus = 604;
      processError(errorStatus, F("There was a problem initializing the TCS34725 RGB color sensor .. check your wiring or I2C Address!"));
    }
  }

  if ((errorStatus == 0) && (HAVE_HEAT_SENSOR)) {
    //  Initialize the TMP006 heat sensor
    if (! heat.begin()) {
      errorStatus = 605;
      processError(errorStatus, F("There was a problem initializing the TMP006 heat sensor .. check your wiring or I2C Address!"));
    }
  }

  if ((errorStatus == 0) && (HAVE_DS1307_RTC)) {
    console.println(F("     DS1307 Real Time Clock.."));

    //  Check to be sure the RTC is running
    if (! clock.isrunning()) {
      errorStatus = 606;
      processError(errorStatus, F("The Real Time Clock is NOT running!"));
    }
  }

  return errorStatus;
}

/*
  Initialize servos to defaults
*/
void initServos (void) {
  lastRoutine = String(F("initServos"));

  mainPan.pin = SERVO_MAIN_PAN_PIN;
  mainPan.descr = String(SERVO_MAIN_PAN_NAME);
  mainPan.offset = SERVO_MAIN_PAN_OFFSET;
  mainPan.homePos = SERVO_MAIN_PAN_HOME;
  mainPan.msPulse = 0;
  mainPan.angle = 0;
  mainPan.minPulse = SERVO_MAIN_PAN_RIGHT_MIN;
  mainPan.maxPulse = SERVO_MAIN_PAN_LEFT_MAX;
  mainPan.maxDegrees = SERVO_MAX_DEGREES;
  mainPan.error = 0;

  //  Set the servo to home position
  pinMode(SERVO_MAIN_PAN_PIN, OUTPUT);
  mainPan.servo.attach(SERVO_MAIN_PAN_PIN);
  moveServoPw(&mainPan, SERVO_MAIN_PAN_HOME);
}

/********************************************************************/
/*  Runs once to set things up                                      */
/********************************************************************/
void setup(void) {
  int i;

  //  Set motor control pins to OUTPUTs
  for(i = 4; i <= 7; i++) {
    pinMode(i, OUTPUT); 
  }

  uint16_t errorStatus = 0;
  uint8_t loopCount = 0;

  lastRoutine = String(F("SETUP"));

  //  Initialize the console port
  console.begin(9600);
  announce(&console);

  //  Delay for a few seconds, before starting initialization
  console.println();
  wait(STARTUP_DELAY_SECONDS, "initialization");

  console.println(F("Initializing Digital Pins.."));

  //  Initialize the LED pin as an output.
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, LOW);

  if (HAVE_COLOR_SENSOR) {
    //  Initialize and turn off the TCS34725 RGB Color sensor's LED
    pinMode(COLOR_SENSOR_LED, OUTPUT);
    digitalWrite(COLOR_SENSOR_LED, LOW);
    delay(250);
    digitalWrite(COLOR_SENSOR_LED, HIGH);
    delay(250);
    digitalWrite(COLOR_SENSOR_LED, LOW);
  }

  if (HAVE_7SEGMENT_DISPLAYS) {
    //  Initialize the displays
    initDisplays(MAX_NUMBER_7SEG_DISPLAYS);

    //  Test the displays
    testDisplays(MAX_NUMBER_7SEG_DISPLAYS);
  }

  //  Initialize all servos
  initServos();

  //  Initialize all sensors
  errorStatus = initSensors();

  //  Do an initial scan of the immediate area
  scanArea(&mainPan, -90, 90, AREA_SCAN_DEGREE_INCREMENT);
}

/********************************************************************/
/*  Runs forever                                                    */
/********************************************************************/
void loop (void) {
/*
  //  Check for a manual control command from the serial link
  if (Serial.available()) {
    checkSerial(50, 50);
  }

  console.println(F("Getting Distance Sensor readings.."));

  //  Get readings from all the GP2Y0A21YK0F Analog IR range sensors, if any, and store them
  if (MAX_NUMBER_IR > 0) {
    for (analogPin = 0; analogPin < MAX_NUMBER_IR; analogPin++) { 
      ir[analogPin] = readSharpGP2Y0A21YK0F(analogPin);
    }

    displayIR();
  }

  if (MAX_NUMBER_IR > 0) {
    //  Let's see if we're too close to an object.. If so, stop and scan the area
    if (ir[IR_FRONT_CENTER] < IR_MIN_DISTANCE_CM) {
      stop();

      //  Scan the area for a clear path
      errorStatus = scanArea(&mainPan, -90, 90, AREA_SCAN_DEGREE_INCREMENT);

      if (errorStatus != 0) {
        processError(errorStatus, F("Unable to scan the area"));
      } else {
        //  Find the closest and farthest objects
        distObject = findDistanceObjects();

        errorStatus = turnToFarthestObject(&distObject, &mainPan);

        if (errorStatus != 0) {
          processError(errorStatus, F("Could not complete a turn to the farthest object"));

          stop();

          if (errorStatus != 0) {
            runAwayRobot(errorStatus);
          }
        }
      }
    }
  }
*/
}
