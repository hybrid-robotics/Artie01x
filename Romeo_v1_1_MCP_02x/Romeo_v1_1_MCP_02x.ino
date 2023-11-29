 /*
  Program:      A.R.T.I.E. (DFRobot Baron Rover) Master Control Program (MCP)
  Date:         23-Jul-2014
  Version:      0.2.5 ALPHA

  Platform:     DFRobot Romeo v1.1 Microcontroller (Arduino Uno compatible)

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
				        v0.2.0 ALPHA 29-Jun-2014:
				        Starting to work on roving behaviors. Also need to test the IR sensor. There is
					        a slight glitch in the area scanner - it doesn't go all the way to the right.

                Added a manual control toggle command.
				        -------------------------------------------------------------------------------
                v0.2.1 ALPHA 30-Jun-2014:
                Added wheel encoder support.
                Checks encoders once each loop, to see if the wheels are still moving.
                This little 4WD Rover can now get out of *most* stuck conditions on its own.
                -------------------------------------------------------------------------------
                v0.2.2 ALPHA 03-Jul-2014:
                Changed platform specific routine names to have a prefix. For the Romeo v1.1 All In One
                  controller, this is "romeoV11" and for the Romeo v2, it will be "romeoV2" if that
                  is necessary.
                ---------------------------------------------------------------------------------
                v0.2.3 ALPHA 06-Jul-2014:
                Changed the check for moving wheels to include moving forward or reverse, depending
                  on which way the rover was moving last.
                ---------------------------------------------------------------------------------
                v0.2.4 ALPHA 08-Jul-2014:
                Changed movingDirection to roverStatus, typeDef Direction to typeDef Status.

                Added statuses Idle, Error, and Scanning.

                No longer checks the encoder if status is Idle or Scanning.

                The processError() routine now sets roverStatus = Error.
                ---------------------------------------------------------------------------------
                v0.2.5 ALPHA 11-Jul-2014:
                Added setting SERVO_MAIN_PAN_STABLE_MS in the header file to set a delay for the pan
                  servo to become stable before doing anything.

                Added definition and code to initialize the camera tilt servo.
                ---------------------------------------------------------------------------------

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

#include "Romeo_v1_1_MCP_02x.h"
#include "Pitches.h"

/********************************************************************/
/*  Bitmaps for the drawBitMap() routines                           */
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
/*  Initialize global variables                             */
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
bool startingUp = true;

//  This will always have the name of the last routine executed before an error
String lastRoutine;

//  PING Ultrasonic range sensor readings
int ping[MAX_NUMBER_PING];

//  Total number of area readings taken, or -1 if data is not valid
int nrAreaReadings;

//  Sharp GP2Y0A21YK0F IR range sensor readings
float ir[MAX_NUMBER_IR];

//  Area scan readings
AreaScanReading areaScan[MAX_NUMBER_AREA_READINGS];
bool areaScanValid = false;

/************************************************************/
/*  Rover control variables                                 */
/************************************************************/

//  Rover status - Idle, Forward, Reverse, Scanning, Stopped, TurningLeft, or TurningRight
Status roverStatus = Idle;

//  Toggle for Manual Control
bool manualControl = false;

//  Encoder variables
long encoder[2] = { 0, 0 };
int lastSpeed[2] = { 0, 0 }; 

//  Rover standard PWM DC motor control pins
int leftPinE1 = 5;                           //  M1 Speed Control
int leftPinM1 = 4;                           //  M1 Direction Control

int rightPinE2 = 6;                          //  M2 Speed Control
int rightPinM2 = 7;                          //  M2 Direction Control

/************************************************************/
/*  Initialize Objects                                      */
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
StandardServo mainPan, irEyeTilt;

/********************************************************************/
/*  Basic movement routines - I've added the ms (millisecond)       */
/*    parameter to provide away to time how long a move is.         */
/*                                                                  */
/*  Rover is configured for standard PWM motor control.             */
/********************************************************************/

//  Move forward
void romeoV11Forward (char leftSpeed, char rightSpeed, uint16_t durationMS = 0) {
  analogWrite (leftPinE1, leftSpeed);
  digitalWrite(leftPinM1, HIGH);   

  analogWrite (rightPinE2, rightSpeed);   
  digitalWrite(rightPinM2, HIGH);

  roverStatus = Forward;
/*
  if (durationMS > 0) {
    delay(durationMS);
  } else {
    delay(ROVER_DEFAULT_MOVE_TIME_MS);
  }
*/
}

//  Move backward 
void romeoV11Reverse (char leftSpeed, char rightSpeed, uint16_t durationMS = 0) {
  analogWrite (leftPinE1, leftSpeed);
  digitalWrite(leftPinM1, LOW);  

  analogWrite (rightPinE2, rightSpeed);   
  digitalWrite(rightPinM2, LOW);

  if (durationMS > 0) {
    delay(durationMS);
  } else {
    delay(ROVER_DEFAULT_MOVE_TIME_MS);
  }

  roverStatus = Reverse;
}

/*
  Stop
*/ 
void romeoV11Stop (void) {
  digitalWrite(leftPinE1, LOW);  
  digitalWrite(rightPinE2, LOW);

  roverStatus = Stopped;
}

/*
  Turn Left for a period of time in ms
*/
void romeoV11TurnLeft (char leftSpeed, char rightSpeed, uint16_t durationMS = 0) {
  analogWrite (leftPinE1, leftSpeed);
  digitalWrite(leftPinM1, LOW);   

  analogWrite (rightPinE2, rightSpeed);   
  digitalWrite(rightPinM2, HIGH);

  if (durationMS > 0) {
    delay(durationMS);
  } else {
    delay(ROVER_DEFAULT_MOVE_TIME_MS);
  }

  roverStatus = TurningLeft;
}

/*
  Turn Right for a period of time in ms
*/
void romeoV11TurnRight (char leftSpeed, char rightSpeed, uint16_t durationMS = 0) {
  analogWrite (leftPinE1, leftSpeed);
  digitalWrite(leftPinM1, HIGH);   

  analogWrite (rightPinE2, rightSpeed);   
  digitalWrite(rightPinM2, LOW);

  if (durationMS > 0) {
    delay(durationMS);
  } else {
    delay(ROVER_DEFAULT_MOVE_TIME_MS);
  }

  roverStatus = TurningRight;
}

/*
  Backup for a period of time in ms
*/
void romeoV11BackUp (short leftSpeed, short rightSpeed, uint16_t durationMS = 0) {
  //  Back up a bit
  romeoV11Reverse(leftSpeed, rightSpeed, durationMS);

  roverStatus = Reverse;
}

/*
  Make a random turn for a period of time in ms
*/
void romeoV11MakeRandomTurn (short leftSpeed, short rightSpeed, uint16_t durationMS = 0) {
  if (random(1000) < 500) {
    romeoV11TurnLeft(leftSpeed, rightSpeed, durationMS);
  } else {
    romeoV11TurnRight(leftSpeed, rightSpeed, durationMS);
  }
}

/********************************************************************/
/*  Wheel interrupt encoder routines                                */
/********************************************************************/

/*
  Interrupt routine for the left encoder
*/
void LeftWheelSpeed (void) {
  //  Count the left wheel encoder interrupts
  encoder[WHEEL_ENCODER_LEFT]++;
}

/*
  Interrupt routine for the right encoder
*/
void RightWheelSpeed (void) {
  //  Count the right wheel encoder interrupts
  encoder[WHEEL_ENCODER_RIGHT]++;
}

/*
  Display the current wheel encoder values
*/
void displayEncoders (void) {
    console.println();
    console.println(F("Encoder values:"));
    console.print(F("     Left Wheel = "));
    console.print(encoder[WHEEL_ENCODER_LEFT]);
    console.print(F(", Right Wheel = "));
    console.print(encoder[WHEEL_ENCODER_RIGHT]);
    console.println(F("."));

    console.println(F("Last Speed values:"));
    console.print(F("     Left Wheel = "));
    console.print(lastSpeed[WHEEL_ENCODER_LEFT]);
    console.print(F(", Right Wheel = "));
    console.print(lastSpeed[WHEEL_ENCODER_RIGHT]);
    console.println(F("."));

    console.println();
}

/*
  Read the encoders and reset the data buffer
*/
unsigned long readEncoders (unsigned long timerValue) {
  if ((millis() - timerValue) > 100) {                  
    displayEncoders();

    //  Record the latest encoder values
    lastSpeed[WHEEL_ENCODER_LEFT] = encoder[WHEEL_ENCODER_LEFT];
    lastSpeed[WHEEL_ENCODER_RIGHT] = encoder[WHEEL_ENCODER_RIGHT];

    //  Clear the data buffer
    encoder[WHEEL_ENCODER_LEFT] = 0;
    encoder[WHEEL_ENCODER_RIGHT] = 0;

    //  Return the current value of the timer
    return millis();
  }
}

/*
  Check to see if the wheels are still moving
*/
bool wheelsAreMoving (void) {
  console.println(F("Checking wheel motion.."));
  displayEncoders();

  return ((encoder[WHEEL_ENCODER_LEFT] > lastSpeed[WHEEL_ENCODER_LEFT]) && (encoder[WHEEL_ENCODER_RIGHT] > lastSpeed[WHEEL_ENCODER_RIGHT]));
}

/************************************************************/
/*  Sound Generation routines               */
/************************************************************/

int r2d2[] = { 16, NOTE_A7, NOTE_G7, NOTE_E7, NOTE_C7, NOTE_D7, NOTE_B7, NOTE_F7, NOTE_C8, NOTE_A7, NOTE_G7, NOTE_E7, NOTE_C7, NOTE_D7, NOTE_B7, NOTE_F7, NOTE_C8 };
int tones[] = { 20, NOTE_C4, NOTE_CS4, NOTE_D4, NOTE_DS4, NOTE_E4, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_GS4, NOTE_A4, NOTE_C5, NOTE_CS5, NOTE_D5, NOTE_DS5, NOTE_F5, NOTE_FS5, NOTE_G5, NOTE_A5, NOTE_C6, NOTE_CS6, NOTE_D6, NOTE_DS6, NOTE_F6, NOTE_FS6, NOTE_G6, NOTE_A6, NOTE_C7, NOTE_CS7, NOTE_D7, NOTE_DS7, NOTE_F7, NOTE_FS7, NOTE_G7, NOTE_A7 };
          // mid C C# D D# E F F# G G# A

/*
  Play a single tone.
*/
void playTone (unsigned long freqHz, uint8_t volume, unsigned long durationMS) {
//  toneAC(freqHz, volume, durationMS);
//  noToneAC();
}

/*
  Play a sequence of tones.
*/
void playSequence (int song[], unsigned long durationMS) {
  int numTones = song[0];
  int toneNr;

  for (toneNr = 1; toneNr < numTones; toneNr++) {
    playTone(song[toneNr], 10, durationMS);
  }
}

/*
  Play a sound - a sequence of pitches (or notes)
*/
uint16_t makeSound (uint8_t soundNr, unsigned long durationMS, uint8_t nrTimes, uint32_t sequenceDelayMS) {
  uint16_t errorStatus = 0;
  uint8_t volume = 10;
  uint8_t count;

  for (count = 0; count < nrTimes; count++) {
    switch (soundNr) {
      case 1:
        //  Startup
        playSequence(r2d2, 150);
        delay(150);
        playSequence(r2d2, 150);
        break;

      case 2:
        //  Alarm
        playTone(NOTE_D8, volume, durationMS);
        delay(150);
        playTone(NOTE_D8, volume, durationMS);
        break;

      default:
        errorStatus = 901;
        break;
    }

    if (errorStatus != 0){
      processError(errorStatus, F("Invalid sound number"));
      break;
    } else {
      delay(sequenceDelayMS);
    }
  }

  return errorStatus;
}

/*
  Sound an alarm when we need assistance
*/
uint16_t soundAlarm (uint8_t nrAlarms) {
  uint16_t errorStatus = 0;
  uint8_t alarmCount;

  for (alarmCount = 0; alarmCount < nrAlarms; alarmCount++) {
    makeSound(1, 150, nrAlarms, 5000);
  }

  return errorStatus;
}

/*
  Call for help!

  We're in a situation we can't get out of on our own.
*/
uint16_t callForHelp (uint8_t nrCalls, uint8_t nrAlarms, uint32_t callDelaySeconds) {
  uint16_t errorStatus = 0;
  uint8_t count;

  //  Send out a call for help every 20 seconds
  for (count = 0; count < nrCalls; count++) {
    console.println(F("Help, help, help! I am stuck!"));

    soundAlarm(nrAlarms);

    //  Delay between calls for help
    delay(callDelaySeconds * 1000);
  }

  return errorStatus;
}

/*
  We can't stop the motors!
*/
void runAwayRobot (uint16_t errorStatus) {
  romeoV11Stop();

  processError(errorStatus, F("Runaway robot"));

  callForHelp(2, 5, 10);
}

/********************************************************************/
/*  Console display routines                                        */
/********************************************************************/

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

/*
  Display the readings from the PING Ultrasonic sensors
*/
void displayPING (void) {
  uint8_t sensorNr = 0;

  lastRoutine = String(F("displayPING"));
  
  console.println(F("PING Ultrasonic Sensor readings:"));
  
  //  Display PING sensor readings (cm)
  while (sensorNr < MAX_NUMBER_PING) {
    console.print(F("Ping #"));
    console.print(sensorNr + 1);
    console.print(F(" range = "));
    console.print(ping[sensorNr]);
    console.println(F(" cm"));

    sensorNr += 1;
  }
 
  console.println();
}

/********************************************************************/
/*  Sensor related routines                                         */
/********************************************************************/

/*
    Convert a pulse width in ms to inches
*/
long microsecondsToInches (long microseconds) {
  /*
    According to Parallax's datasheet for the PING))), there are
      73.746 microseconds per inch (i.e. sound travels at 1130 feet per
      second).  This gives the distance travelled by the ping, outbound
      and return, so we divide by 2 to get the distance of the obstacle.
    See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  */

  lastRoutine = String(F("microsecondsToInches"));
  
  return microseconds / 74 / 2;
}

/*
    Convert a pulse width in ms to a distance in cm
*/
long microsecondsToCentimeters (long microseconds) {
  /*
    The speed of sound is 340 m/s or 29 microseconds per centimeter.

    The ping travels out and back, so to find the distance of the
      object we take half of the distance travelled.
  */

  lastRoutine = String(F("microsecondsToCentimeters"));

  return microseconds / 29 / 2;
}

/*
  Read the TCS74325 RGB Color sensor
*/
ColorSensor readColorSensor (void) {
  ColorSensor colorData;

  rgb.getRawData(&colorData.red, &colorData.green, &colorData.blue, &colorData.c);
  colorData.colorTempC = rgb.calculateColorTemperature(colorData.red, colorData.green, colorData.blue);
  colorData.lux = rgb.calculateLux(colorData.red, colorData.green, colorData.blue);

  return colorData;
}

/*
  Read the TMP006 Heat sensor
*/
HeatSensor readHeatSensor (void) {
  HeatSensor heatData;

  heatData.dieTempC = heat.readDieTempC();
  heatData.objectTempC = heat.readObjTempC();

  return heatData;
}

/*
  Read a Parallax Ping))) Sensor 

  This routine reads a PING))) ultrasonic rangefinder and returns the
    distance to the closest object in range. To do this, it sends a pulse
    to the sensor to initiate a reading, then listens for a pulse
    to return.  The length of the returning pulse is proportional to
    the distance of the object from the sensor.

  The circuit:
    * +V connection of the PING))) attached to +5V
    * GND connection of the PING))) attached to ground
    * SIG connection of the PING))) attached to digital pin 7

  http://www.arduino.cc/en/Tutorial/Ping

  Created 3 Nov 2008
    by David A. Mellis

  Modified 30-Aug-2011
    by Tom Igoe

  Modified 09-Aug-2013
    by Dale Weber

    Set units = true for cm, and false for inches
*/
int readParallaxPING (byte sensorNr, boolean units = true) {
  byte pin = sensorNr + PING_PIN_BASE;
  long duration;
  int result;

  lastRoutine = String(F("readParallaxPING"));

  /*
    The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  */
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  /*
    The same pin is used to read the signal from the PING))): a HIGH
    pulse whose duration is the time (in microseconds) from the sending
    of the ping to the reception of its echo off of an object.
  */
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);

  //  Convert the duration into a distance
  if (units) {
    //  Return result in cm
    result = microsecondsToCentimeters(duration);
  } else {
    //  Return result in inches.
    result = microsecondsToInches(duration);
  }
 
  delay(100);
  
  return result;
}

/*
  Read distance in cm from a Sharp GP2Y0A21YK0F IR sensor
*/
float readSharpGP2Y0A21YK0F (byte sensorNr) {
  byte pin = sensorNr + IR_PIN_BASE;
  float reading;
  float distance;

  reading = float(analogRead(pin));
  distance = (6762.0 / (reading - 9)) - 4;

  lastRoutine = String(F("readSharpGP2Y0A21YK0F"));

  return distance;
}

/********************************************************************/
/*  Servo related routines                                          */
/********************************************************************/

/*
    Move a servo by pulse width in ms (500ms - 2500ms)
*/
uint16_t moveServoPw (StandardServo *servo, int servoPosition) {
  uint16_t errorStatus = 0;
  char asciiCR = 13;
  int position = servoPosition + servo->offset;

  lastRoutine = String(F("moveServoPw"));

  servo->error = 0;
  
  if ((servoPosition >= servo->minPulse) && (servoPosition <= servo->maxPulse)) {
//  if ((position >= servo->minPulse) && (position <= servo->maxPulse)) {
    if (servo->maxDegrees == 180) {
      servo->angle += 90;
    }

    //  Move the servo
    servo->servo.writeMicroseconds(position);
  } else {
    console.print(F("(moveServoPw) servoPosition = "));
    console.print(servoPosition);
    console.print(F(", position = "));
    console.print(position);
    console.print(F(", '"));
    console.print(servo->descr);
    console.print(F("', Min Pulse = "));
    console.print(servo->minPulse);
    console.print(F(", Max Pulse = "));
    console.print(servo->maxPulse);
    console.print(F(", Offset = "));
    console.print(servo->offset);
    console.println(F("."));

    errorStatus = 201;
    processError(errorStatus, F("Servo pulse is out of range"));
    servo->error = errorStatus;
  }

  return errorStatus;
}

/*
    Move a servo by degrees (-90 to 90) or (0 - 180)
*/
uint16_t moveServoDegrees (StandardServo *servo, int servoDegrees) {
  uint16_t servoPulse;

  uint16_t errorStatus = 0;
  String errorMsg;

  lastRoutine = String(F("moveServoDegrees"));

  servo->error = 0;
  
  //  Convert degrees to ms for the servo move
  if (servo->maxDegrees == 90) {
    servoPulse = SERVO_CENTER_MS + (servoDegrees * 12);
  } else if (servo->maxDegrees == 180) {
    servoPulse = SERVO_CENTER_MS + ((servoDegrees -90) * 12);
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

/*
  Find the closest and farthest objects in an area scan
*/
DistanceObject findDistanceObjects () {
  uint8_t readingNr;

  DistanceObject distObj = { 0, 0, 0, 0, 0, 0, 0, 0 };

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

  //  Stop, so we can do this scan
  console.println(F("Stopping to scan the area.."));
  romeoV11Stop();

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
      roverStatus = Scanning;

      readingNr = 0;
      positionDeg = startDeg;

      console.println(F("Scanning the area.."));

      while (positionDeg <= stopDeg) {
//      for (positionDeg = startDeg; positionDeg <= stopDeg; positionDeg += incrDeg) {
        errorStatus = moveServoDegrees(pan, positionDeg);

        if (errorStatus != 0) {
          processError(errorStatus, "Could not move the " + pan->descr + " servo");
          break;
        } else {
          //  Delay to let the pan/tilt stabilize after moving it
          delay(1500);

          //  Take a reading from our sensor(s)
          if (MAX_NUMBER_PING > 0) {
            areaScan[readingNr].ping = readParallaxPING(PING_FRONT_CENTER);
          }

          if (MAX_NUMBER_IR > 0) {
            areaScan[readingNr].ir = readSharpGP2Y0A21YK0F(IR_FRONT_CENTER);
          }
/*
          if (HAVE_COLOR_SENSOR) {
            areaScan[readingNr].color = readColorSensor();
          }

          if (HAVE_HEAT_SENSOR) {
            areaScan[readingNr].heat = readHeatSensor();
          }
*/          
          areaScan[readingNr].positionDeg = positionDeg;

          readingNr += 1;
        }

        positionDeg += incrDeg;
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
    console.println(F("The area scan is valid.."));

    //  Set the number of readings taken
    nrAreaReadings = readingNr;

    //  This area scan is valid
    areaScanValid = true;
  }

  console.println(F("Leaving the area scanner.."));
  roverStatus = Stopped;

  return errorStatus;
}

/*
  Turn towards the closest detected object
*/
uint16_t turnToClosestObject (DistanceObject *distObj, StandardServo *pan) {
  uint16_t errorStatus = 0;
  int pingReading;

  if (distObj->closestPosPING < 0) {
    //  Turn to the right
    romeoV11TurnRight(150, 150, ROVER_DEFAULT_MOVE_SPEED);
    delay(2500);
  } else if (distObj->closestPosPING > 0) {
    //  Turn to the left
    romeoV11TurnLeft(150, 150, ROVER_DEFAULT_MOVE_SPEED);
    delay(2500);
  } else {
    //  Set the pan servo to home position (straight ahead)
    errorStatus = moveServoPw(&mainPan, SERVO_MAIN_PAN_HOME);
    delay(250);

    //  Take a reading from the PING distance sensor
    pingReading = readParallaxPING(PING_FRONT_CENTER);

    if (pingReading < PING_MIN_DISTANCE_CM) {
      romeoV11BackUp(125, 125, 2500);

      romeoV11MakeRandomTurn(100, 100, ROVER_DEFAULT_MOVE_TIME_MS);
    }
  }

  roverStatus = Stopped;

  return errorStatus;
}

/*
  Turn towards the farthest detected object
*/
uint16_t turnToFarthestObject (DistanceObject *distObj, StandardServo *pan) {
  uint16_t errorStatus = 0;
  int pingReading;

  if (distObj->farthestPosPING < 0) {
    //  Turn to the right
    romeoV11TurnRight(150, 150);
    delay(2500);
  } else if (distObj->farthestPosPING >= 0) {
    //  Turn to the left
    romeoV11TurnLeft(150, 150, ROVER_DEFAULT_MOVE_SPEED);
    delay(2500);
  } else {
    //  Set the pan servo to home position (straight ahead)
    errorStatus = moveServoPw(&mainPan, SERVO_MAIN_PAN_HOME);
    delay(250);

    //  Take a reading from the PING distance sensor
    pingReading = readParallaxPING(PING_FRONT_CENTER);

    if (pingReading < PING_MIN_DISTANCE_CM) {
      romeoV11BackUp(125, 125, 2500);

      romeoV11MakeRandomTurn(100, 100, ROVER_DEFAULT_MOVE_SPEED);
    }
  }

  roverStatus = Stopped;

  return errorStatus;
}

/********************************************************************/
/*  Miscellaneous routines                                          */
/********************************************************************/

/*
  Show announcement message on the desired port.
*/
void announce (BMSerial *port) {
  port->println();
  port->print("A.R.T.I.E., Master Control Program (MCP), version ");
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

  roverStatus = Error;
}

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
  Convert a temperature in Celsius to Fahrenheit
*/
float toFahrenheit (float celsius) {
  lastRoutine = String(F("toFahrenheit"));

  return (celsius * 1.8) + 32;
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

/********************************************************************/
/*  7 segment and 8x8 matrix display routines                       */
/********************************************************************/

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

/********************************************************************/
/*  Serial port handling routines                                   */
/********************************************************************/

/*
  Process serial port commands
*/
void processRemoteCommand (byte leftSpd, byte rightSpd, uint16_t durationMS = 0) {
  char command = console.read();
  bool moving = false;

  if (command != -1) {
    if (manualControl && (command != 'm') && (command != 'M')) {
      switch(command) {
        case 'w':
        case 'W':
          romeoV11Forward(leftSpd, rightSpd, durationMS);
          moving = true;
          break;

        case 's':
        case 'S':
          romeoV11Reverse(leftSpd, rightSpd, durationMS);
          moving = true;
          break;

        case 'a':
        case 'A':
          romeoV11TurnLeft(leftSpd, rightSpd, durationMS);
          moving = true;
          break;      

        case 'd':
        case 'D':
          romeoV11TurnRight(leftSpd, rightSpd, durationMS);
          moving = true;
          break;

        case 'z':
        case 'Z':
          console.println(F("Hello"));
          break;

        case 'x':
        case 'X':
          romeoV11Stop();
          mainPan.servo.writeMicroseconds(SERVO_MAIN_PAN_HOME + SERVO_MAIN_PAN_OFFSET);
          break;

        case 'h':
        case 'H':
          mainPan.servo.writeMicroseconds(SERVO_MAIN_PAN_HOME + SERVO_MAIN_PAN_OFFSET);
          break;

        default:
          console.println(F("Invalid command received!"));
          break;
      }
    } else {
      //  Toggle Manual Control
      manualControl = !manualControl;

      if (manualControl) {
        console.println(F("Manual Control is disabled"));
      } else {
        console.println(F("Manual Control is enabled"));
      }
    }
  }
}

/********************************************************************/
/*  initialization routines                                         */
/********************************************************************/

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

  if (HAVE_COLOR_SENSOR) {
    //  Initialize the TCS3725 RGB Color sensor (Adafruit)
    if (! rgb.begin()) {
      errorStatus = 604;
      processError(errorStatus, F("There was a problem initializing the TCS34725 RGB color sensor .. check your wiring or I2C Address!"));
    } else {
      //  Initialize and turn off the TCS34725 RGB Color sensor's LED
      pinMode(COLOR_SENSOR_LED, OUTPUT);
      digitalWrite(COLOR_SENSOR_LED, LOW);
      delay(250);
      digitalWrite(COLOR_SENSOR_LED, HIGH);
      delay(250);
      digitalWrite(COLOR_SENSOR_LED, LOW);
    }
  }

  if (HAVE_HEAT_SENSOR) {
    //  Initialize the TMP006 heat sensor
    if (! heat.begin()) {
      errorStatus = 605;
      processError(errorStatus, F("There was a problem initializing the TMP006 heat sensor .. check your wiring or I2C Address!"));
    }
  }

  if (HAVE_DS1307_RTC) {
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
  console.println(F("Initializing servos.."));

  lastRoutine = String(F("initServos"));

  irEyeTilt.pin = SERVO_IR_EYE_TILT_PIN;
  irEyeTilt.descr = String(SERVO_IR_EYE_TILT_NAME);
  irEyeTilt.offset = SERVO_IR_EYE_TILT_OFFSET;
  irEyeTilt.homePos = SERVO_IR_EYE_TILT_HOME;
  irEyeTilt.msPulse = 0;
  irEyeTilt.angle = 0;
  irEyeTilt.minPulse = SERVO_IR_EYE_TILT_UP_MIN;
  irEyeTilt.maxPulse = SERVO_IR_EYE_TILT_DOWN_MAX;
  irEyeTilt.maxDegrees = SERVO_MAX_DEGREES;
  irEyeTilt.error = 0;

  //  Set the servo to home position
  pinMode(SERVO_IR_EYE_TILT_PIN, OUTPUT);
  irEyeTilt.servo.attach(SERVO_IR_EYE_TILT_PIN);
  moveServoPw(&irEyeTilt, SERVO_IR_EYE_TILT_HOME);

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
void setup (void) {
  int pin;
  uint16_t errorStatus = 0;
  uint8_t loopCount = 0;

  //  Seed the random number generator
  randomSeed(analogRead(0));

  //  Set motor control pins to OUTPUTs
  for (pin = 4; pin <= 7; pin++) {
    pinMode(pin, OUTPUT); 
  }

  lastRoutine = String(F("SETUP"));

  //  Initialize the console port
  console.begin(9600);
  delay(1000);
  announce(&console);

  //  Delay for a few seconds, before starting initialization
  console.println();
  wait(STARTUP_DELAY_SECONDS, "initialization");

  console.println(F("Initializing Digital Pins.."));

  //  Initialize the heartbeat LED pin as an output.
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, LOW);

  if (HAVE_7SEGMENT_DISPLAYS) {
    console.println(F("Initializing displays.."));

    //  Initialize the displays
    initDisplays(MAX_NUMBER_7SEG_DISPLAYS);

    //  Test the displays
    testDisplays(MAX_NUMBER_7SEG_DISPLAYS);
  }

  //  Initialize all sensors
  errorStatus = initSensors();

  //  Initialize all servos
  initServos();

  //  Do an initial scan of the immediate area
  console.println(F("Doing initial area scan.."));
  scanArea(&mainPan, ROVER_DEFAULT_SCAN_START_DEG, ROVER_DEFAULT_SCAN_END_DEG, ROVER_DEFAULT_SCAN_INCR_DEG);
  console.println(F("Leaving setup.."));

  // Init the interrupt mode for the digital pin 2
  attachInterrupt(WHEEL_ENCODER_LEFT, LeftWheelSpeed, CHANGE);
  // Init the interrupt mode for the digital pin 3
  attachInterrupt(WHEEL_ENCODER_RIGHT, RightWheelSpeed, CHANGE);

  //  Initialize encoder data values
  lastSpeed[WHEEL_ENCODER_LEFT] = -1;
  lastSpeed[WHEEL_ENCODER_RIGHT] = -1;

  // Clear the encoder data buffer
  encoder[WHEEL_ENCODER_LEFT] = 0;
  encoder[WHEEL_ENCODER_RIGHT] = 0;

  roverStatus = Idle;
}

/********************************************************************/
/*  Runs forever                                                    */
/********************************************************************/
void loop (void) {
  uint16_t errorStatus = 0;

  // Print manager timer
  static unsigned long timer = 0;

  //  The current date and time from the DS1307 real time clock
//  DateTime now = clock.now();

  uint16_t displayInt;

  //  Display related variables
  boolean amTime;
  uint8_t displayNr = 0, count = 0, readingNr = 0;
//  uint8_t currentHour = now.hour(), nrDisplays = 0;

  uint8_t analogPin = 0;
  uint8_t digitalPin = 0;

  DistanceObject distObject;
  ColorSensor colorData;
  HeatSensor heatData;

  lastRoutine = String(F("LOOP"));

  // Pulse the heartbeat LED
  pulseDigital(HEARTBEAT_LED, 500);

//  currentMinute = now.minute();
  /*
    This is code that only runs ONE time, to initialize
      special cases.
  */
  if (startingUp) {
    console.println(F("Processing startup instuctions.."));

    lastMinute = currentMinute;
  }

  //  Check for a manual control command from the serial link
  console.println(F("Checking for remote commands.."));

  if (console.available()) {
    processRemoteCommand(ROVER_DEFAULT_MOVE_SPEED, ROVER_DEFAULT_MOVE_SPEED);
  } else {
    //  Check readings from all the Parallax PING ultrasonic range sensors, if any, and store them
    if (MAX_NUMBER_PING > 0) {
      console.println(F("Reading PING distance sensors.."));

      //  Set the main pan servo to home position before reading it.
      errorStatus = moveServoPw(&mainPan, SERVO_MAIN_PAN_HOME);
      delay(SERVO_MAIN_PAN_STABLE_MS);

      //  Read the ultrasonic sensor(s)
      for (digitalPin = 0; digitalPin < MAX_NUMBER_PING; digitalPin++) { 
        ping[digitalPin] = readParallaxPING(digitalPin);
      }

      displayPING();

      console.println(F("Checking PING distance sensor readings.."));

      //  Let's see if we're too close to an object.. If so, stop and scan the area
      if (ping[PING_FRONT_CENTER] <= PING_MIN_DISTANCE_CM) {
        console.println(F("I'm too close to something, so I'm backing away.."));

        romeoV11BackUp(125, 125);

        romeoV11MakeRandomTurn(ROVER_DEFAULT_MOVE_SPEED, ROVER_DEFAULT_MOVE_SPEED, ROVER_TURN_RANDOM_TIME_MS);

        if (errorStatus != 0) {
          processError(errorStatus, F("Unable to scan the area"));
        } else {
          console.println(F("I'm looking for the closest and farthest objects.."));
          //  Find the closest and farthest objects
          distObject = findDistanceObjects();

          errorStatus = turnToFarthestObject(&distObject, &mainPan);

          if (errorStatus != 0) {
            processError(errorStatus, F("Could not complete a turn to the farthest object"));

            romeoV11Stop();

            if (errorStatus != 0) {
              runAwayRobot(errorStatus);
            }
          }
        }
      }
    }

    if (roverStatus != Error) {
      //  Let's start moving forward!
      console.println(F("Moving forward.."));
      romeoV11Forward(ROVER_DEFAULT_MOVE_SPEED, ROVER_DEFAULT_MOVE_SPEED);
    } else {
      processError(999, F("Unrecoverable ERROR"));
      romeoV11Stop();
      callForHelp(5, 5, 10);
    }

    if (WHEEL_ENCODER_SUPPORT) {
      console.println(F("Checking the encoders.."));

      //  Check to make sure the motors are still spinning
      if (wheelsAreMoving()) {
        console.println(F("Wheels are moving.."));

        //  Update encoder data
        lastSpeed[WHEEL_ENCODER_LEFT] = encoder[WHEEL_ENCODER_LEFT];
        lastSpeed[WHEEL_ENCODER_RIGHT] = encoder[WHEEL_ENCODER_RIGHT];
      } else if ((roverStatus != Idle) && (roverStatus != Scanning) && (roverStatus != Error)) {
        console.println(F("Wheels are NOT moving.."));

        //  We are apparently stuck
        displayEncoders();

        if (roverStatus == Forward) {
          //  Backup a bit
          romeoV11BackUp(125, 125);
        } else if (roverStatus == Reverse) {
          //  Move forward a bit
          romeoV11Forward(125, 125);
        }

        romeoV11MakeRandomTurn(ROVER_DEFAULT_MOVE_SPEED, ROVER_DEFAULT_MOVE_SPEED, ROVER_TURN_RANDOM_TIME_MS);

        errorStatus = scanArea(&mainPan, ROVER_DEFAULT_SCAN_START_DEG, ROVER_DEFAULT_SCAN_END_DEG, ROVER_DEFAULT_SCAN_INCR_DEG);

        turnToFarthestObject(&distObject, &mainPan);

        delay(ROVER_DEFAULT_DELAY_MS);
      } else if (roverStatus == Error) {
        
        callForHelp(5, 5, 10);
      }
    }
  }

  if (startingUp) {
    startingUp = false;
  }
}
