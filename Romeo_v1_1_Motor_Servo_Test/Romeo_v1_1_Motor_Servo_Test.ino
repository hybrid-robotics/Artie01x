/*
  Program:      4WD Rover (DFRobot Baron Rover) Motor and Servo Test sketch
  Date:         29-Jun-2014
  Version:      0.1.1 ALPHA

  Platform:     DFRobot Romeo v1.1 Microcntroller (Arduino Uno compatible)

  Purpose:      To have FUN with a little 4WD rover - can control the Rover over a serial link.

                                                  Change Log
                -------------------------------------------------------------------------------
                v0.1.0 ALPHA 27-Jun-2014:
                Initial build from example code on the DFRobot Wiki
                -------------------------------------------------------------------------------
                v0.1.1 ALPHA 29-Jun-2014:
                Motors work when controlled over the serial port.
                Adding servo test code.
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
#include <BMSerial.h>

#include "Romeo_v1_1_Motor_Servo_Test.h"

/*
  Standard PWM DC control
*/
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 7;    //M1 Direction Control

//  Our pan servo
Servo mainPan;

/*
  Code starts here
*/



void stop (byte durationMS = 0) {
  digitalWrite(E1, LOW);  
  digitalWrite(E2, LOW);     

  if (durationMS > 0) {
    delay(durationMS);
  }
}

void forward (char a, char b, byte durationMS = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, HIGH);   
  analogWrite (E2, b);   
  digitalWrite(M2, HIGH);

  if (durationMS > 0) {
    delay(durationMS);
    stop();
  }
} 

void reverse (char a, char b, byte durationMS = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, LOW);  
  analogWrite (E2, b);   
  digitalWrite(M2, LOW);

  if (durationMS > 0) {
    delay(durationMS);
    stop();
  }
}

void turnLeft (char a, char b, byte durationMS = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, LOW);   
  analogWrite (E2, b);   
  digitalWrite(M2, HIGH);

  if (durationMS > 0) {
    delay(durationMS);
    stop();
  }
}

void turnRight (char a, char b, byte durationMS = 0) {
  analogWrite (E1, a);
  digitalWrite(M1, HIGH);   
  analogWrite (E2, b);   
  digitalWrite(M2, LOW);

  if (durationMS > 0) {
    delay(durationMS);
    stop();
  }
}

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
  Parallax Ping))) Sensor 

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
  Function to read a value from a GP2D12 infrared distance sensor and return a
    distance value in centimeters.

  This sensor should be used with a refresh rate of 36ms or greater.

  Javier Valencia 2008

  float readGP2D12(byte pin)

  It can return -1 if something has gone wrong.

  TODO: Make several readings over a time period, and average them
    for the final reading.
*/
float readSharpGP2D12 (byte sensorNr) {
  byte pin = sensorNr + IR_PIN_BASE;
  int tmp;

  lastRoutine = String(F("readSharpGP2D12"));

  tmp = analogRead(pin);

  if (tmp < 3) {
    return -1.0;                // Invalid value
  } else {
    return (6787.0 /((float)tmp - 3.0)) - 4.0;  // Distance in cm
  }
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

void setup (void) {
  int i;

  //  Set all the motor control pins to OUTPUT
  for(i = M1; i <= M2; i++) {
    pinMode(i, OUTPUT); 
  }

  //  Setup the pan servo and put it at home position
  Serial.println(F("Initializing servos.."));
  mainPan.attach(SERVO_MAIN_PAN_PIN);
  mainPan.writeMicroseconds(SERVO_MAIN_PAN_HOME + SERVO_MAIN_PAN_OFFSET);

  //  Initialize serial port(s)
  Serial.println(F("Initializing serial port(s).."));
  Serial.begin(9600);

  //  Announce ourselves
  Serial.println(F("Announcing our presense.."));
  Serial.println(F("Run keyboard control.."));
}

void loop(void) {
  char val;

  if (Serial.available()) {
    Serial.println(F("Processing serial command.."));

    val = Serial.read();

    if (val != -1) {
      switch(val) {
        case 'w':
        case 'W':
          forward(100, 100);
          break;

        case 's':
        case 'S':
          reverse(100, 100);
          break;

        case 'a':
        case 'A':
          turnLeft(100, 100);
          break;      

        case 'd':
        case 'D':
          turnRight(100, 100);
          break;

        case 'z':
        case 'Z':
          Serial.println("Hello");
          break;

        case 'x':
        case 'X':
          stop();
          mainPan.writeMicroseconds(SERVO_MAIN_PAN_HOME + SERVO_MAIN_PAN_OFFSET);
          break;

        case 'q':
        case 'Q':
          mainPan.write(45);
          break;

        case 'r':
        case 'R':
          mainPan.write(135);
          break;

        default:
          Serial.println(F("Invalid command received!"));
          break;
      }
    }
  } else {
    Serial.println(F("No commands to process.."));

    forward(100, 100);
    delay(2000);

    turnRight(100, 100);
    delay(2000);

    forward(100, 100);
    delay(2000);

    turnRight(100, 100);
    delay(2000);

    forward(100, 100);
    delay(2000);

    stop();
  }
}
