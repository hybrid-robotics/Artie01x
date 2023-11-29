/*
  Program:      4WD Rover (DFRobot Baron Rover) Motor and Servo Test sketch
  Date:         29-Jun-2014
  Version:      0.1.0 ALPHA

  Platform:     DFRobot Romeo v1.1 Microcntroller (Arduino Uno compatible)

  Purpose:      To have FUN with a little 4WD rover - can control the Rover over a serial link.

                                                  Change Log
                -------------------------------------------------------------------------------
                v0.1.0 ALPHA 29-Jun-2014:
                Initial build of the code
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
  byte pin = sensorNr;
  long duration;
  int result;

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
  byte pin = sensorNr;
  int reading;
  float distance;

  reading = analogRead(pin);
  distance = (6762.0 / (reading - 9)) - 4;

  Serial.print(F("The reading is "));
  Serial.print(reading);
  Serial.println(F("."));

  return distance;
}

void setup (void) {
	Serial.begin(9600);
}

void loop (void) {
	int distanceReading;

	distanceReading = readParallaxPING(2);

	Serial.print(F("Distance = "));
	Serial.print(distanceReading);
	Serial.println(F(" cm."));
	Serial.println();

	delay(2000);
}
