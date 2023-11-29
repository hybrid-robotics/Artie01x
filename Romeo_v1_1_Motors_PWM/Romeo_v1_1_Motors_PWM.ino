/*
  Program:      4WD Rover (DFRobot Baron Rover) Master Control Program (MCP)
  Date:         28-Jun-2014
  Version:      0.1.1 ALPHA

  Platform:     DFRobot Romeo v1.1 Microcntroller (Arduino Uno compatible)

  Purpose:      To have FUN with a little 4WD rover - controls the Rover over the serial link.

                                                  Change Log
                -------------------------------------------------------------------------------
                v0.1.0 ALPHA 27-Jun-2014:
                Initial build from example code on the DFRobot Wiki
                -------------------------------------------------------------------------------

  Dependencies: None (Yet)

  Comments:     Credit is given, where applicable, for code I did not originate.

  Copyright (c) 2014 Dale A. Weber <hybotics.pdx@gmail.com, @hybotics on App.Net and Twitter>
*/

//  Standard PWM DC control
int E1 = 5;                           //  M1 Speed Control
int M1 = 4;                           //  M1 Direction Control

int E2 = 6;                           //  M2 Speed Control
int M2 = 7;                           //  M2 Direction Control
 
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

void setup (void) {
  int i;

  for(i = 4; i <= 7; i++) {
    pinMode(i, OUTPUT); 
  }

  //  Set Baud Rate
  Serial.begin(9600);

  delay(1000);

  Serial.println("Run keyboard control");
}

void loop (void) {
  char val;

  if (Serial.available()) {
    Serial.println(F("Processing serial command.."));

    val = Serial.read();

    if (val != -1) {
      switch(val) {
        case 'w':                     //  Move Forward
        case 'W':
          Serial.println(F("(Serial) Moving forward.."));
          forward(50, 50);          //  Move forward in max speed
          break;

        case 's':                     //  Move Backward
        case 'S':
          Serial.println(F("(Serial) Reversing.."));
          reverse(50, 50);          //  Move back in max speed
          break;

        case 'a':                     //  Turn Left
        case 'A':
          Serial.println(F("(Serial) Turning LEFT.."));
          turnLeft(50, 50);
          break;      

        case 'd':                     //  Turn Right
        case 'D':
          Serial.println(F("(Serial) Turning RIGHT.."));
          turnRight(50, 50);
          break;

        case 'z':
        case 'Z':
          Serial.println("(Serial) Hello");
          break;

        case 'x':
        case 'X':
          Serial.println(F("(Serial) Stopping.."));
          stop();
          break;

        default:
          Serial.println(F("(Serial) Invalid command!"));
          break;
      }
    } else {
      Serial.println(F("Stopping.."));
      stop();
    }
  } else {
    delay(2000);
    Serial.println(F("Moving forward.."));
    forward(50, 50, 50);
    Serial.println(F("Turning LEFT.."));
    turnLeft(50, 50, 50);
    Serial.println(F("Moving forward.."));
    forward(50, 50, 50);
    Serial.println(F("Turning RIGHT.."));
    turnRight(50, 50, 50);
    Serial.println(F("Reversing.."));
    reverse(50, 50, 50);
    Serial.println(F("Turning LEFT.."));
    turnLeft(50, 50, 50);
    Serial.println(F("Moving forward.."));
    forward(50, 50, 50);
    Serial.println(F("Stopping.."));
    stop();
  }
}
