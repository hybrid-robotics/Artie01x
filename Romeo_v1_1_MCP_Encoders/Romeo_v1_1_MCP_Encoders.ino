// #
// # Editor     : Lauren from DFRobot
// # Date       : 17.01.2012
 
// # Product name: Wheel Encoders for DFRobot 3PA and 4WD Rovers
// # Product SKU : SEN0038
 
// # Description:
// # The sketch for using the encoder on the DFRobot Mobile platform
 
// # Connection:
// #        left wheel encoder  -> Digital pin 2
// #        right wheel encoder -> Digital pin 3
// #
 
 
#define LEFT 0
#define RIGHT 1
 
long encoder[2] = {
  0,0};
int lastSpeed[2] = {
  0,0}; 
 
 
void LwheelSpeed() {
  coder[LEFT] ++;  //count the left wheel encoder interrupts
}
 
void RwheelSpeed(){
  coder[RIGHT] ++; //count the right wheel encoder interrupts
}

void setup(){
   
  Serial.begin(9600);                            //init the Serial port to print the data
  attachInterrupt(LEFT, LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
  attachInterrupt(RIGHT, RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3
   
}
 
void loop() {
  static unsigned long timer = 0;                //print manager timer
   
  if(millis() - timer > 100){                  
    Serial.print("Encoder values: ");
    Serial.print(encoder[LEFT]);
    Serial.print("Left Wheel = ");
    Serial.print(encoder[RIGHT]);
    Serial.println("Right Wheel = ");
     
    lastSpeed[LEFT] = encoder[LEFT];   //record the latest speed value
    lastSpeed[RIGHT] = encoder[RIGHT];

    encoder[LEFT] = 0;                 //clear the data buffer
    encoder[RIGHT] = 0;

    timer = millis();
  }
}
