//Standard DLL Speed control
 
int E1 = 4;     //M1 Speed Control
int E2 = 7;     //M2 Speed Control
int M1 = 5;    //M1 Direction Control
int M2 = 6;    //M1 Direction Control
 
///For previous Romeo, please use these pins.
//int E1 = 6;     //M1 Speed Control
//int E2 = 9;     //M2 Speed Control
//int M1 = 7;    //M1 Direction Control
//int M2 = 8;    //M1 Direction Control
 
//When m1p/m2p is 127, it stops the motor
//when m1p/m2p is 255, it gives the maximum speed for one direction
//When m1p/m2p is 0, it gives the maximum speed for reverse direction
 
void DriveMotorP(byte m1p, byte m2p)//Drive Motor Power Mode
{
 
  digitalWrite(E1, HIGH);
  analogWrite(M1, (m1p));
 
  digitalWrite(E2, HIGH);
  analogWrite(M2, (m2p));
 
}
 
void setup(void)
{
  int i;
  for(i=6;i<=9;i++)
    pinMode(i, OUTPUT); 
  Serial.begin(19200);      //Set Baud Rate
}
void loop(void)
{
  if(Serial.available()){
    char val = Serial.read();
    if(val!=-1)
    {
      switch(val)
      {
      case 'w'://Move Forward
        DriveMotorP(0xff,0xff); // Max speed
        break;
      case 'x'://Move Backward
        DriveMotorP(0x00,0x00);
        ; // Max speed
        break;
      case 's'://Stop
        DriveMotorP(0x7f,0x7f);
        break;      
 
      }
    }
  }
}