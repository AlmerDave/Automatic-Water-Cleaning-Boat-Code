 #include <Servo.h>

Servo bldc1, bldc2, servo, servo_pbank;
long myTime, deadline ;
//------------------------------Command Variables--------------
String readBuffer = "";
byte incomingByte;
int eRight = 0, eLeft = 0, eForward = 0;
//------------------------------ Bldc Variables----------------------------
int bldcSpd1, bldcSpd2; // bldcspd2 yung pinakamtaas na value

//---------------------------------Servo Variables-------------------
int servoPin = 6;
int pbankPin = 5;
int refAngle = 90;
int servo_angle = 90;
int pos = 0;

void setup(){
  Serial.begin(9600);
 // Main.begin(9600); // Software Serial for the main arduino

  deadline = 0;
   
  bldc1.attach(9); // bldc motor 1
  delay(1);
  bldc1.write(18);

  bldc2.attach(10); // bldc motor 2
  delay(1);
  bldc2.write(15);

  servo_pbank.attach(pbankPin);
   
 
  
 
  servo.attach(servoPin);
  delay(1);
  servo.write(180); 
  delay(1000); 
  servo.write(0);
  delay(1000);
  servo.write(70); // 70 center/
  delay(1000);
 
}
// REMINDER.... SA CONNECTION ILAGAY YUNG 5V NG ESC SA SERVO PARA MAG SUPPLY TOH NG POWER PERO SAME SILA NG GROUND KASAMA NG ARDUINO
void loop(){
 
  myTime = millis(); 
 
  if(myTime > deadline)
  {
    servo_pbank.write(145);
    delay(350);
    servo_pbank.write(90);
    delay(650);
    servo_pbank.write(145);
    delay(350);
    servo_pbank.write(90);
    deadline = deadline + 31000;
  }
 
 
 commandLogic();
 
}

void commandLogic()   // kunin muna kung ano command galing sa main body
{
  Serial.flush();
  
  if (Serial.available())
  {
 
    readBuffer = ""; 
      while (Serial.available())
    { 
      incomingByte = Serial.read() ;
 
      readBuffer  = char(incomingByte); 
  
    }  
  }
  

  // r - right , l - left, f - forward, s - stop

  if (readBuffer == "f" && eForward == 0)
  {
    eForward = 1;
    eRight = 0;
    eLeft = 0;
    
    Serial.println("f");
    Forward();
    
  }
  else if(readBuffer == "r" && eRight == 0)
  {
    eForward = 0;
    eRight = 1;
    eLeft = 0;
    
    Serial.println("r");
    RightTurn();
  }
  else if (readBuffer == "l" && eLeft == 0)
  {
    eForward = 0;
    eRight = 0;
    eLeft = 1;
    
    Serial.println("l");
    LeftTurn();
  }
  else if (readBuffer == "s" )
  {
    eForward = 0;
    eRight = 0;
    eLeft = 0;
    Serial.println("s");
    Stop();
  }
  readBuffer = "";
}

void Forward()
{
  bldcSpd1 = 25; 
  bldcSpd2 = 65;// run  2 bldc motors at 25 speed
  
  servo.write(70);
  
  bldc1.write(bldcSpd1);
  bldc2.write(bldcSpd2);

 
}
void RightTurn()
{
  bldcSpd1 = 25; 
  bldcSpd2 = 65;

  servo.write(180); //move to the left but the boat will make right turn
 
  bldc1.write(bldcSpd1);
  bldc2.write(bldcSpd2);
 
}

void LeftTurn()
{
  bldcSpd1 = 25; 
  bldcSpd2 = 65;

  servo.write(0); //move to the right but the boat will make left turn
 

  bldc1.write(bldcSpd1);
  bldc2.write(bldcSpd2);

  
}

void Stop()
{
  bldcSpd1 = 17; // default stop of motor

  bldc1.write(bldcSpd1);
  bldc2.write(bldcSpd1);
 
  servo.write(70);

}
 
 
