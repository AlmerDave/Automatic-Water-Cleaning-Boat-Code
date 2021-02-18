#include <SoftwareSerial.h>
SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

int ButtonState = 0;
int left = 2;
int right = 3;
int returnb = 4;
int stopb = 5;
int goLed = 8;
int stopLed = 9;

 
int buttonOld=0;
int LEDState=0;

int leftState = 0, rightState = 0, returnState = 0, stopState = 0;

void setup() 
{
  pinMode(left,INPUT);
  pinMode(right,INPUT);
  pinMode(returnb,INPUT);
  pinMode(stopb,INPUT);
  pinMode(goLed,OUTPUT);
  pinMode(stopLed,OUTPUT);

  digitalWrite(goLed,LOW);
  digitalWrite(stopLed,LOW);
  
  Serial.begin(9600); // Serial port to computer
  HC12.begin(9600);  // Serial port to HC12
}
void loop() 
{
 ControllerLogic();
}
void ControllerLogic() 
{
  leftState = digitalRead(left);
  rightState = digitalRead(right);
  returnState = digitalRead(returnb);
  stopState = digitalRead(stopb);
 
  if(leftState == 0 && ButtonState == 0)
  {
    ButtonState = 1; 
    Serial.println("1");
    HC12.write("1");
    
 
    digitalWrite(goLed, HIGH); 
    digitalWrite(stopLed, LOW);
    Serial.println(ButtonState);
  }
  if(rightState == 0 && ButtonState == 0)
  {
    ButtonState = 1; 
    Serial.println("2");
    HC12.write("2");
 
    digitalWrite(goLed, HIGH); 
    digitalWrite(stopLed, LOW);
    Serial.println(ButtonState);
  }
  if(returnState == 0)
  { 
    if(buttonOld==1 && returnState==0){
      if (LEDState==0){
        ButtonState = 0;
        Serial.println("A");
        HC12.write("A");
        
        digitalWrite(goLed, LOW);
        for(int i = 0; i < 5; i++)
        {
          digitalWrite(stopLed, HIGH);
          delay(60);
          digitalWrite(stopLed, LOW);
          delay(60);
        }
        digitalWrite(stopLed, HIGH);
        delay(1000);
        digitalWrite(stopLed, LOW);
        
        LEDState=1;
      }
      else{
        
        ButtonState = 1;
        Serial.println("B");
        HC12.write("B");
 
        digitalWrite(goLed, LOW);
        for(int i = 0; i < 20; i++)
        {
          digitalWrite(stopLed, HIGH);
          delay(80);
          digitalWrite(stopLed, LOW);
          delay(80);
        }
        LEDState=0;
      }
    } 
 
    
    
    
  }
  if(stopState == 0)
  {
    ButtonState = 0;
  
    
    Serial.println("4");
    HC12.write("4");
    
    digitalWrite(goLed, LOW); 
    digitalWrite(stopLed, HIGH);
    delay(2000);
    digitalWrite(stopLed, LOW);
    Serial.println(ButtonState);
  } 
  buttonOld=returnState;
  
}
 
