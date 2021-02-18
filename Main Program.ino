#include <Servo.h>
#include <SoftwareSerial.h>
#include<Wire.h>
#include "compass.h"
#include <TinyGPS++.h>

//--------------------Software Serials---------------------------------
int rxHC = 7;   //RX TX PIN FOR HC-12
int txHC = 6;


int RXSoft = 2;  //RX TX PIN FOR 3rd arduino
int TXSoft = 3;
 
SoftwareSerial HC12(txHC, rxHC);
SoftwareSerial Third(RXSoft,TXSoft);  // software serial for the 3rd arduino responsible for motors
 
int redPin = 4;                            

//-------------------------------- Servo for ultrasonic----------------
Servo servo;

int angle = 90;
int change = 90;
int pos = 0;  
//---------------------Ultrasonic--------------------------------------- 
int ultraMain_trig = 12;
int servoPin = 9;

int ultraBin_echo = 11;
int ultraFront_echo = 10;
int ultraBack_echo = 8;

int distance_Front, distance_Back, distance_Bin, currentState; // variables for unique Component
int modeState = 0, turnState = 0 ; 
//------------------------HC-12-------------------------

byte incomingByte;
String inputHC = "";

//-----------------------Compass-----------------------------

// compass SDA = A4 SCL = A5 (baliktad sa component mismo)
int compassHeading, newCompassHeading, desired_heading;


int Heading_A;                                                     // variable to store compass heading
int Heading_B;                                                     // variable to store compass heading in Opposite direction
int pass = 0;                                                      // variable to store which pass the robot is on

//--------------------------GPS--------------------------------
TinyGPSPlus gps;

unsigned long Distance_To_Home;                                    // variable for storing the distance to destination
double homeLAT, homeLON;

int wpCount = 0;
int Number_of_SATS  ;
int GPS_Course; 
bool waypoint =  false ; 

void Unique_setup()
{
  //Initialize the pins 
  pinMode(ultraMain_trig, OUTPUT);
  pinMode(servoPin, OUTPUT);
 
  
  pinMode(ultraBin_echo, INPUT);
  pinMode(ultraFront_echo, INPUT);
  pinMode(ultraBack_echo, INPUT);

  delay(2000);
  servo.attach(servoPin);
  delay(1);
  leftServo();
  rightServo();
  centerServo(); 
  
}

void setup() 
{
 Unique_setup();
 pinMode(LED_BUILTIN, OUTPUT);
 pinMode(redPin, OUTPUT);

  //compass
  Wire.begin();
  compass_x_offset = 158.95; //158.95
  compass_y_offset = 1108.99; //1108.99
  compass_z_offset = 128.20;//128.20
  compass_x_gainError = 1.12;
  compass_y_gainError = 1.13;
  compass_z_gainError = 1.03;

  compass_init(2);

 
  Serial.begin(9600);
  HC12.begin(9600);
  Third.begin(9600); 

  Serial.println("Searching for Satellites ");
  digitalWrite(redPin, LOW);   
 
 
}

void loop() 
{  
 
      //searchSatellites();  // search satelites first before going to the loop
  
    controllerLogic();
 
  getGPS() ;  // bawal toh gamitin
   getCompass() ;
         //getCompassHeading();
        //getCompassDirection();

    //Runnning_mode();
 
 
 
 
}

void returnHome()
{
  while(true)
  {
    digitalWrite(redPin, LOW);
    controllerLogic();
    if(inputHC == "4"){break;}
    getCompassHeading();
    getGPS();
     
    if (millis() > 5000 && gps.charsProcessed() < 10)                // If no Data from GPS within 5 seconds then send error
      Serial.println(F("No GPS data: check wiring"));

    Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),homeLAT, homeLON);
    GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),homeLAT,homeLON);
    Serial.print ("gss");
    Serial.println(GPS_Course);


    if(Distance_To_Home <= 5)
    {
      Third.write("s");

      Serial.print("Distance to Home: ");
      Serial.println(Distance_To_Home);
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("You have arrvied in your destination");
      break;
      
    }

    if(abs(GPS_Course - newCompassHeading) <= 15)
    {
      
      Serial.println("Forward");
      Third.write("f");
      Serial.print("Distance to Home: ");
      Serial.println(Distance_To_Home);
      digitalWrite(redPin, HIGH);
      delay(200);
      digitalWrite(redPin, LOW); 
      
    }
    else
    {
      int x = (GPS_Course - 360);                           // x = the GPS desired heading - 360
      int y = (newCompassHeading - (x));                      // y = the Compass heading - x
      int z = (y - 360);                                    // z = y - 360
 
            
      if ((z <= 180) && (z >= 0))                           // if z is less than 180 and not a negative value then turn left otherwise turn right
      { 
        Serial.print("Distance to Home: ");
        Serial.println(Distance_To_Home);
        Serial.println("left");
        Third.print("l");
   
        delay(200)  ;  // need delay because of transfer of data
      }
      else 
      { 
        Serial.print("Distance to Home: ");
      Serial.println(Distance_To_Home);
        Serial.println("right"); 
        Third.print("r");
        delay(200)  ;  // need delay because of transfer of data
      }           
    }
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);    

  }
}

void setHome()
{
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(redPin, HIGH);
  
  for (int i = 0 ; i <= 6; i++)
  {
    getGPS();
 
      homeLAT = gps.location.lat();
      homeLON = gps.location.lng();
 
  }

  for (int j = 0 ; j <=5; j++)
  {
    getCompassHeading();
  }

  desired_heading = newCompassHeading;
  Heading_A = newCompassHeading;
  Heading_B = newCompassHeading + 180;

  if(Heading_B >= 360)
  {
    Heading_B = Heading_B - 360;
  }

  Serial.print(homeLAT,6);
  Serial.print(" ");
  Serial.println(homeLON,6);

  Serial.print("desired heading");
  Serial.println(desired_heading);
  Serial.print("compass heading");
  Serial.println(newCompassHeading);

  digitalWrite(redPin, LOW);
 
}


void getCompassDirection()
{
  getCompassHeading();
  
  if (newCompassHeading >= 338 && newCompassHeading <=360 || newCompassHeading >= 0 && newCompassHeading <= 22)
  {
    Serial.println("N");
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(redPin, LOW);
  }
  else if (newCompassHeading >= 23 && newCompassHeading <= 70)
    Serial.println("NE");
  else if (newCompassHeading >= 71 && newCompassHeading <= 112)
  {
    Serial.println("E");
    digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(redPin, HIGH);
  }
  else if (newCompassHeading >= 113 && newCompassHeading <= 157)
    Serial.println("SE");
  else if (newCompassHeading >= 158 && newCompassHeading <= 202)
  {
    Serial.println("S");
    digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(redPin, HIGH);
  }
  else if (newCompassHeading >= 203 && newCompassHeading <= 247)
    Serial.println("SW");
  else if (newCompassHeading >= 248 && newCompassHeading <= 292)
  {
    Serial.println("W");
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(redPin, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(redPin, LOW);
  }
  else if (newCompassHeading >= 293 && newCompassHeading <= 337)
    Serial.println("NW");
}

void getGPS()                                                 // Get Latest GPS coordinates
{  
    while (Serial.available() > 0)
      gps.encode(Serial.read());

 
    
} 

void getCompass()
{
  compass_heading();
}

void getCompassHeading()
{
  compass_heading();
  delay(200);

  compassHeading = bearing;
  /*
  int north = 141;
  int east = 196;
  int south = 249;
  int west = 303;
  int diff1, diff2;

  if(compassHeading >= north && compassHeading < east)
  { 
    diff1 = compassHeading - north; //150
    diff2 = 215 - (6*diff1);
    newCompassHeading = compassHeading + diff2;
  }
  else if(compassHeading >= east && compassHeading < south)
  { 
    diff1 = compassHeading - east;
    diff2 = -103 + (0.8462*diff1);
    newCompassHeading = compassHeading + diff2;
  }
  else if(compassHeading >= south && compassHeading < west)
  { 
    diff1 = compassHeading - south;
    diff2 = -59 + (0.4906*diff1);
    newCompassHeading = compassHeading + diff2;
  }
  else if((compassHeading >= west && compassHeading <= 360) || (compassHeading >= 0 && compassHeading < nor th))
  { 
    diff1 = compassHeading - west;
    diff2 = -33 + (1.24*diff1);
    newCompassHeading = compassHeading + diff2;
  }
  /*
  else if(compassHeading > east && compassHeading <= south)
  else if(compassHeading > south && compassHeading <= west)
  else
  */

/*
  if (compassHeading >= 0 && compassHeading < 52)
    newCompassHeading = (360 + compassHeading) - 52;
  else
    newCompassHeading = compassHeading - 52;

  if(newCompassHeading < 0)
    newCompassHeading += 360;
  else if(newCompassHeading > 360)
    newCompassHeading -= 360;
  */

  newCompassHeading = compassHeading - 97;
  if (newCompassHeading < 0  )
  {
    newCompassHeading += 360;
  }
  
   Serial.print("New-> ");
   Serial.println(newCompassHeading);
  
}
void controllerLogic()
{
  
  inputHC = "";
  HC12.listen();
  HC12.flush();//clear the serial buffer for unwanted inputs     
  if (HC12.available())
  {
    
    inputHC = "";
    while (HC12.available()){
    incomingByte = HC12.read();          // Store each icoming byte from HC-12
    inputHC  = char(incomingByte);
     

    if (inputHC == "1")
    {
      turnState = 0; 
 
      
    }
    else if (inputHC == "2")
    {
      turnState = 1; 
 
    }
  }

  }

  //----------------------conditon-----------------------
    if (inputHC == "1") // left
    {
      Serial.println("1");
       
      runBoat();
 
      
    }
    else if (inputHC == "2") // right
    {
      Serial.println("2");
      
      runBoat();
    }
    else if (inputHC == "A") // Set
    {
        setHome();
        Serial.println("A");
        
    }
    else if (inputHC == "B") // return
    {
 
        returnHome();
        Serial.println("B");  
    }
    else if (inputHC == "4") //  stop
    {
      Serial.println("4");
      
      stopBoat();
    }
}

void stopBoat()
{
 
  Third.write("s");
  
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(redPin, HIGH);
  
  centerServo() ;

  
}

void runBoat()
{ 
    digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(redPin,LOW);

  
  
  while(true)                                             // dito na aandar yung code di na sa main loop
  {
    controllerLogic();
   
    if (inputHC == "4" || inputHC == "B" ){break;}

    
     if(modeState == 0)
      {
        Third.write("f");
         
        Runnning_mode();
        
      }
      else if(modeState == 1)
      {
        if ( turnState == 0)
        {
          change =  90;                        // left
          leftServo();
        
          //Turning_mode();
          
          turnState = 1;
        }
        else if ( turnState == 1)
        {
          change =  -90;                      //  right
          rightServo(); 
        
          //Turning_mode();
 
          turnState = 0;
        }
        
      }
      else if (modeState == 2)
      {
        Reverse_mode();
      }
      else if (modeState == 3)
      {
        stopBoat();
        modeState = 0;
        break;
      }   
  }

}

void TurnAroundMotor() //turns 180 degress
{

  if(turnState == 0)                      //make left turn
  {
    while(true)
    {
      controllerLogic();
      if(inputHC == "4"){break;}
      getCompassHeading();
      if (abs(Heading_B - newCompassHeading) <= 15)
      {
        Heading_B = Heading_B + 180 ;          //set new heading para sa susunod na cycle

        if(Heading_B >= 360)
        {
          Heading_B = Heading_B - 360;
        }

        stopBoat();
        Serial.print("Success");

        digitalWrite(redPin, HIGH);
        delay(1000);               
        digitalWrite(redPin, LOW);   
        break;
      }
      else
      {
        Third.write("l");
        Serial.println("Turn Left");
      }
    }
    
  }
  else if(turnState == 1)                      //make right turn
  {
    while(true)
    {
      controllerLogic();
      if(inputHC == "4"){break;}
      getCompassHeading();
      if (abs(Heading_B - newCompassHeading) <= 5)
      {

        Heading_B = Heading_B + 180 ;          //set new heading para sa susunod na cycle

        if(Heading_B >= 360)
        {
          Heading_B = Heading_B - 360;
        }
        
        stopBoat();
        digitalWrite(redPin, HIGH);
        delay(1000);               
        digitalWrite(redPin, LOW);     
        break;
      }
      else
      {
        Third.write("r");
        Serial.println("Turn Right");
      }
    }
  }
}

void Runnning_mode()
{
 //Code for trigering the Ultrasonic Runnning Mode
  distance_Front = 0, distance_Back = 0, distance_Bin = 0;
  
  setUltra();
  long duration_Bin = pulseIn(ultraBin_echo, HIGH);
  distance_Bin = (duration_Bin/2) / 29.1;
  delay(100);
  setUltra();
  long duration_Front = pulseIn(ultraFront_echo, HIGH);
  distance_Front = (duration_Front/2) / 29.1;
  delay(100);

  Serial.println("Running Mode");
  Serial.print("Front: ");
  Serial.println(distance_Front);
  Serial.print("Back: ");
  Serial.println(distance_Back);
  Serial.print("Bin: ");
  Serial.println(distance_Bin);
  Serial.println("");
  distance_Front = (duration_Front/2) / 29.1;
  delay(100);

 //-------------------------------------------------------
/*
  Serial.println("Turning Mode");
  Serial.print("Front: ");
  Serial.println(distance_Front);
  Serial.print("Back: ");
  Serial.println(distance_Back);
  Serial.print("Bin: ");
  Serial.println(distance_Bin);
  Serial.println("");
  */
  if(distance_Front <= 10 && distance_Front > 0)
  {
    centerServo(); 
    modeState = 3; 
  
  }
  else
  {
    centerServo();  
    modeState = 0;
    TurnAroundMotor();  // turns the motor 180 degrees
  }
  
}

void Reverse_mode()
{
  //Code for trigering the Ultrasonic Runnning Mode 
  distance_Front = 0, distance_Back = 0, distance_Bin = 0;
  
  setUltra();
  long duration_Front = pulseIn(ultraFront_echo, HIGH);
  distance_Front = (duration_Front/2) / 29.1;
  delay(100);
  setUltra();
  long duration_Back = pulseIn(ultraBack_echo, HIGH);
  distance_Back = (duration_Back/2) / 29.1;
  delay(100);
 //-------------------------------------------------------

  Serial.println("Reverse Mode");
  Serial.print("Front: ");
  Serial.println(distance_Front);
  Serial.print("Back: ");
  Serial.println(distance_Back);
  Serial.print("Bin: ");
  Serial.println(distance_Bin);
  Serial.println("");
  
  if(distance_Front <= 10 && distance_Front > 0)
  {
    modeState = 2; 
  }
  else
  {
    
    modeState = 0;
  }
  
  
}
void setUltra()
{
  digitalWrite(ultraMain_trig, LOW);
  delay(2);
  digitalWrite(ultraMain_trig, HIGH);
  delay(10);
  digitalWrite(ultraMain_trig, LOW); 
}

void leftServo()
{
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees                             (LEFT)
    // in steps of 1 degree
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(20);                       // waits 15ms for the servo to reach the position
  }
}

void rightServo()
{
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees                             (RIGHT)
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(20);                       // waits 15ms for the servo to reach the position
  }
}

void centerServo()
{
  for (pos = 7; pos <= 90; pos += 1) { // goes from 180 degrees to 0 degrees                             (CENTER)
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 173; pos >= 90; pos -= 1) { // goes from 180 degrees to 0 degrees                             (RIGHT)
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
}

void searchSatellites()
{
  while (waypoint == false)                         // Wait until x number of satellites are acquired before starting main loop
  {                                  
    getGPS();                                         // Update gps data
    Number_of_SATS = (int)(gps.satellites.value());
    Serial.print(Number_of_SATS);
        Serial.println(" Satellites Acquired");
    Serial.println(gps.location.lat(),6); 
    Serial.println(gps.location.lng(),6);     

    digitalWrite(redPin,HIGH);
    delay(100);
    digitalWrite(redPin,LOW);
    delay(100);
 
    //if ( Number_of_SATS >= 0 ) 
    if ( gps.location.lat() != 0 )
    {
        Serial.print(Number_of_SATS);
        Serial.print(" Satellites Acquired");
        setHome();

        digitalWrite(redPin, LOW);
        digitalWrite(LED_BUILTIN, HIGH);
 
        waypoint = true ; 
    } 
    
  }
  
}
