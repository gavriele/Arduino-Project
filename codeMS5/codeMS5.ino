#include <Servo.h>
#include <SoftwareSerial.h>
#include "enes100.h"

//Declare the software serial which on pins 8,9 which
//are connected to the RF module tx and rx.
SoftwareSerial sSerial(10,11);

//Declare an enes100::RfClient which is the interface for
//receiving markers and sending messages to the vision system.
//  To use a hardware serial connection:
//    enes100::RfClient<HardwareSerial rf(&Serial1);
//                                            ^
//                              whatever serial is used
enes100::RfClient<SoftwareSerial> rf(&sSerial);

//Declare an enes100::Marker.  This marker will store the id,
//x, y, theta, and time values of a received marker.
enes100::Marker marker;

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

//Standard PWM DC control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 7;    //M1 Direction Control
int sensorPin = A5;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

void stop(void)                    //Stop
{
  digitalWrite(E1,LOW);   
  digitalWrite(E2,LOW);      
}

   //MOVE FORWARD CODE
void advance(char a,char b)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
  delay(3000);
  //to cover 4m requires 14.46863119 advances
}
void halfAdvance(char a,char b)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
  delay(1500);//half revolution
  //to cover 4m requires 14.46863119 advances
}
void smAdvance(char a,char b)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
  delay(750);//quarter revolution
  //to cover 4m requires 14.46863119 advances
}
void endAdvance(char a,char b)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
  delay(1405.89357);//represents the remainder after 14  full revolutions to finish 4m
}
void fullArena()          //cover full arena
{
  delay(1000);//don't start right away
  for(int i=0;i<14;i++){
    advance(255, 255);
  }
  endAdvance(255, 255);
  stop();
}

    //OTHER MOVEMENT CODE
void back_off (char a,char b)          //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);   
  analogWrite (E2,b);    
  digitalWrite(M2,LOW);
}
void turn_L (char a,char b)             //Turn Left 45
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
  delay(3500);//45
  
}
void turn_L90 (char a,char b)             //Turn Left 90
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
  delay(7500);//90
  
}
void turn_R (char a,char b)             //Turn Right
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,LOW);
}

//OTHER CODE
double readpH(void){

  // 240 for ph = 4, 462 for ph = 7, 619 for ph = 10
  // y = 0.0157x + 0.1124
  
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  //Serial.println(0.0166*sensorValue + 0.3204);
  return 0.0166*sensorValue + 0.5204;   
}


void pHturn_Down(){
  for (pos = 0; pos <= 160; pos ++) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void pHturn_Up(){
  for (pos = 160; pos >= 120; pos --) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
void pHturn_D2(){ //down to solution
  for (int p=160; p >= 25; p--) { // goes from 180 degrees to 0 degrees
    myservo.write(p);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

//get x coordinate
double getX(){
  //double[] coords = {0, 0, 0}; //x, y, theta
  double x = 0;
  delay(100);
  //rf.ReceiveMarker returns true if a marker was received
  //before timing out, and false otherwise.
  if(rf.receiveMarker(&marker, 104)){
//    Serial.println("in function");
//    Serial.print("x: ");
//    Serial.println(marker.x*1000);
    x = marker.x*1000;
  }
  return x;

//    if(marker.x >= 1.0) {
//      rf.sendMessage("t1 reached right side");
//    }
//  }
}

//get y coordinate
double getY(){
  //double[] coords = {0, 0, 0}; //x, y, theta
  double y = 0;
  delay(100);
  //rf.ReceiveMarker returns true if a marker was received
  //before timing out, and false otherwise.
  if(rf.receiveMarker(&marker, 104)){
//    Serial.println("in function");
//    Serial.print("y: ");
//    Serial.println(marker.y*1000);
    y = marker.y*1000;
  }
  return y;

//    if(marker.x >= 1.0) {
//      rf.sendMessage("t1 reached right side");
//    }
//  }
}

//get theta
double getTheta(){
  //double[] coords = {0, 0, 0}; //x, y, theta
  double theta = 0;
  delay(100);
  //rf.ReceiveMarker returns true if a marker was received
  //before timing out, and false otherwise.
  if(rf.receiveMarker(&marker, 104)){
//    Serial.println("in function");
//    Serial.print("theta: ");
//    Serial.println(marker.theta);
    theta = marker.theta;
  }
  return theta;

//    if(marker.x >= 1.0) {
//      rf.sendMessage("t1 reached right side");
//    }
//  }
}

void setup(void) 
{ 
  myservo.attach(A0);  // attaches the servo on pin 9 to the servo object  
  Serial.begin(9600);      //Set Baud Rate
  Serial.println("Run keyboard control");
  
  //Initialize the software and hardware serial
  sSerial.begin(9600);
  
  delay(500);
  
  //Reset the state of the server, and send a message indicating
  //our status.
  rf.resetServer();
  rf.sendMessage("Neutral team Connected.");
}

void loop(void) 
{
  //myservo.write(140);
  //advance(255, 255);
  double pH = readpH();
  Serial.println(pH);
  delay(1000);
  
//  double x = getX();
//  double y = getY();
//  double theta = getTheta();
//  //myservo.write(160);
//
//  //move to spot
//  /*while(x < 1940){
//    myservo.write(160);
//    smAdvance(255, 255);
//    x = getX();
//    rf.sendMessage(x);
//  }*/
//
//  stop();
//  //pH code
//  Serial.println("loop");
//  
//  pHturn_D2();
//  myservo.write(40);  
//  delay(5000);
//  readpH();


//  for (int ijkl=1; ijkl<=400; ijkl++) {
//    myservo.write(25);  
//    delay(2000);
//    readpH();
//    delay(2000);
//  }


  //readpH();


  
}
