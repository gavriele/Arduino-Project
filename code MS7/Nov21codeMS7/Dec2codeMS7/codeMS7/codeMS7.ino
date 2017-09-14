#include <Servo.h>
#include <math.h>
#include <SoftwareSerial.h>
#include "enes100.h"
#define echoPin 11 // Echo Pin
#define trigPin 2 // Trigger Pin

//distance sensor vars
int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration, distance; // Duration used to calculate distance

//angle vals
double thetaUP = 1.5;
double thetaRIGHT = 0;// -.35;
double thetaDOWN = -1.5;
double thetaLEFT = 3; //never used

//initial location
double x;
double y;
double theta;

//Declare the software serial which on pins 8,9 which
//are connected to the RF module tx and rx.
SoftwareSerial sSerial(12,13);

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

void pause(void)                    //Stop
{
  digitalWrite(E1,LOW);   
  digitalWrite(E2,LOW);
  delay(15);      
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
void turn_R (char a,char b)             //Turn Right 
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
  delay(10);
  
}
void turn_R90 (char a,char b)             //Turn Right 90
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
  delay(7500);//90
  
}
void turn_L (char a,char b)             //Turn Left
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,LOW);
  delay(10);
}

//OTHER CODE
double readpH(void){

  // 240 for ph = 4, 462 for ph = 7, 619 for ph = 10
  // y = 0.0157x + 0.1124
  
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  Serial.println(0.0157*sensorValue + 0.1124);
//  sSerial.print("pH: ");
//  sSerial.println(0.0157*sensorValue + 0.1124);
  return 0.0157*sensorValue + 0.1124;   
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
  for (pos = 0; pos <= 110; pos ++) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
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

boolean yesBarrier(){
   /* The following trigPin/echoPin cycle is used to determine the
  distance of the nearest object by bouncing soundwaves off of it. */ 
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2); 

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
 
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
 
  //Calculate the distance (in cm) based on the speed of sound.
  distance = duration/58.2;
  Serial.print("before distance: ");
  Serial.println(distance);
//  rf.sendMessage("distance before if is ");
  sSerial.print("(s test) ");
  sSerial.print(distance);
  //distance+=5;
  if (distance >= maximumRange || distance <= minimumRange){ // THERE IS NO BARRIER
    /* Send a negative number to computer and Turn LED ON 
    to indicate "out of range" */
    Serial.println("-1");
    rf.sendMessage("inside the if: -1");
    delay(50);
    return false;
  }else {// THERE IS A BARRIER
    sSerial.print(distance*10);
    /* Send the distance to the computer using Serial protocol, and
    turn LED OFF to indicate successful reading. */
    Serial.println(distance*10); 
    delay(50);
    return true;
  }
 
  //Delay 50ms before next reading.
  
}




void turnToTheta(double thetaIn){
  double theta = getTheta();
  //if(thetaIn == thetaUP){
     /* while(!((theta > (thetaIn-.05) && theta < (thetaIn+.05)) || (theta > (-thestaIn-.05) && theta < (-thetaIn+.05))   )){
          myservo.write(160);
          turn_R(255,255);
          theta = getTheta();
      }
  }else{*/
  //double deltatheta = theta - thetaIn;

    //if (deltatheta > 0) {
      while(!(theta > (thetaIn-.1) && theta < (thetaIn+.1))){
              rf.sendMessage(thetaIn);
              rf.sendMessage("  |  ");
              rf.sendMessage(theta);
              myservo.write(150);
              turn_R(255,255);
              theta = getTheta();
              Serial.println(theta);
      }
    //}
    /*else {
      while(!(theta > (thetaIn-.08) && theta < (thetaIn+.08))){
              myservo.write(150);
              turn_L(255,255);
              theta = getTheta();
              Serial.println(theta);
      }
    }*/
 //t }
  pause();
}


void navigateToPool(boolean isTop){
  rf.sendMessage("navigateToPool begin\n");
  if(isTop){
    /*while(x<1940){
      myservo.write(150);
      smAdvance(255,255);
      x = getX();
    }*/
    //correctNavigate(1940, 1700);
  }else{
    //no barrier on the bottom
    //follow coordinate from bottom to site
    //coordinates are estimates and should probably be edited
   /* while(x<1500){
      myservo.write(150);
      smAdvance(255,255);
      x = getX();
    }
    pause();
    turnToTheta(thetaUP);
    pause();
    while(y<1700){
        myservo.write(150);
        smAdvance(255,255);
        y = getY();
    }
    turnToTheta(thetaRIGHT);
    //we should now be aligned with the top position
    navigateToPool(true);
    */
    correctNavigate(1500,300);
    //correctNavigate(1960,1400);
  }
  correctNavigate(1500,1400);
  correctNavigate(1900,1350);
  rf.sendMessage("navigateToPool finished\n");
}


void navigateMain(boolean isTop){
  rf.sendMessage("navigateMain begin\n");
  turnToTheta(thetaRIGHT);
  delay(2000);
  if(yesBarrier()){ 
    rf.sendMessage("BARRIER ALERT\n");
    //isTop: change theta to down and move y while>250
    sSerial.print(isTop);
    if(isTop){
      isTop = false;
      /*turnToTheta(thetaDOWN);
      while(y>250){
        myservo.write(150);
        smAdvance(255,255);
        y = getY();
        isTop = false;
      }*/
      rf.sendMessage("navigating to bottom to avoid barrier\n");
      correctNavigate(500,250);
    }else{ //else: change theta to up and move y while<1700  
      isTop = true;
      /*turnToTheta(thetaUP);
      while(y<1600){
        myservo.write(150);
        smAdvance(255,255);
        y = getY();
      }*/
      rf.sendMessage("navigating to top to avoid barrier\n");
      correctNavigate(500, 1700);
    }
    //change theta to right
    //turnToTheta(thetaRIGHT);
    //navigate to pool
    //navigateToPool(true);
  }else{
    rf.sendMessage("NO BARRIER\n");
  }
  navigateToPool(isTop);
  pause();
}
//Theta up - 1.5
//           Theta right - 0
//Theta down - -1.5
/*
 * Implementation of newCorrectNavigate
//navigate forwards
theta = getTheta();
if(abs(theta - thetaLeft)>.8) newCorrectNavigate(thetaLeft);
*/
void newCorrectNavigate(int thetaDest){
  theta = getTheta();
  if(thetaDest = thetaUP){
    while(!(abs(theta - thetaDest) < 0.08)){//Not in range of 
      //turn
      turn_R(255,255);
    }
  }
  else if(thetaDest = thetaDOWN){
    while(!(abs(theta - thetaDest) < 0.08)){//Not in range of 
      //turn
        turn_R(255,255);
    }
  }
   else if(thetaDest = thetaRight){
    while(!(abs(theta - thetaDest) < 0.08)){//Not in range of 
      //turn
      turn_R(255,255);
    }
  }
}
void correctNavigate(int xdest, int ydest) { //Correct right navigate
  x = getX();
  y = getY();
  theta = getTheta();
  double thetadest = (3/3.14159265359) * atan2(ydest - y, xdest - x); // direct navigate to pool

  turnToTheta(thetadest);
  
  while (abs(xdest - x) > 100 || abs(ydest - y) > 100) {
    thetadest = (3/3.14159265359) * atan2(ydest - y, xdest - x);

    rf.sendMessage("current position: ");
    rf.sendMessage(x);
    rf.sendMessage(", ");
    rf.sendMessage(y);
    rf.sendMessage(", ");
    rf.sendMessage(theta);
    rf.sendMessage(", ");
    rf.sendMessage(thetadest);
    rf.sendMessage("\n");

    if (abs(theta - thetadest) < 0.08) {//Theta within .16
      rf.sendMessage("on track\n");
      analogWrite (E1,255);
      digitalWrite(M1,HIGH);    
      analogWrite (E2,255);    
      digitalWrite(M2,HIGH);
    }
    else if (theta - thetadest <= -0.08) {//Too far right
      rf.sendMessage("tracking left\n");
      //thetadest = (3/3.14159265359) * atan2(ydest - y, xdest - x); // direct navigate to pool
      turnToTheta(thetadest);
    }
    else if (theta - thetadest >= 0.08) { //too far left
      rf.sendMessage("tracking right\n");
      //thetadest = (3/3.14159265359) * atan2(ydest - y, xdest - x); // direct navigate to pool
      turnToTheta(thetadest);
    }
    delay(25);
    
    x = getX();
    y = getY();
    theta = getTheta();
  }
}

void setup(void) 
{ 
  //distance sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //pH servo
  myservo.attach(A0);  // attaches the servo on pin 9 to the servo object
//  int i;
//  for(i=4;i<=7;i++)
//    pinMode(i, OUTPUT);  
  Serial.begin(9600);      //Set Baud Rate
  Serial.println("Run keyboard control");
  
  //Initialize the software and hardware serial for RF monitor
  sSerial.begin(9600);
  
  delay(500);
  
  //Reset the state of the server, and send a message indicating
  //our status.
  rf.resetServer();
  rf.sendMessage("Neutral team Connected.\n");
}

void loop(void) 
{
  x = getX();
  y = getY();
  theta = getTheta();
  rf.sendMessage("x: ");
  rf.sendMessage(x);
  rf.sendMessage("\n");
  rf.sendMessage("y: ");
  rf.sendMessage(y);
  rf.sendMessage("\n");
  
  boolean isTop = false;
  double pH = 0;
  myservo.write(150);

  //Check y value move to obstacle position
  if(y > 950){//Move to top
    /*turnToTheta(thetaUP); 
    pause();
    while(y<1600){
      myservo.write(150);
      smAdvance(255,255);
      y = getY();
    }*/
    rf.sendMessage("navigating to top start\n");
    correctNavigate(400,1600);
    rf.sendMessage("navigated to top\n");
    isTop = true;
  }else{//Move to bottom
    /*turnToTheta(thetaDOWN);
    pause();
    while(y>300){
      myservo.write(150);
      smAdvance(255,255);
      y = getY();
    }*/
    rf.sendMessage("navigating to bottom start\n");
    correctNavigate(400,400);
    rf.sendMessage("navigated to bottom\n");
    isTop = false;
  }
  pause();
  rf.sendMessage("navigateMain call\n");
  navigateMain(isTop);

  //pH code

  delay(1000);
  myservo.write(30);  
  delay(4000);
  pH = readpH();
  //print them to the monitor
  sSerial.print("pH: ");
  sSerial.println(pH);
 
}
