#include <Servo.h>
#include <math.h>
#include <SoftwareSerial.h>
#include "enes100.h"
#define echoPin 11 // Echo Pin
#define trigPin 2 // Trigger Pin

int in3 = 8;
int in4 = 3;
double pH = 0.0;

//distance sensor vars
int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration, distance; // Duration used to calculate distance

//angle vals
double thetaUP = 1.5;
double thetaRIGHT = -0.049;
double thetaDOWN = -1.6;
double thetaLEFT = 3.1; //never used
double thetaAdjustTop = -.65;
double thetaAdjustBottom = .0614;

boolean wasTop;
boolean isTop2 = false;
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
  delay(500);
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
  delay(10);//quarter revolution
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
void turn_R (char a,char b)             //Turn Right 45
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
  delay(5);//45
  
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
  delay(5);
}

//OTHER CODE
double readpH(void){

  // 240 for ph = 4, 462 for ph = 7, 619 for ph = 10
  // y = 0.0157x + 0.1124
  
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  //Serial.println(0.0166*sensorValue + 0.3204);
  return 0.0166*sensorValue + 0.5004;   
}

double readpH2(void){
  unsigned long int avgValue;  //Store the average value of the sensor feedback
  int buf[10],temp;
  
  for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(sensorPin);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+=buf[i];
  double phValue=(double)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=3.5*phValue;                      //convert the millivolt into pH value 
  return phValue;
   
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
  //distance+=5;
  rf.sendMessage("" + distance);

  if (distance >= maximumRange || distance <= minimumRange){ // THERE IS NO BARRIER
    /* Send a negative number to computer and Turn LED ON 
    to indicate "out of range" */
    Serial.println("-1");
    //rf.sendMessage("inside the if: -1");
    delay(50);
    return false;
  }else {// THERE IS A BARRIER
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
    while(!(theta > (thetaIn-.05) && theta < (thetaIn+.05))){
            myservo.write(150);
            turn_L(255,255);
            theta = getTheta();
            Serial.println(theta);
    }
  pause();
}


void turnToThetaR(double thetaIn){
  double theta = getTheta();
    while(!(theta > (thetaIn-.05) && theta < (thetaIn+.05))){
            myservo.write(150);
            turn_R(255,255);
            theta = getTheta();
            Serial.println(theta);
    }
  pause();
}

void navigateToPool(boolean isTop){
  y = getY();
  x = getX();
  if(isTop){
    while(x<1540){
      myservo.write(150);
      smAdvance(255,255);
      x = getX();
    }
    turnToThetaR(thetaDOWN);
    y = getY();
    while(y>1520){ //FIND THIS Y VALUE
      myservo.write(150);
      smAdvance(255,255);
      y = getY();
    }
    turnToTheta(thetaRIGHT);
    while(x<1791){
      myservo.write(150);
      smAdvance(255,255);
      x = getX();
    }
  }else{
    //no barrier on the bottom
    //follow coordinate from bottom to site
    //coordinates are estimates and should probably be edited
    while(x<1540){ //MAYBE ADJUST?
      myservo.write(150);
      smAdvance(255,255);
      x = getX();
    }
    pause();
    turnToTheta(thetaUP);
    pause();
    while(y<1353){
        myservo.write(150);
        smAdvance(255,255);
        y = getY();
    }
    turnToThetaR(thetaAdjustBottom);
    //we should now be aligned with the top position
    while(x<1791){
      myservo.write(150);
      smAdvance(255,255);
      x = getX();
    }
    
  }
}


void navigateMain(boolean isTop){
  if(isTop){
    turnToThetaR(0);
  }else{
    turnToTheta(thetaRIGHT);
  }
  delay(2000);
  if(yesBarrier()){ 
    rf.sendMessage("BARRIER ALERT");
    //isTop: change theta to down and move y while>250
    if(isTop){
      turnToThetaR(thetaDOWN);
      while(y>250){
        myservo.write(150);
        smAdvance(255,255);
        y = getY();
      }
      turnToTheta(thetaRIGHT);
      isTop2 = false;
      navigateToPool(false);
    }else{ //else: change theta to up and move y while<1700
      turnToTheta(thetaUP);
      while(y<1600){
        myservo.write(150);
        smAdvance(255,255);
        y = getY();
      }
      if(wasTop){
        turnToTheta(thetaRIGHT);
      }else{
        turnToThetaR(thetaRIGHT);
      }
      isTop2 = true;
      navigateToPool(true);
    }
    //change theta to right
    //navigate to pool
  }else{
    isTop2 = isTop;
    rf.sendMessage("NO BARRIER");
    //Navigate to pool
    navigateToPool(isTop);
  }
  //pause();
}

void correctNavigate(int xdest, int ydest) { //Correct right navigate
  x = getX();
  y = getY();
  theta = getTheta();

  while (abs(x - xdest) > 50 || abs(y - ydest) > 50) {
    double thetadest = (3/3.14159265359) * atan2(ydest - y, xdest - x); // direct navigate to pool

    if (abs(theta - thetadest) < 0.08) {//Theta within .08
      rf.sendMessage("on track");
      analogWrite (E1,255);
      digitalWrite(M1,HIGH);    
      analogWrite (E2,255);    
      digitalWrite(M2,HIGH);
    }
    else if (theta - thetadest <= -0.08) {//Too far left
      rf.sendMessage("tracking left");
      analogWrite (E1,255);
      digitalWrite(M1,HIGH);    
      analogWrite (E2,127);    
      digitalWrite(M2,HIGH);
    }
    else if (theta - thetadest >= 0.08) {
      rf.sendMessage("tracking right");
      analogWrite (E1,127);
      digitalWrite(M1,HIGH);    
      analogWrite (E2,255);    
      digitalWrite(M2,HIGH);
    }
    delay(500);
    
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
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //pH servo
  myservo.attach(A0);  // attaches the servo on pin 9 to the servo object
//  int i;
//  for(i=4;i<=7;i++)
//    pinMode(i, OUTPUT);  
  //Serial.begin(9600);      //Set Baud Rate
  //Serial.println("Run keyboard control");
  
  //Initialize the software and hardware serial for RF monitor
  sSerial.begin(9600);
  
  delay(500);
  
  //Reset the state of the server, and send a message indicating
  //our status.
  rf.resetServer();
  rf.sendMessage("Neutral team Connected.");
}

void loop(void) 
{
  myservo.write(150);
  x = getX();
  y = getY();
  theta = getTheta();
  rf.sendMessage(x);
  rf.sendMessage(",");
  rf.sendMessage(y);
  rf.sendMessage(",");
  rf.sendMessage(theta);
  boolean isTop = false;
  myservo.write(150);

  //Check y value move to obstacle position
  if(y > 950){//Move to top
    turnToTheta(thetaUP); 
    pause();
    while(y<1600){
      myservo.write(150);
      smAdvance(255,255);
      y = getY();
    }
    isTop = true;
    wasTop = true;
  }else{//Move to bottom
    turnToTheta(thetaDOWN);
    pause();
    while(y>325){
      myservo.write(150);
      smAdvance(255,255);
      y = getY();
    }
    wasTop = false;
    isTop = false;
  }
  pause();
  navigateMain(isTop);
  stop(); 
  //pH code 
  myservo.write(35); 
  
  delay(10000);
  pH = readpH();
  //Serial.println(pH);
  if(pH<6.0){
    rf.sendMessage("begin pH: <");
    rf.sendMessage(pH);
    rf.sendMessage(">end of pH");
  }

  //collect
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(3500);
  digitalWrite(in3, LOW);
  delay(1000);
  
  //failsafe

  myservo.write(150);
  if(isTop2){
    turnToTheta(thetaRIGHT + 0.25);
  }else{
    turnToTheta(thetaAdjustBottom + 0.25);
  }
  stop();
  delay(100);

  //pH code failsafe 
  myservo.write(35); 
  
  delay(10000);
  pH = readpH();
  //Serial.println(pH);
  if(pH<6.0){
    rf.sendMessage("begin pH: <");
    rf.sendMessage(pH);
    rf.sendMessage(">end of pH");
  }

  //collect failsafe
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(3500);
  digitalWrite(in3, LOW);
  delay(1000);

  //failsafe2
  
  myservo.write(150);
  if(isTop2){
    turnToThetaR(thetaRIGHT - 0.25);
  }else{
    turnToThetaR(thetaAdjustBottom - 0.25);
  }
  stop();
  delay(100);

  //pH code failsafe2 
  myservo.write(35); 
  
  delay(10000);
  pH = readpH();
  //Serial.println(pH);
  if(pH<6.0){
    rf.sendMessage("begin pH: <");
    rf.sendMessage(pH);
    rf.sendMessage(">end of pH");
  }

  //collect failsafe2
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(3500);
  digitalWrite(in3, LOW);
  delay(1000);
  
  while(true){
    
  }
}
