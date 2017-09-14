#include <Servo.h>

Servo myservo;
double pH = 0.0;
// connect motor controller pins to Arduino digital pins
// motor one
int in3 = 8;
int in4 = 3;
int in1 = 1;
int in2 = 0;
int sensorValue = 0.0;
int sensorPin = A5;// select the input pin for the potentiometer


double readpH(void){

  // 240 for ph = 4, 462 for ph = 7, 619 for ph = 10
  // y = 0.0157x + 0.1124
  
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  return 0.0157*sensorValue + 0.1124;   
}

void setup() {
  // put your setup code here, to run once:
   myservo.attach(A0);  // attaches the servo on pin 9 to the servo object
 //  sSerial.begin(9600);      //Set Baud Rate
    //Serial.println("Run keyboard control");
    // set all the motor control pins to outputs
    //pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //pH code
  myservo.write(35);
  delay(5000);
  pH = readpH();
  //Serial.println("pH: ");
  //Serial.println(pH);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(4000);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(1000);
  
 boolean flag = true;
  while (flag) {
    //insert code to dispense base
    //i guess this is it 
    //Serial.println("in flag loop");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    delay(10000);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    delay(1000);
    //then stop dispensing base
    //insertl3    
    double pH1 = readpH();
    //Serial.println("pH: ");
   // Serial.println(pH1);
    if (pH1 > 6.5) { //tweak: if pH is within range
      delay(10000);               //tweak: wait some time
      double pH2 = readpH();
      if (pH2 > 6.5) { //tweak: if in range and the second pH is close enough to the first pH, exit the loop
        flag = false;
      }
    }
  }
  //Serial.println("neutralized");

}
