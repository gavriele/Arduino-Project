// connect motor controller pins to Arduino digital pins
// motor one
int in3 = 8;
int in4 = 3;
int in1 = 1;
int in2 = 0;
void setup()
{
  // set all the motor control pins to outputs
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}
void loop()
{ 
  
  /*digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(900);
  digitalWrite(in3, LOW);*/
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(100);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(1000);
}
