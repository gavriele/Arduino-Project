int in1 = 1;
int in2 = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  //suck base to front
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(3000);
  digitalWrite(in1, LOW);
  
  boolean acidic = true;
  while (acidic) {
    //insert code to dispense base
    //i guess this is it 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    delay(1500);
    //then stop dispensing base
    //insert
    digitalWrite(in1, LOW);
    delay(1000);
    double pH1 = readpH();
    if (pH1 > 6.5) { //tweak: if pH is within range
      delay(10000);               //tweak: wait some time
      double pH2 = readpH();
      if (abs(pH1 - pH2) < 0.5) { //tweak: if in range and the second pH is close enough to the first pH, exit the loop
        acidic = false;
      }
    }
  }
}
