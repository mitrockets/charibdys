/*
This code checks for the altimeter signal
When given, it tursn the hot wire on for 10 seconds
*/
int altimeter_check_pin = 3;     
int altimeter_value = 0;
unsigned long now, start;
int deploypin = 5;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(deploypin,OUTPUT);
  digitalWrite(deploypin,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  pyro_detect();
  deploy();  
}

void pyro_detect(){
  while (altimeter_value < 20){
    altimeter_value = analogRead(altimeter_check_pin);
    Serial.print(altimeter_value);
  }
}

void deploy(){
  start = millis();
  now = millis();
  while (now-start < 15000){/////////////////////Edit delay for hot wire
    digitalWrite(deploypin, HIGH);
    now= millis();
   }
  digitalWrite(deploypin, LOW);
}
