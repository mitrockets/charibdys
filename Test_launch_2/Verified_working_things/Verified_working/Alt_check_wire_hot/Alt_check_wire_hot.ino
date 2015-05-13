/*
This code checks for the altimeter signal
When given, it tursn the hot wire on for 10 seconds
*/
int altimeter_check_pin = 3;     
int altimeter_value = 0;
unsigned long time;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  time = millis();
  altimeter_value = analogRead(altimeter_check_pin);
  Serial.print(altimeter_value);
  Serial.print(":");
  Serial.println(time);
  
}
