#include <Servo.h>
Servo serv_1;
Servo serv_2;
/*
.write(90) should be center but you have to check and adjust
.write(180) one side
.write(0) other side
Use this code to find center, max, and min of each servo when mounted
*/


void setup() {
serv_1.attach(8);
serv_2.attach(9);
}
//Servo 2 middle is 105
//Max is 135
//Min is
void loop() {
//Label the servos
  serv_1.write(90);
  serv_2.write(90);
  delay(1000);
}
