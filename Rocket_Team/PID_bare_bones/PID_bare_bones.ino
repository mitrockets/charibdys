#include <PID_v4.h>

double Setpoint, Input, Output;  
PID myPID(&Input, &Output, &Setpoint, 1.5, .01, .5, REVERSE);//pid///////////////
Servo myservo;

void setup() {
  // put your setup code here, to run once:
  Setpoint = 140;//////////////////////////////////////////////////////////////////////////////////// Set this
  myPID.SetMode(AUTOMATIC); //turn PID on
  myPID.SetOutputLimits(-400,400);

}

void loop() {
  //PID stuff
  Input = roll_mag;///////////////////////////////////////////////////Put your roll into the Input
  myPID.Compute();
  Serial.print(" Output (from PID): ");
  Serial.println(Output);
  double servoOutput = map(Output, -400, 400, 50, 130);////////////////////Set servo constraint here
  myservo.write(servoOutput);////////////////////////////////////////////////////Add your two servos here
  Serial.print("; Servo Output: ");
  Serial.println(servoOutput);
}



if (Serial.available()) {
      // Read command
      byte straight = Serial.read ();
      
myPID.SetTunings(Kp, Ki, Kd);
