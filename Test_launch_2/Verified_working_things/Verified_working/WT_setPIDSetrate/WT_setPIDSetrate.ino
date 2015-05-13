/*
Make Setpoint a roll rate
Make Input roll rate

*/
/*
Listen to user for PID setpoints

Activation Sequence- put this in void setup
Start with PID set in the beginning to do position control

void loop
Collect data and save it
PID
Actuate
Listen for user input

If user input
ask for P
ask for I
ask for D
ask for setpoint
Set P,I,D, setpoint appropiately
Set servos straight for 5 seconds

*/
#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_Simple_AHRS.h>
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
#define sampleFreq   25.0f        // sample frequency in Hz
#define betaDef      3.0f         // 2 * proportional gain
#include <PID_v4.h>
double Setpoint, Input, Output;  
float P;
float I;
float D;
float rate;
float interval;
float time;
float previousTime;
float previous_psi;
float diff;

PID myPID(&Input, &Output, &Setpoint, 1.5, .01, .5, REVERSE);//pid///////////////
Servo servo_1;
Servo servo_2;
float psi, theta, phi;
float invSqrt(float x);
volatile float beta = betaDef;    // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // quaternion of sensor frame relative to auxiliary frame

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Setting up...");
  Setpoint = 90;
  servo_1.attach(2);
  servo_2.attach(3);
  accel.begin();
  mag.begin();
  bmp.begin();
  gyro.begin();
  myPID.SetMode(AUTOMATIC); //turn PID on
  myPID.SetOutputLimits(-400,400);
  arm_seq();
  Serial.println("Done with Setup");
}

void loop() {
  collect_data();
  pid();
  actuate();
  input();
  servos_straight;
}

void arm_seq(){
  while (Serial.available() == false) {
    Serial.println("Ready to be armed. Type 'a'");
    collect_data();
    delay(1000);
  }
  if (Serial.available()) {
      // Read command
      byte arm = Serial.read();
  
      if (arm == 'a'){
        Serial.println("ARMED");
      }
  }
  delay(1000);
}

void servos_straight(){
  servo_1.write(80);/////////////////////////////////////////////////////////////////////////////////////////////////
  servo_2.write(100);////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.print("Servos being commanded to 0 degrees\n");
  delay(5000);
}

void collect_data(){
  MadgwickAHRSupdate();
  psi = atan2((2 * q1 * q2) - (2 * q0 * q3), (2 * pow(q0, 2)) + (2 * pow(q1, 2)) - 1);
  psi *= 180.0/PI;
  if (psi < 0) psi+=360;
  Serial.println(psi);
}

void pid(){
  Input = psi;///////////////////////////////////////////////////Put your roll into the Input
  myPID.Compute();
  Serial.print(" Output (from PID): ");
  Serial.println(Output);
}

void actuate(){
  double servoOutput = map(Output, -400, 400, 50, 130);////////////////////Set servo constraint here
  servo_1.write(servoOutput);////////////////////////////////////////////////////Tweak with calibration
  servo_1.write(servoOutput);////////////////////////////////////////////////////Tweak with calibration
  Serial.print("; Servo Output: ");
  Serial.println(servoOutput);
}

void compute_rate(){
  previous_psi = psi;
  previousTime = time;
  time = micros();
  interval = time - previousTime;
  diff = psi-previous_psi;
  if (diff > abs(180)){
    if (diff >= 0){ 
    diff = 360-diff;
  }
    else { 
    diff += 360; 
  }
  }
  
  rate = (diff)/interval;
}

void input(){
  if (Serial.available()) {  
    Serial.println("Enter P: ");
    while (Serial.available() == false){
    }
    P = Serial.parseFloat();
    
    Serial.println("Enter I: ");
    while (Serial.available() == false){
    }
    I = Serial.parseFloat();
    
    Serial.println("Enter D: ");
    while (Serial.available() == false){
    }
    D = Serial.parseFloat();
    
    Serial.println("Enter P: ");
    while (Serial.available() == false){
    }
    Setpoint = Serial.parseFloat();
    myPID.SetTunings(P,I,D);
  }
}

void MadgwickAHRSupdate() {
  /* Get a new sensor event */
  sensors_event_t eventg;
  gyro.getEvent(&eventg);
  float  gx = eventg.gyro.x;
  float  gy = eventg.gyro.y;
  float  gz = eventg.gyro.z;

  /* Get a new sensor event */
  sensors_event_t eventa;
  accel.getEvent(&eventa);
  float  ax = (eventa.acceleration.x)*101;
  float  ay = eventa.acceleration.y*101;
  float  az = eventa.acceleration.z*101;
  /* Get a new sensor event */
  sensors_event_t eventm;
  mag.getEvent(&eventm);
  float  mx = eventm.magnetic.x;
  float  my = eventm.magnetic.y;
  float  mz = eventm.magnetic.z;
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    MadgwickAHRSupdateIMU();
    return;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU() {
  /* Get a new sensor event */
  sensors_event_t eventg;
  gyro.getEvent(&eventg);
  float  gx = eventg.gyro.x;
  float  gy = eventg.gyro.y;
  float  gz = eventg.gyro.z;

  /* Get a new sensor event */
  sensors_event_t eventa;
  accel.getEvent(&eventa);
  float  ax = eventa.acceleration.x;
  float  ay = eventa.acceleration.y;
  float  az = eventa.acceleration.z;
  /* Get a new sensor event */
  sensors_event_t eventm;
  mag.getEvent(&eventm);
  float  mx = eventm.magnetic.x;
  float  my = eventm.magnetic.y;
  float  mz = eventm.magnetic.z;
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 , _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}
