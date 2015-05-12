/*
This is for windtunnel testing
No controls
Collects IMU data and stores it in SD card
Has a set sequence of servo commands in square wave of increasing amplitude
Purpose is to gauge dynamics of system
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
#include <SD.h>
#include <SPI.h>



Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
#define sampleFreq   25.0f        // sample frequency in Hz
#define betaDef      3.0f         // 2 * proportional gain

float psi, theta, phi;
float invSqrt(float x);
volatile float beta = betaDef;    // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // quaternion of sensor frame relative to auxiliary frame
unsigned long now;


float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

unsigned long start;
bool start_again = false;
Servo servo_1;
Servo servo_2;

int count = 0;
int delay_vals[] =   {3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000};
int actuate_vals[] = {95  ,85  ,100 ,80  ,105 ,75  ,110 ,70  ,115 ,65  ,120 ,60};
double dataFile_values[10];
const int chipSelect = 53;

void setup() {
  File dataFile = SD.open("Data.txt", FILE_WRITE);
  Serial.begin(9600);
  now = millis();
  dataFile.println("Time:");
  dataFile.println(now);
  Serial.println("Setting up...");
  dataFile.println("Setting up...");
  initialize_SD_card();
  servo_1.attach(2);
  servo_2.attach(3);
  now = millis();
  dataFile.println("Time:");
  dataFile.println(now);
  Serial.println("Time:");
  Serial.println(now);
  Serial.println("Done with Setup");
  dataFile.println("Done with Setup");
  accel.begin();
  mag.begin();
  bmp.begin();
  gyro.begin();
}

void loop() {
  Serial.println("Starting loop...");
  beginning:
  //Ask via bluetooth if ready to start
  //Send a blip of IMU roll data via bluetooth so we know it is working
  start_again = false;
  arm_seq();
  
  if (start_again == true){
    goto beginning;
  }
  
  //Check if servos are straight
  servos_straight();
  
  
  //Collect data and do servo commands
  collect_actuate();
  

}

void arm_seq(){
  File dataFile = SD.open("Data.txt", FILE_WRITE);
  while (Serial.available() == false) {
    now = millis();
    dataFile.println("Time:");
    dataFile.println(now);
    Serial.println("Time:");
    Serial.println(now);
    Serial.println("Ready to be armed. Type 'a'");
    dataFile.println("Ready to be armed. Type 'a'");
    collect_data();
    delay(1000);
  }
  if (Serial.available()) {
      // Read command
      byte arm = Serial.read();
      now = millis();
      if (arm == 'a'){
        dataFile.println("Time:");
        dataFile.println(now);
        Serial.println("ARMED");
        dataFile.println("ARMED");
      }
      else {
        dataFile.println("Time:");
        dataFile.println(now);
        Serial.println("Incorrect key. Press 'a' to arm. Starting over...");
        dataFile.println("Incorrect key. Press 'a' to arm. Starting over...");
        begin_again();
      }
  }
  dataFile.close();
  delay(1000);
}

void begin_again(){
  start_again = true;
}

void servos_straight(){
  now = millis();
  File dataFile = SD.open("Data.txt", FILE_WRITE);
  servo_1.write(80);
  servo_2.write(100);
    dataFile.println("Time:");
    dataFile.println(now);
    Serial.println("Time:");
    Serial.println(now);
    Serial.print("Servos being commanded to 0 degrees\n");
  Serial.print("Check slots on rocket- Make sure fins will deploy\n");
  dataFile.println("Servos being commanded to 0 degrees\n");
  dataFile.println("Check slots on rocket- Make sure fins will deploy\n");
  
  while (Serial.available() == false) {
    now = millis();
    dataFile.println("Time:");
    dataFile.println(now);
    Serial.println("Time:");
    Serial.println(now);
    Serial.print("Are the fins clear of the slots? Type 'y'\n");
    dataFile.println("Are the fins clear of the slots? Type 'y'\n");
    delay(1000);
  }
  if (Serial.available()) {
      // Read command
      byte straight = Serial.read ();
      now = millis();
      if (straight == 'y'){
        dataFile.println("Time:");
        dataFile.println(now);
        Serial.println("Time:");
        Serial.println(now);
        Serial.println("Flaps confirmed straight. Continuing...\n");
        dataFile.println("Flaps confirmed straight. Continuing...\n");
      }
      else {
        dataFile.println("Time:");
        dataFile.println(now);
        Serial.print("Type 'y' if they are good. Starting over...\n");
        dataFile.println("Type 'y' if they are good. Starting over...\n");
        begin_again();
      }
  }
  dataFile.close();
  delay(1000);
}

void collect_actuate(){
  File dataFile = SD.open("Data.txt", FILE_WRITE);
  while (count < sizeof(actuate_vals)/sizeof(int)){
    now = millis();
    dataFile.println("Time:");
    dataFile.println(now);
    Serial.println("Time:");
    Serial.println(now);
    collect_delay();
    Serial.print("count is "+String(count)+"\n");
    dataFile.println("count is "+String(count)+"\n");
    actuate();
  } 
  now = millis();
  dataFile.println("Time: " + now);
  Serial.print("Done with actuation sequence!\n");
  dataFile.println("Done with actuation sequence!\n");
  dataFile.close();
}

void collect_delay(){
  now = millis();
  start = millis();
  while (now-start < delay_vals[count]){
    collect_data();
    now = millis();
  }
}

void actuate(){
  servo_1.write(actuate_vals[count]);
  servo_2.write(actuate_vals[count]);
  count += 1;
  collect_data();
}

void collect_data(){
  now = millis();
  File dataFile = SD.open("Data.txt", FILE_WRITE);
  MadgwickAHRSupdate();
  psi = atan2((2 * q1 * q2) - (2 * q0 * q3), (2 * pow(q0, 2)) + (2 * pow(q1, 2)) - 1);
  psi *= 180.0/PI;
  if (psi < 0) psi+=360;
  dataFile.println("Time: " + now);
  Serial.println(psi);
  dataFile.println(psi);
  dataFile.close();
  
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

void initialize_SD_card() {
   Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(53, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed, pin not connected");
    return;
  }
  Serial.println("SD card initialized.");
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

