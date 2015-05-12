/*
This is to test the collect detect method by itself
It is supposed to detect when launch has occurred
Using 9dof
*/
#include <SPI.h> 
#include <Wire.h>
#include <PID_v4.h>
#include <SFE_LSM9DS0.h>
#include <Servo.h>

#define LSM9DS0_XM 0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G 0x6B // Would be 0x6A if SDO_G is LOW
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  collect_detect();
}

void collect_detect() {
  float ax = dof9.calcAccel(dof9.ax);
  float ay = dof9.calcAccel(dof9.ay);
  float az = dof9.calcAccel(dof9.az);
  acceleration = sqrt(ax * ax + ay * ay + az * az);
  while (acceleration < launch_accel) {
    if (Serial.available()) {
      begin_again();
      break;
    }
    collect_9dof_data();
    save_string("INFO, will start time once 3gs have been detected\n");
  }
}
