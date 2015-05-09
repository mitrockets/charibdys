#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_L3GD20_U.h>

Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel10 = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag10   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
//Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified();

float *get_10DOF_data(double IMU_values[]) {
  accel10.begin();
  mag10.begin();
  bmp.begin();

  sensors_event_t event; 
  sensors_vec_t orientation;
  
  // Accelerometer data
  accel10.getEvent(&event);
  Serial.print("Acceleration Data:");
  IMU_values[0] = event.acceleration.x;
  IMU_values[1] = event.acceleration.y;
  IMU_values[2] = event.acceleration.z;
  
  // Magnetometer data
  mag10.getEvent(&event);
  Serial.print("Mag Data:");
  IMU_values[3] = event.magnetic.x;
  IMU_values[4] = event.magnetic.y;
  IMU_values[5] = event.magnetic.z;
  
  // Pressure data
  bmp.getEvent(&event);
  Serial.print("Pressure Data:");
  IMU_values[6] = event.pressure;
  
  // Gyroscope Data
//  gyro.getEvent(&event);
//  Serial.print(event.gyro.x);
//  Serial.print(event.gyro.y);
//  Serial.print(event.gyro.z);
//  IMU_values[7] = event.gyro.x;
//  IMU_values[8] = event.gyro.y;
//  IMU_values[9] = event.gyro.z;
  
  //Orientation
    /* 'orientation' should have valid .roll and .pitch fields */
  Serial.print(F("Orientation: "));
  Serial.print(orientation.roll);
  Serial.print(F(" "));
  Serial.print(orientation.pitch);
  Serial.print(F(" "));
  Serial.print(orientation.heading);
  Serial.println(F(""));
  IMU_values[10] = orientation.roll;
  IMU_values[11] = orientation.pitch;
  IMU_values[12] = orientation.heading; 
  
//  gyro.read();
//  Serial.print("X: "); Serial.print((int)gyro.data.x);   Serial.print(" ");
//  Serial.print("Y: "); Serial.print((int)gyro.data.y);   Serial.print(" ");
//  Serial.print("Z: "); Serial.println((int)gyro.data.z); Serial.print(" ");
  
  Serial.print(*IMU_values);
 }
