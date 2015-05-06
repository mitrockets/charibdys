#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_L3GD20_U.h>

Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);


float *get_10DOF_data() {
  float IMU_values[7];

  sensors_event_t event; 
  sensors_vec_t orientation;
  
  // Accelerometer data
  accel.getEvent(&event);
  IMU_values[0] = event.acceleration.x;
  IMU_values[1] = event.acceleration.y;
  IMU_values[2] = event.acceleration.z;
  
  // Magnetometer data
  sensors_vec_t mag_event;
  mag.getEvent(&mag_event);
 
  IMU_values[3] = event_two.magnetic.x;
  IMU_values[4] = event_two.magnetic.y;
  IMU_values[5] = event_two.magnetic.z;
  
  // Pressure data
  bmp.getEvent(&event);
  IMU_values[6] = event_two.pressure;

  return &IMU_values;
 }
