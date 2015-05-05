#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_L3GD20_U.h>

Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);


float *get_10DOF_data() {
  /*sensors_event_t accel_event_one;
  sensors_vec_t orientation_one;*/
  float IMU_values[7];

/* Calculate pitch and roll from the raw accelerometer data */
/*accel.getEvent(&accel_event_one);*/

/*
if (dof.accelGetOrientation(&accel_event_one, &orientation_one))
{
  //'orientation' should have valid .roll and .pitch fields 
  //Serial.print(F("Roll: "));
  //Serial.print(orientation_one.roll); 
  IMU_values[0] = orientation_one.roll;
  Serial.print(F("; "));
  Serial.print(F("Pitch: "));
  Serial.print(orientation_one.pitch); 
  IMU_values[1] = orientation_one.pitch;
  Serial.print(F("; "));
}

sensors_event_t mag_event_one;
*/
/* Calculate the heading using the magnetometer */
//mag.getEvent(&mag_event_one);
/*
if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event_one, &orientation_one))
{
  //'orientation' should have valid .heading data now
 // Serial.print(F("Heading: "));
 // Serial.print(orientation_one.heading);
  IMU_values[2] = orientation_one.heading;
  Serial.print(F("; "));
}
*/

/*this applies to the 10DOF. Delete all else later*/

sensors_event_t event_two; 
sensors_vec_t orientation_two;
accel.getEvent(&event_two);
 
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("Acceleration X: "); Serial.print(event_two.acceleration.x); Serial.print("  ");
  Serial.print("Acceleration Y: "); Serial.print(event_two.acceleration.y); Serial.print("  ");
  Serial.print("Acceleration Z: "); Serial.print(event_two.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  IMU_values[0] = event_two.acceleration.x;
  IMU_values[1] = event_two.acceleration.y;
  IMU_values[2] = event_two.acceleration.z;
  
    /* Get a new sensor event */ 
  sensors_vec_t mag_event;
  mag.getEvent(&mag_event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("Mag X: "); Serial.print(event_two.magnetic.x); Serial.print("  ");
  Serial.print("Mag Y: "); Serial.print(event_two.magnetic.y); Serial.print("  ");
  Serial.print("Mag Z: "); Serial.print(event_two.magnetic.z); Serial.print("  ");Serial.println("uT");
  IMU_values[3] = event_two.magnetic.x;
  IMU_values[4] = event_two.magnetic.y;
  IMU_values[5] = event_two.magnetic.z;
  
  
    /* Get a new sensor event */ 
  bmp.getEvent(&event_two);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event_two.pressure)
  {
    /* Display atmospheric pressure in hPa */
    Serial.print("Pressure: "); Serial.print(event_two.pressure); Serial.println(" hPa");
    IMU_values[6] = event_two.pressure;
  }
  else
  {
    Serial.println("Sensor error");
  }
  return IMU_values;
 }
