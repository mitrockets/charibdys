#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Simple_AHRS.h>

Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

float roll_blue;

void setup()
{
  Serial.begin(115200);

  // Initialize the sensors.
  accel.begin();
  mag.begin();
  bmp.begin();
}

void loop(void)
{
  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    roll_blue = orientation.heading;
    if (roll_blue < 0) roll_blue = roll_blue + 360;
    Serial.print(roll_blue);
    Serial.println(F(""));
    delay(10);
  }
}
  
