/*
This code includes both IMU's (red 99dof and blue 10dof)
It prints the data to Serial monitor side by side
Copy paste into Excel
Excel has a parsing feature (go to Data and then text to column)
Graph data on same graph- Use to calibrate
*/
#include <SPI.h> 
#include <Wire.h>
#include <PID_v4.h>
#include <SFE_LSM9DS0.h>
#define LSM9DS0_XM 0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G 0x6B // Would be 0x6A if SDO_G is LOW
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Simple_AHRS.h>

Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

float roll_blue;
float A = 1;
float total = 0;
const int index = 30;
float average = 0;
float roll_avg[index];
float x, y, z, roll_mag, smoothData1;

const byte INT1XM = 9; // INT1XM tells us when accel data is ready
const byte INT2XM = 8; // INT2XM tells us when mag data is ready
const byte DRDYG = 4;  // DRDYG tells us when gyro data is ready

void setup() {
  Serial.begin(115200); // Start serial at 115200 bps
  
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG, INPUT);
  
  uint16_t status = dof.begin();
  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println();
  accel.begin();
  mag.begin();
  bmp.begin();
}

void loop() {
  //Read values
  dof.readMag();
  x = dof.calcMag(dof.mx);
  y = dof.calcMag(dof.my);
  z = dof.calcMag(dof.mz);
//Obtain roll_mag (Roll as determined only from magnetometer)
  roll_mag = atan2(y, x);
  roll_mag *= 180.0 / PI;//minus sign added to match IMU with Blue
  if (roll_mag < 0) roll_mag = roll_mag + 360;
  roll_mag = roll_mag;////////////////////////////////////////////////////////////////////////
//Filter roll_mag
  roll_avg[index] = roll_mag;
  smoothData1 = digitalSmooth(roll_mag, roll_avg);
//Send data to Serial Monitor
//  Serial.print(" RED Sensor: ");
//  Serial.println(roll_mag);
  
  
  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    roll_blue = orientation.heading;
    if (roll_blue < 0) roll_blue = roll_blue + 360;
    Serial.print(" BLUE Sensor ");
    Serial.println(roll_blue);
  }  0, 320, 350, 31,
  delay(10);
}

float digitalSmooth(float rawIn, float *sensSmoothArray){
  int j, k, temp, top, bottom;
  long total;
  static int i;
  static int sorted[index];
  boolean done;
  
  i = (i + 1) % index;
  sensSmoothArray[i] = rawIn;
  
  for (j = 0; j<index; j++){
    sorted[j] = sensSmoothArray[j];
  }
  done = 0;
  while(done != 1){
    done = 1;
    for (j = 0; j < (index - 1); j++){
      if (sorted[j] > sorted[j + 1]){
        temp = sorted[j +1];
        sorted [j+1] = sorted[j];
        sorted [j] = temp;
        done = 0;
      }
    }
  }
  //throw out top and bottom 15% of samples
  bottom = max(((index*15)/100), 1);
  top = min ((((index*85)/100) + 1), (index - 1));
  k = 0;
  total = 0;
  for (j = bottom; j < top; j++){
    total += sorted[j];
    k++;
  }
  return total/k;
}
