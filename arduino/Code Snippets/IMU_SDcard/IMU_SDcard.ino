#include <SPI.h> 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <SFE_LSM9DS0.h>
#include <SPI.h>
#include <SdFat.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

// SD chip select pin
const uint8_t chipSelect = SS;
// file system object
SdFat sd;
// create Serial stream
ArduinoOutStream cout(Serial);
char fileName[] = "rocket_team_launch_data_4_11_15.csv";
//------------------------------------------------------------------------------
// store error strings in flash to save RAM
#define error(s) sd.errorHalt(F(s))

unsigned long StartTime = millis();
float maxAccel;

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

// write test file
void writeFile(String stringIn) {
  int stringLength = stringIn.length();
  char data [stringLength];
  stringIn.toCharArray(data, stringLength);
  
  // create or open and truncate output file
  ofstream sdout(fileName, O_APPEND);

  // write file from string stored in flash
  sdout << data << flush;
  sdout << F("\n") << flush;

  // check for any errors
  if (!sdout) {
    error("writeFile");
  }

  sdout.close();
}

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Accelerometer Test"); Serial.println("");
  uint16_t status = dof.begin(dof.G_SCALE_2000DPS, 
                              dof.A_SCALE_8G, dof.M_SCALE_2GS);
    
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  
  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance
 if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    sd.initErrorHalt();
  }

  // create test file
  writeFile("Test write. It's 4AM and Johannes and Erick and Lisandro are still grinding\n");
}

void loop(void) 
{
  unsigned long CurrentTime = millis();
  unsigned long ElapsedTime = CurrentTime - StartTime;
  if(ElapsedTime>1000){
    StartTime = millis();
    Serial.println("Try harder...");
  }
  
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);
  dof.readAccel();
  
  float accels10 [3] = {event.acceleration.x, event.acceleration.y, event.acceleration.z};
  float accels9 [3] = {dof.calcAccel(dof.ax),dof.calcAccel(dof.ay),dof.calcAccel(dof.az)};
  writeFile("Current Time: " + String(CurrentTime));
//  Serial.print(event.acceleration.x, 2);
//  Serial.print(", ");
//  Serial.print(event.acceleration.y, 2);
//  Serial.print(", ");
//  Serial.println(event.acceleration.z, 2);
//  Serial.print(dof.calcAccel(dof.ax), 2);
//  Serial.print(", ");
//  Serial.print(dof.calcAccel(dof.ay), 2);
//  Serial.print(", ");
//  Serial.println(dof.calcAccel(dof.az), 2);
  for(int i = 0;i<=2;i++){
    if(abs(accels9[i]>maxAccel)){
      newMax(accels9[i]);
    }
  }
}

void newMax(float newMaxAccel)
{
  maxAccel = newMaxAccel;
  Serial.print("New max: ");
  Serial.println(maxAccel);
}
