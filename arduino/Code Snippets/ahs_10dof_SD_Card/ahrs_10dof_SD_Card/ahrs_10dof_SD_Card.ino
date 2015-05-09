#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <SFE_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

// Create sensor instances.
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

// Update this with the correct SLP for accurate altitude measurements
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

/* microSD save data*/
File collected_data;
bool print_data = true;
char fileName[] = "arduino_test_data_5_6_15.csv";

void setup()
{
  Serial.begin(9600);
  Serial.println(F("Adafruit 10 DOF Board AHRS Example")); Serial.println("");
  
  initialize_SD_card();
  save_string("INFO, Arduino boot\n");
  
  // Initialize the sensors.
  accel.begin();
  mag.begin();
  bmp.begin();
}

void loop(void)
{
  double IMU_values[5];
  
  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.heading);
    Serial.println(F(""));
    IMU_values[0] = orientation.roll;
    IMU_values[1] = orientation.pitch;
    IMU_values[2] = orientation.heading;
  }

  // Calculate the altitude using the barometric pressure sensor
  sensors_event_t bmp_event;
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    /* Convert atmospheric pressure, SLP and temp to altitude */
    //Serial.print(F("Alt: "));
    //Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                       // bmp_event.pressure,
                                       // temperature)); 
                                  
    Serial.println(F(""));
    /* Display the temperature */
    //Serial.print(F("Temp: "));
    //Serial.print(temperature);
    Serial.println(F(""));
    IMU_values[3] = bmp.pressureToAltitude(seaLevelPressure,
                                        bmp_event.pressure,
                                        temperature);
    IMU_values[4] = temperature;
    Serial.print("IMU_values list: ");
    for (int i = 0; i<5; i++) {
    Serial.print(IMU_values[i]);
    Serial.println(F(""));
    }                         
  }
    save_array(IMU_values, 5);
  delay(2000);
}

void initialize_SD_card() {
   Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(10, OUTPUT);

  if (!SD.begin(10)) {
    Serial.println("initialization failed, pin not connected");
    return;
  }
  Serial.println("SD card initialized.");
 }
 
void save_string(String string_to_save)
 {
   collected_data = SD.open(fileName, FILE_WRITE);
   
   unsigned long time = millis();
   
   collected_data.print(time);
   collected_data.println(" " + string_to_save);
   
   if (print_data)
   {
     Serial.print("This string is being printed to the SD card: ");
     Serial.println(string_to_save);
   }
   
   //SD.remove("test.txt");
   
   collected_data.close();
 }
   
 void save_array(double array_to_save[], int size_of_array)
 {
   collected_data = SD.open(fileName, FILE_WRITE);
   
   unsigned long time = millis();
   
   collected_data.print(time);
   collected_data.print(" ");

   int i;
     for (i=0; i < size_of_array; i++) {
       collected_data.print(array_to_save[i]);
       collected_data.print(" ");
       if (print_data){
         Serial.print("-> SD: ");
         Serial.print(array_to_save[i]);
         Serial.print(" ");
       }
     }
      
     collected_data.println(" ");
     //Serial.println();
   /*
   if (print_data)
   {
     int i;
     for (i=0; i < 4; i++) {
       Serial.print(array_to_save[i]);
       Serial.println(" ");
     }
   }
  */ 
   
   collected_data.close();
 }
  void read_file() 
 {
    collected_data = SD.open(fileName);
  Serial.println("This is on the SD card:");

  // read from the file until there's nothing else in it:
  while (collected_data.available()) {
    Serial.write(collected_data.read());
  }
  // close the file:
  collected_data.close();
  
}
  
void remove_data_file()
{
  SD.remove(fileName);
}
